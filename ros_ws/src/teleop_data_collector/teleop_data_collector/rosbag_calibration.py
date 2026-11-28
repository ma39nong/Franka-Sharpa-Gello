"""Rosbag-only calibration snapshots, appended after the CLI recorder closes.

The existing ros2 bag record subscriptions, QoS and 256 MiB cache are unchanged.
No TF is published to the live graph. Snapshot CDR and its receipt time are frozen
at start; only the physical SQLite insertion is deferred until finalization.
"""
from copy import deepcopy
from pathlib import Path
import os
import sqlite3
import tempfile
import threading
import yaml
from .calibrated_tf import merge_static, validate_specs
from .mcap_util import write_json

TF_TOPIC='/tf_static'
TF_TYPE='tf2_msgs/msg/TFMessage'
COLOR_INFOS=tuple(f'/cam{i}/color/camera_info' for i in range(3))


def plain(message):
    if hasattr(message,'get_fields_and_field_types'):
        return {key:plain(getattr(message,key)) for key in message.get_fields_and_field_types()}
    if isinstance(message,(str,int,float,bool,bytes)):return message
    return [plain(value) for value in message]


def calibrated_message(edges,specs,stamp):
    from tf2_msgs.msg import TFMessage
    from rosidl_runtime_py.set_message import set_message_fields
    message=TFMessage()
    set_message_fields(message,merge_static({'transforms':list(edges.values())},specs,stamp))
    return message


class RosbagCalibration:
    def __init__(self,node,profile_path):
        path=Path(profile_path)
        if not path.is_file():raise ValueError('Rosbag wrist calibration profile missing: '+str(path))
        profile=yaml.safe_load(path.read_text()) or {}
        self.specs=deepcopy(profile.get('static_extrinsics',[]))
        validate_specs(self.specs)
        if not all(any(spec['child_frame'].startswith(camera+'_') for spec in self.specs) for camera in ('cam1','cam2')):
            raise ValueError('Rosbag requires calibrated cam1/cam2 wrist extrinsics')
        self.node=node;self.lock=threading.Lock();self.edges={};self.infos={};self.subscriptions=[]
        from tf2_msgs.msg import TFMessage
        from sensor_msgs.msg import CameraInfo
        from rclpy.qos import QoSProfile,DurabilityPolicy
        def static(message):
            with self.lock:
                for transform in message.transforms:self.edges[transform.child_frame_id]=plain(transform)
        self.subscriptions.append(node.create_subscription(TFMessage,TF_TOPIC,static,
            QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        for topic in COLOR_INFOS:
            def info(message,topic=topic):
                with self.lock:self.infos[topic]=deepcopy(message)
            self.subscriptions.append(node.create_subscription(CameraInfo,topic,info,10))

    def readiness(self):
        with self.lock:
            missing=[topic for topic in COLOR_INFOS if topic not in self.infos]
            if not self.edges:missing.append(TF_TOPIC)
        return dict(ready=not missing,message='等待 rosbag 标定快照：'+', '.join(missing) if missing else '')

    def capture(self,stamp_ns):
        from rclpy.serialization import serialize_message
        with self.lock:
            missing=[topic for topic in COLOR_INFOS if topic not in self.infos]
            if not self.edges:missing.append(TF_TOPIC)
            if missing:raise RuntimeError('等待 rosbag 标定快照：'+', '.join(missing))
            edges=deepcopy(self.edges);infos=dict(self.infos)
        messages={topic:serialize_message(message) for topic,message in infos.items()}
        messages[TF_TOPIC]=serialize_message(calibrated_message(edges,self.specs,stamp_ns))
        return dict(stamp_ns=stamp_ns,messages=messages,source_static=edges,specs=deepcopy(self.specs))

    def close(self):
        for sub in self.subscriptions:self.node.destroy_subscription(sub)
        self.subscriptions=[]


def append_snapshots(bag_dir,snapshot):
    """Update only closed SQLite bags and their native rosbag2 metadata."""
    from rclpy.serialization import serialize_message,deserialize_message
    from tf2_msgs.msg import TFMessage
    bag=Path(bag_dir);metadata_path=bag/'metadata.yaml'
    metadata=yaml.safe_load(metadata_path.read_text())
    info=metadata['rosbag2_bagfile_information']
    if info['storage_identifier']!='sqlite3':raise ValueError('Calibration append requires a closed sqlite3 rosbag')
    start_key='nanoseconds_since_epoch' if 'nanoseconds_since_epoch' in info.get('starting_time',{}) else 'nanoseconds'
    edges=deepcopy(snapshot['source_static']);counts={};types={};file_reports=[];all_starts=[];all_ends=[]
    for index,relative in enumerate(info['relative_file_paths']):
        path=bag/relative
        if not path.is_file() or path.suffix!='.db3':raise ValueError('Unsupported or missing closed rosbag file: '+str(path))
        with sqlite3.connect(path) as db:
            db.execute('PRAGMA synchronous=FULL')
            topic_rows={row[1]:row for row in db.execute('SELECT id,name,type,serialization_format,offered_qos_profiles FROM topics')}
            row=topic_rows.get(TF_TOPIC)
            if row:
                if row[2]!=TF_TYPE:raise ValueError('Recorded /tf_static has unexpected type')
                # Preserve incoming edges, applying the same wrist calibration as DAgger
                # to every recorded static snapshot, not merely the injected first one.
                for message_id,data,stamp in db.execute('SELECT id,data,timestamp FROM messages WHERE topic_id=? ORDER BY timestamp,id',(row[0],)).fetchall():
                    incoming=deserialize_message(bytes(data),TFMessage)
                    for transform in incoming.transforms:edges[transform.child_frame_id]=plain(transform)
                    patched=serialize_message(calibrated_message(edges,snapshot['specs'],stamp))
                    db.execute('UPDATE messages SET data=? WHERE id=?',(patched,message_id))
            if index==0:
                for topic,payload in snapshot['messages'].items():
                    kind=TF_TYPE if topic==TF_TOPIC else 'sensor_msgs/msg/CameraInfo'
                    row=topic_rows.get(topic)
                    if row and row[2]!=kind:raise ValueError('Snapshot topic has unexpected type: '+topic)
                    if not row:
                        columns={entry[1] for entry in db.execute('PRAGMA table_info(topics)')}
                        values=dict(name=topic,type=kind,serialization_format='cdr',offered_qos_profiles='')
                        if 'type_description_hash' in columns:values['type_description_hash']=''
                        names=list(values)
                        cursor=db.execute('INSERT INTO topics ('+','.join(names)+') VALUES ('+','.join('?' for _ in names)+')',tuple(values[key] for key in names))
                        row=(cursor.lastrowid,topic,kind,'cdr','');topic_rows[topic]=row
                    exists=db.execute('SELECT 1 FROM messages WHERE topic_id=? AND timestamp=? LIMIT 1',
                                      (row[0],snapshot['stamp_ns'])).fetchone()
                    if not exists:db.execute('INSERT INTO messages (topic_id,timestamp,data) VALUES (?,?,?)',(row[0],snapshot['stamp_ns'],payload))
            per_topic=dict(db.execute('SELECT topic_id,COUNT(*) FROM messages GROUP BY topic_id'))
            for topic_id,name,kind,serialization,qos in topic_rows.values():
                count=per_topic.get(topic_id,0)
                counts[name]=counts.get(name,0)+count;types[name]=(kind,serialization,qos)
            total,first,last=db.execute('SELECT COUNT(*),MIN(timestamp),MAX(timestamp) FROM messages').fetchone()
            if first is not None:all_starts.append(first);all_ends.append(last)
            file_reports.append(dict(path=relative,starting_time={start_key:first or 0},
                duration={'nanoseconds':max(0,(last or 0)-(first or 0))},message_count=total))
    previous={item['topic_metadata']['name']:item for item in info['topics_with_message_count']}
    for topic,count in counts.items():
        if topic not in previous:
            kind,serialization,qos=types[topic]
            previous[topic]=dict(topic_metadata=dict(name=topic,type=kind,serialization_format=serialization,offered_qos_profiles=qos))
        previous[topic]['message_count']=count
    info['topics_with_message_count']=list(previous.values())
    info['message_count']=sum(counts.values())
    info['starting_time']={start_key:min(all_starts) if all_starts else 0}
    info['duration']={'nanoseconds':max(all_ends)-min(all_starts) if all_starts else 0}
    if 'files' in info:info['files']=file_reports
    for relative in info['relative_file_paths']:
        with (bag/relative).open('rb') as stream:os.fsync(stream.fileno())
    temporary=None
    try:
        with tempfile.NamedTemporaryFile(mode='w',encoding='utf-8',dir=bag,prefix='.calibration-metadata-',delete=False) as stream:
            temporary=Path(stream.name);yaml.safe_dump(metadata,stream,sort_keys=False);stream.flush();os.fsync(stream.fileno())
        temporary.replace(metadata_path)
    finally:
        if temporary:temporary.unlink(missing_ok=True)
    write_json(bag/'wrist_extrinsics.json',dict(static_extrinsics=snapshot['specs']))
    descriptor=os.open(bag,os.O_RDONLY|os.O_DIRECTORY)
    try:os.fsync(descriptor)
    finally:os.close(descriptor)
