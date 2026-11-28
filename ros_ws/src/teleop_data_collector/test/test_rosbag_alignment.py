"""Closed SQLite bag calibration and task snapshots; no devices or ROS graph."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest

pytest.importorskip('rclpy')
import rosbag2_py
from rclpy.serialization import serialize_message,deserialize_message
from sensor_msgs.msg import Image,CameraInfo
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from teleop_data_collector.rosbag_calibration import RosbagCalibration,append_snapshots,TF_TOPIC,COLOR_INFOS
from teleop_data_collector.calibrated_tf import transform_matrix


def calibration(tmp_path):
    import yaml
    specs=[]
    for camera,side in [('cam1','right'),('cam2','left')]:
        matrix=np.eye(4);matrix[0,3]=.1 if side=='left' else -.1
        specs.append(dict(parent_frame=f'{side}_fr3_link8',child_frame=f'{camera}_color_optical_frame',matrix=matrix.tolist()))
    profile=tmp_path/'profile.yaml';profile.write_text(yaml.safe_dump(dict(static_extrinsics=specs)))
    callbacks={}
    node=NS(create_subscription=lambda cls,topic,callback,*a,**kw:callbacks.setdefault(topic,callback),destroy_subscription=lambda _:None)
    helper=RosbagCalibration(node,profile)
    static=TFMessage()
    for index in range(3):
        edge=TransformStamped();edge.header.frame_id=f'cam{index}_link';edge.child_frame_id=f'cam{index}_color_optical_frame'
        edge.transform.rotation.w=1.;edge.transform.translation.z=.01;static.transforms.append(edge)
    callbacks[TF_TOPIC](static)
    infos={}
    for topic in COLOR_INFOS:
        info=CameraInfo();info.header.stamp.sec=2;info.header.frame_id=topic.split('/')[1]+'_color_optical_frame'
        info.width=640;info.height=480;info.k=[100.,0.,320.,0.,100.,240.,0.,0.,1.]
        infos[topic]=info;callbacks[topic](info)
    return helper,callbacks,static,infos,specs


def test_closed_bag_snapshots_preserve_native_samples_and_metadata(tmp_path):
    helper,callbacks,static,infos,specs=calibration(tmp_path)
    assert helper.readiness()['ready']
    snapshot=helper.capture(3_000_000_000)
    # Later device/Home changes cannot alter the frozen start snapshot.
    mutated=copy.deepcopy(infos[COLOR_INFOS[0]]);mutated.width=99;callbacks[COLOR_INFOS[0]](mutated)
    bag=tmp_path/'episode0';writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    for topic,kind in [(TF_TOPIC,'tf2_msgs/msg/TFMessage'),('/cam0/color/image_raw','sensor_msgs/msg/Image'),('/tf','tf2_msgs/msg/TFMessage')]:
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic,type=kind,serialization_format='cdr'))
    image=Image();image.header.stamp.sec=3;image.header.stamp.nanosec=100_000_000;image.width=1;image.height=1;image.encoding='rgb8';image.step=3;image.data=b'abc'
    original=serialize_message(image)
    writer.write('/cam0/color/image_raw',original,3_100_000_000)
    writer.write(TF_TOPIC,serialize_message(static),3_150_000_000)
    writer.write('/tf',serialize_message(static),3_160_000_000)
    del writer
    append_snapshots(bag,snapshot)
    append_snapshots(bag,snapshot)  # A retry must not duplicate the start records.
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    rows=[]
    while reader.has_next():rows.append(reader.read_next())
    assert len(rows)==7
    color=next(row for row in rows if row[0]=='/cam0/color/image_raw')
    assert color[1]==original and color[2]==3_100_000_000
    info_row=next(row for row in rows if row[0]==COLOR_INFOS[0])
    info=deserialize_message(info_row[1],CameraInfo)
    assert info.width==640 and info.header.stamp.sec==2 and info_row[2]==3_000_000_000
    for topic,data,stamp in rows:
        if topic!=TF_TOPIC:continue
        message=deserialize_message(data,TFMessage)
        edges={edge.child_frame_id:edge for edge in message.transforms}
        for camera,side in [('cam1','right'),('cam2','left')]:
            assert edges[camera+'_link'].header.frame_id==side+'_fr3_link8'
            assert edges[camera+'_link'].transform.translation.z==pytest.approx(-.01)
    dynamic=next(row for row in rows if row[0]=='/tf')
    assert deserialize_message(dynamic[1],TFMessage)==static
    import yaml
    metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    assert metadata['message_count']==len(rows)
    assert metadata['starting_time']['nanoseconds_since_epoch']==3_000_000_000
    assert sum(row['message_count'] for row in metadata['topics_with_message_count'])==len(rows)
    assert json.loads((bag/'wrist_extrinsics.json').read_text())==dict(static_extrinsics=specs)
    helper.close()


def test_rosbag_only_configuration_adds_calibration_topics():
    from teleop_data_collector.config import CollectorConfig,TopicConfig
    from teleop_data_collector.recording_backend import select_config
    config=CollectorConfig('/tmp/test',(TopicConfig('rgb','/cam0/color/image_raw','sensor_msgs/msg/Image'),),(),0.,0.,{})
    options=dict(head_depth=False,wrist_depth=False,tactile=False)
    rosbag=select_config(config,dict(format='rosbag',**options))
    mcap=select_config(config,dict(format='mcap',**options))
    assert {t.topic for t in rosbag.static_topics}=={TF_TOPIC,'/tf',*COLOR_INFOS}
    assert not mcap.static_topics


def test_task_metadata_is_frozen_and_saved_for_rosbag(tmp_path,monkeypatch):
    from teleop_data_collector import rosbag_recording_node as recording
    logger=NS(info=lambda _:None,warn=lambda _:None,error=lambda _:None)
    node=NS(get_logger=lambda:logger,get_clock=lambda:NS(now=lambda:NS(nanoseconds=3_000_000_000)))
    process=NS(poll=lambda:None,wait=lambda **kw:0,returncode=0)
    monkeypatch.setattr(recording.shutil,'which',lambda _:'/ros2')
    monkeypatch.setattr(recording.subprocess,'Popen',lambda *a,**kw:process)
    monkeypatch.setattr(recording,'_signal_process_group',lambda *a:None)
    recorder=recording.RosbagEpisodeRecorder(node,tmp_path,'episode',(),30.,(),postflight=lambda _:((),{}))
    recorder.task_metadata=dict(task_id='assembly',task_name='装配',task_description='Assemble parts.')
    bag=recorder.start();bag.mkdir();recorder.task_metadata['task_description']='Later edit'
    recorder.stop()
    state=json.loads((bag/'collection_state.json').read_text())
    assert state['task_metadata']['task_description']=='Assemble parts.'
    assert state['finalized'] is True
