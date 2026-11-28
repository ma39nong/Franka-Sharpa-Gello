"""ROS read-only MCAP backend implementing the existing episode recorder interface."""
from pathlib import Path
import json
import tempfile
import time
import uuid
from .rosbag_recording_node import RosbagEpisodeRecorder
from .mcap_capture import Capture
from .mcap_profile import load_profile
from .mcap_storage import seal
from .mcap_util import write_json


class McapEpisodeRecorder(RosbagEpisodeRecorder):
    def __init__(self, *args, profile_path, options, **kwargs):
        super().__init__(*args, **kwargs)
        self.profile,self.channels=load_profile(profile_path,options,self._provenance.get('device_identities'))
        from copy import deepcopy
        self.profile['teleop_provenance']=deepcopy(self._provenance)
        self.options=options;self.subscriptions=[];self._temporary=tempfile.TemporaryDirectory(prefix='gello-mcap-')
        from rclpy.serialization import serialize_message
        self.capture=Capture(self._temporary.name,self.profile,self.channels,serialize=serialize_message)
        from rosidl_runtime_py.utilities import get_message
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        types={ch['source_topic']:get_message(ch.get('source_type',ch['type'])) for ch in self.channels}
        for ch in self.channels:
            qos=QoSProfile(depth=30,reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL if ch['source_topic']=='/tf_static' else DurabilityPolicy.VOLATILE)
            def receive(msg,channel=ch):
                capture=self.capture
                with capture.lock:capture.accept(channel,msg,time.time_ns())
            self.subscriptions.append(self._node.create_subscription(types[ch['source_topic']],ch['source_topic'],receive,qos))

    @property
    def active(self):
        return self.capture.active

    def readiness(self):
        health=self.capture.health()
        return dict(ready=health['ready'],message='' if health['ready'] else 'MCAP等待所选传感器/CameraInfo/TF；未启动的传感器在下次正常启动后生效：'+json.dumps(health,ensure_ascii=False))

    def start(self, *, archive_parent=None):
        if self.active:raise RuntimeError('MCAP recording already active')
        failures=tuple(self._preflight() if self._preflight else ())
        if failures:raise RuntimeError('Recording preflight failed: '+'; '.join(failures))
        if not self.capture.readiness():raise RuntimeError(self.readiness()['message'])
        if self.capture.finished.is_set():
            from rclpy.serialization import serialize_message
            old=self.capture
            capture=Capture(self._temporary.name,self.profile,self.channels,serialize=serialize_message)
            with old.lock:
                capture.static=dict(old.static);capture.static_tf_edges=dict(old.static_tf_edges);capture.receipts=dict(old.receipts)
                self.capture=capture
        target=self._next_episode_dir(archive_parent)
        self._milestones=[];self._validation_report={};self._live_continuity_report={}
        self._source_recording_id=uuid.uuid4().hex
        from .mcap_util import recorder_commit
        commit=recorder_commit()
        task=getattr(self,'task_metadata',{})
        self.capture.target=target;self.capture.root=archive_parent or target.parent
        self.capture.identity=dict(episode_id=target.name,episode_uid=uuid.uuid4().hex,
            session_id=self.profile['session_id'],recording_id=self._source_recording_id,
            task_id=task.get('task_id','teleoperation'),task_description=task.get('task_description') or task.get('task_name') or 'Teleoperation',
            source_type='teleoperation',source_id='gello',end_reason='operator stop',recorder_commit=commit,
            action_storage_layout='four-joint-state-command-streams-v2',action_semantics=dict(arms='gateway_validated_target',hands='sdk_submitted_target'))
        try:self.capture.request('start')
        except Exception:
            self.capture.close()
            raise
        self._current_bag_dir=target;self._last_bag_dir=target
        return target

    def stop(self, *, interrupted=False):
        bag=self._current_bag_dir
        if bag is None:return None
        failures=list(self._live_continuity_report.get('failures',[]))
        if interrupted:failures.append('recording interrupted')
        if failures:self.capture.error='; '.join(failures)
        self.capture.close()
        self._current_bag_dir=None
        self.on_recording_stopped()
        if self.capture.error:failures.append(self.capture.error)
        if not bag.is_dir() and self.capture.writer:
            # Preserve failed partial evidence; never claim a complete episode.
            partial=self.capture.writer.directory
            if partial.exists() and not bag.exists():partial.rename(bag)
        self._write_state(bag,state='incomplete' if failures else 'finalized',finalized=not failures,failures=failures)
        return bag

    def _write_state(self,bag_dir,*,state,finalized,failures,validation_report=None):
        path=bag_dir/'episode.json'
        if not path.exists():return
        document=json.loads(path.read_text())
        container_complete=finalized if state!='discarded' else document.get('collection',{}).get('container_complete',False)
        if failures and state!='discarded':document['end_reason']='; '.join(map(str,failures))
        document['collection']=dict(state=state,finalized=finalized,container_complete=container_complete,failures=list(dict.fromkeys(failures)),
            source_recording_id=self._source_recording_id,milestones=self.milestones,
            live_continuity=self._live_continuity_report,auto_stop_reason=self._live_continuity_report.get('auto_stop_reason'),
            recording_options=self.options,owner='gello_upper_body_teleop',updated_at_ns=time.time_ns())
        write_json(path,document)
        seal(bag_dir,'complete' if container_complete else 'failed')

    def close(self):
        for sub in self.subscriptions:self._node.destroy_subscription(sub)
        self.subscriptions=[]
        self.capture.close();self._temporary.cleanup()
