"""Format selection and per-selection source contracts around independent backends."""
from dataclasses import replace
from pathlib import Path
from .config import TopicConfig
from .rosbag_recording_node import RosbagEpisodeRecorder
from .recording_options import optional_channels


def select_config(config,options):
    # All nonoptional channels retain their current required/validation policy.
    topics=[t for t in config.topics if '/depth/' not in t.topic and '/tactile/' not in t.topic]
    static=[t for t in config.static_topics if '/depth/' not in t.topic]
    original={t.topic:t for t in (*config.topics,*config.static_topics)}
    for ch in optional_channels(options):
        kind=ch.get('source_type',ch['type']);snapshot=kind=='sensor_msgs/msg/CameraInfo'
        rate=18. if ch['source_topic'].startswith('/cam0/') else 25. if ch['source_topic'].startswith('/cam') else 18.
        old=original.get(ch['source_topic'])
        item=TopicConfig(old.name if old else ch['source_topic'].strip('/').replace('/','_'),ch['source_topic'],kind,
            required=ch.get('presence_policy')!='conditional',min_frequency_hz=rate,max_gap_ms=500. if '/tactile/' in ch['source_topic'] else 150.)
        (static if snapshot or kind=='std_msgs/msg/UInt64MultiArray' or ch.get('presence_policy')=='conditional' else topics).append(item)
    provenance=dict(config.provenance,recording_options=options,topics={t.name:dict(topic=t.topic,type=t.type_name,required=t.required,min_frequency_hz=t.min_frequency_hz,max_gap_ms=t.max_gap_ms) for t in topics})
    return replace(config,topics=tuple(topics),static_topics=tuple(static),provenance=provenance)


class RecordingBackend:
    def __init__(self,node):
        self.node=node;self._output_dir=node.output_dir;self.base_config=node.config
        self.presence=None;self.last_backend=None;self.backend=None;self.callback=lambda:None;self.options=None;self.task_metadata={}
        from teleop_runtime.recording_settings import read_settings
        self.configure(read_settings())

    def __getattr__(self,name):return getattr(self.backend,name)

    @property
    def on_recording_stopped(self):return self.callback

    @on_recording_stopped.setter
    def on_recording_stopped(self,value):
        self.callback=value
        if self.backend:self.backend.on_recording_stopped=value

    def configure(self,options):
        from teleop_runtime.recording_settings import validate_options,profile_path
        options=validate_options(options)
        if self.backend and self.backend.current_bag_dir is not None:raise RuntimeError('请先完成本段录制')
        if self.options==options:return
        config=select_config(self.base_config,options)
        node=self.node
        kwargs=dict(node=node,output_dir=node.output_dir,bag_prefix=node.bag_prefix,
            topics=tuple(dict.fromkeys(t.topic for t in (*config.topics,*config.static_topics))),stop_timeout_sec=node.stop_timeout_sec,
            rosbag_record_args=tuple(node.rosbag_record_args),default_qos=node.rosbag_record_default_qos,
            provenance=config.provenance,preflight=node.preflight_failures,postflight=node.postflight)
        error=None
        if options['format']=='mcap':
            try:
                from .mcap_recorder import McapEpisodeRecorder
                backend=McapEpisodeRecorder(**kwargs,profile_path=profile_path(),options=options)
            except Exception as exc:
                # Keep the service and GUI available to report missing assets/deps,
                # and to allow selecting rosbag. Never start a fallback recorder.
                error=str(exc);backend=RosbagEpisodeRecorder(**kwargs)
        else:backend=RosbagEpisodeRecorder(**kwargs)
        if self.presence:self.presence.close();self.presence=None
        if options['format']=='rosbag':
            from .sensor_readiness import SensorReadiness
            self.presence=SensorReadiness(node,options)
        if self.backend:
            if self.last_backend is None and self.backend.last_bag_dir is not None:self.last_backend=self.backend
            close=getattr(self.backend,'close',None)
            if close:close()
        self.backend=backend;self.options=options;self.error=error
        backend.on_recording_stopped=self.callback
        node.config=config;node.topic_names=kwargs['topics']

    def readiness(self):
        if self.error:return dict(ready=False,message=self.error)
        if self.presence:return self.presence.snapshot()
        check=getattr(self.backend,'readiness',None)
        return check() if check else dict(ready=True,message='')

    def start(self,**kwargs):
        readiness=self.readiness()
        if not readiness['ready']:raise RuntimeError(readiness.get('message','传感器未就绪'))
        self.backend.task_metadata=dict(self.task_metadata)
        result=self.backend.start(**kwargs)
        self.last_backend=self.backend
        return result

    @property
    def last_bag_dir(self):
        return self.last_backend.last_bag_dir if self.last_backend else None

    def archive_quality(self,*args,**kwargs):return self.last_backend.archive_quality(*args,**kwargs)

    def mark_discarded(self):return self.last_backend.mark_discarded()

    def watchdog(self):
        if self.options['format']=='mcap' and self.active:
            health=self.backend.capture.health()
            if not health['ready']:self.backend.capture.fail('MCAP selected sensor disconnected: '+str(health))

    def close(self):
        if self.presence:self.presence.close()
        close=getattr(self.backend,'close',None)
        if close:close()
