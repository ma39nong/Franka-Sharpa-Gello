"""Presence/progress gate for selected optional rosbag sensors; no device ownership."""
import threading
import time
from .recording_options import optional_channels


class SensorReadiness:
    def __init__(self,node,options):
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        from .image_metadata import read_image_metadata
        self.node=node;self.subscriptions=[];self.samples={};self.snapshots=set();self.lock=threading.Lock()
        for ch in optional_channels(options):
            if ch.get('presence_policy')=='conditional':continue
            source=ch['source_topic'];kind=ch.get('source_type',ch['type']);cls=get_message(kind)
            self.samples[source]=(0.,0,0)
            if kind=='sensor_msgs/msg/CameraInfo':self.snapshots.add(source)
            def receive(data,source=source,kind=kind,cls=cls):
                try:
                    if kind=='sensor_msgs/msg/Image':stamp=read_image_metadata(data).header_time_ns
                    else:
                        message=deserialize_message(data,cls)
                        if kind=='std_msgs/msg/UInt64MultiArray':
                            if len(message.data)!=3:raise ValueError('Invalid SDK sample info')
                            stamp=int(message.data[1])
                        else:stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
                    with self.lock:
                        _,previous,count=self.samples[source]
                        if stamp>0 and stamp>previous:self.samples[source]=(time.monotonic(),stamp,count+1)
                except (ValueError,AttributeError,IndexError):pass
            self.subscriptions.append(node.create_subscription(cls,source,receive,QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE),raw=True))

    def snapshot(self):
        now=time.monotonic()
        with self.lock:
            waiting=[topic for topic,(last,stamp,count) in self.samples.items() if not count or (topic not in self.snapshots and (count<2 or now-last>.5))]
        return dict(ready=not waiting,message='等待所选传感器；未启动时需下次正常启动后生效：'+', '.join(waiting) if waiting else '')

    def close(self):
        for sub in self.subscriptions:self.node.destroy_subscription(sub)
        self.subscriptions=[]
