"""Direct MCAP capture. Profile is explicit so missing calibration never becomes invented data."""
import json
import os
from pathlib import Path
import queue
import signal
import socketserver
import threading
from concurrent.futures import ThreadPoolExecutor
import time
import uuid
import yaml

from .mcap_util import write_json
from .mcap_storage import McapEpisode, VIDEO_TYPE
from .mcap_video import VideoEncoder


def channel_role(channel):
    """Presence requirements are independent of storage or message frequency."""
    source = channel['source_topic']
    if channel.get('adapter')=='dagger_context' or channel.get('presence_policy')=='conditional':return 'event'
    if channel.get('snapshot') or source == '/tf_static' or channel['type'] == 'sensor_msgs/msg/CameraInfo':
        return 'snapshot'
    if source in ('/sharpa/left/command','/sharpa/right/command','/dagger/frame', '/openpi/executed_joint_targets',
                  '/teleop/arm_commands', '/teleop/validated_arm_commands'):
        return 'action'
    if source == '/tf' or channel['type'] == 'std_msgs/msg/String' or source == '/teleop/arm_command_status':
        return 'event'
    return 'sensor'


def native(message):
    if hasattr(message,'get_fields_and_field_types'):
        return {k:native(getattr(message,k)) for k in message.get_fields_and_field_types()}
    if isinstance(message,(str,int,float,bool,bytes)): return message
    return [native(v) for v in message]


def stamp_ns(message):
    stamp=message.header.stamp
    return stamp.sec*1_000_000_000+stamp.nanosec


def recording_event(data, receipt_ns):
    """Adapt legacy DAgger events without confusing control and event phases."""
    event=json.loads(data)
    if not isinstance(event,dict):
        raise ValueError('MCAP recording event must be a JSON object')
    legacy=not event.get('event_type')
    if legacy:
        if not isinstance(event.get('event'),str) or not event['event'].strip():
            raise ValueError('MCAP recording event missing event_type/event')
        event['event_type']=event['event']
        # Legacy phase describes the controller, not an event lifecycle.
        if 'phase' in event:
            context=dict(event.get('context') or {})
            context['phase']=event['phase']
            event['context']=context
        event['phase']='instant'
    event.setdefault('phase','instant')
    event.setdefault('severity','info')
    event.setdefault('event_id',uuid.uuid4().hex)
    event.setdefault('stamp_ns',event.get('source_timestamp_ns',receipt_ns))
    if not isinstance(event['event_type'],str) or not event['event_type'].strip():
        raise ValueError('MCAP recording event_type must be a nonempty string')
    if event['phase'] not in ('start','end','instant'):
        raise ValueError('MCAP recording event has invalid lifecycle phase')
    if event['severity'] not in ('info','warning','error','fatal'):
        raise ValueError('MCAP recording event has invalid severity')
    return dict(data=json.dumps(event,ensure_ascii=False))


class Capture:
    def __init__(self, directory, profile, channels, *, serialize=None):
        from copy import deepcopy
        self.directory=Path(directory);self.profile=deepcopy(profile);self.channels=channels;self.serialize=serialize
        # Archive sparse channels as conditional, so zero events are not declared
        # to be missing enabled sensors by downstream conformance checks either.
        for device in self.profile['devices']:
            for channel in device['channels']:
                if channel.get('recording') in ('enabled','event_driven') and channel_role(channel) == 'event':
                    channel.update(recording='event_driven', presence_policy='conditional')
        self.finished=threading.Event();self.lock=threading.RLock();self.active=False
        self.error=None;self.writer=None;self.receipts={};self.static={};self.encoders={};self.static_tf_edges={};self.video_workers={}
        self.queue=queue.Queue(maxsize=256);self.worker=None;self.started=None;self.close_lock=threading.Lock();self.last_context=None;self.last_enqueued_context=None;self.compact=os.environ.get('SHARPA_COMPACT_RECORDS')=='1'
        self.camera_info_keys={};self.camera_info_repeats_omitted={}
        self.accepted_counts={ch['topic']:0 for ch in channels};self.queue_high_watermark=0
        self.report={'status':'PREPARED','recording_format':'mcap','started_unix_ns':None}
        self.persist()

    def persist(self):
        if self.writer is not None:
            self.report['payload_directory']=str(self.writer.target if self.writer.target.exists() else self.writer.directory)
        write_json(self.directory/'recording.json',dict(self.report,error=self.error,
            camera_info_repeats_omitted=dict(self.camera_info_repeats_omitted),
            accepted_counts=dict(self.accepted_counts),queue_high_watermark=self.queue_high_watermark,
            counts={} if self.writer is None else {k:v['message_count'] for k,v in self.writer.stats.items()}))

    def fail(self,error):
        self.error=str(error);self.active=False;self.finished.set();self.persist()

    def accept(self,ch,message,rx):
        with self.lock:
            if ch['type']==VIDEO_TYPE:
                if (message.width,message.height)!=(ch['width'],ch['height']) or message.encoding not in ('rgb8','bgr8'):
                    self.fail('MCAP camera native dimensions/encoding differ from verified profile: '+ch['source_topic']);return
            previous=self.receipts.get(ch['source_topic'])
            self.receipts[ch['source_topic']]=(time.monotonic(),1+(previous[1] if previous else 0))
            if channel_role(ch) == 'snapshot':
                if ch['type']=='tf2_msgs/msg/TFMessage':
                    for transform in message.transforms:
                        self.static_tf_edges[transform.child_frame_id]=transform
                    combined=type(message)();combined.transforms=list(self.static_tf_edges.values())
                    message=combined
                self.static[ch['source_topic']]=(message,rx)
            if not self.active:return
            if ch.get('adapter')=='split_gateway_command':
                from .mcap_actions import split_gateway_message
                try:
                    if ch['command_group'] not in split_gateway_message(message,native(message.header)):return
                except Exception as error:
                    self.fail(error);return
            if ch['type']=='sensor_msgs/msg/CameraInfo':
                from .recording_slim import camera_info_signature
                key=camera_info_signature(message)
                if self.camera_info_keys.get(ch['topic'])==key:
                    self.camera_info_repeats_omitted[ch['topic']]=self.camera_info_repeats_omitted.get(ch['topic'],0)+1
                    return
                self.camera_info_keys[ch['topic']]=key
            if ch.get('adapter')=='dagger_context':
                frame=json.loads(message.data)
                context={key:frame.get(key) for key in ('mode','phase','generation','intervention_id','stage','prompt','model_version')}
                if context==self.last_enqueued_context:return
                self.last_enqueued_context=context
            try:
                prepared = (self.video_workers[ch['topic']].submit(self.encode_video,ch,message)
                            if ch['type']==VIDEO_TYPE else None)
                self.queue.put_nowait((ch,message,rx,prepared))
                self.accepted_counts[ch['topic']]+=1
                self.queue_high_watermark=max(self.queue_high_watermark,self.queue.qsize())
            except queue.Full:self.fail('MCAP encoding queue overflow')

    def health(self):
        now=time.monotonic()
        missing, stale, insufficient = [], [], []
        with self.lock:
            for ch in self.channels:
                source = ch['source_topic']; role = channel_role(ch)
                if role == 'snapshot':
                    if source not in self.static: missing.append(source)
                elif role == 'sensor':
                    receipt = self.receipts.get(source)
                    if receipt is None: missing.append(source)
                    else:
                        if not 0 <= now-receipt[0] < .5: stale.append(source)
                        if receipt[1] < 2: insufficient.append(source)
        return dict(ready=not (missing or stale or insufficient), missing=missing,
                    stale=stale, insufficient_progress=insufficient)

    def readiness(self):
        return self.health()['ready']

    def prepare_resources(self):
        """Warm schemas, assets and encoders only; accept no data before the start gate."""
        if self.writer is not None:
            return
        target=self.target
        root=self.root
        identity=dict(self.identity)
        self.writer=McapEpisode(target,root=root,profile=self.profile,identity=identity,start_time_ns=0)
        for ch in self.channels:
            self.writer.register(ch['topic'],ch['type'])
            if ch['type']==VIDEO_TYPE:
                self.encoders[ch['topic']]=VideoEncoder(ch)
                self.video_workers[ch['topic']]=ThreadPoolExecutor(max_workers=1,thread_name_prefix='mcap-'+str(len(self.video_workers)))

    def request(self,operation):
        if self.error:raise RuntimeError(self.error)
        if operation=='status': return dict(status='PASS',active=self.active,sensor_readiness=self.health())
        if operation=='start':
            if self.writer is None:self.prepare_resources()
            with self.lock:
                if self.finished.is_set():raise RuntimeError('Cannot reuse stopped MCAP recorder')
                if self.active:return dict(status='PASS',active=True)
                health = self.health()
                if not health['ready']:raise RuntimeError('MCAP channels/CameraInfo/TF snapshot not ready: '+json.dumps(health))
                if self.writer is None:self.prepare_resources()
                # Recheck after preparation, as rosbag does before activation.
                if not self.readiness():raise RuntimeError('MCAP sensors changed during preparation')
                self.started=time.time_ns();self.writer.start=self.started
                for ch in self.channels:
                    if channel_role(ch) == 'snapshot':
                        message,rx=self.static[ch['source_topic']]
                        self.queue.put_nowait((ch,message,self.started,None))
                        self.accepted_counts[ch['topic']]+=1
                        if ch['type']=='sensor_msgs/msg/CameraInfo':
                            from .recording_slim import camera_info_signature
                            self.camera_info_keys[ch['topic']]=camera_info_signature(message)
                self.active=True
                self.report.update(status='RECORDING',started_unix_ns=self.started)
                self.worker=threading.Thread(target=self.work,daemon=True);self.worker.start();self.persist()
                return dict(status='PASS',active=True)
        if operation=='stop':self.close();return dict(status='PASS',active=False)
        raise ValueError(operation)

    def encode_video(self, channel, message):
        """One FIFO encoder worker per camera; MCAP writes still follow receipt order."""
        stamp=stamp_ns(message)
        return dict(timestamp=dict(sec=stamp//10**9,nanosec=stamp%10**9),
                    frame_id=message.header.frame_id,format='h264',
                    data=self.encoders[channel['topic']].encode(message,stamp))

    def work(self):
        try:
            while True:
                item=self.queue.get()
                if item is None:break
                ch,msg,rx,prepared=item;topic=ch['topic'];kind=ch['type']
                if ch.get('adapter')=='split_gateway_command':
                    from .mcap_actions import split_gateway_message
                    parts=split_gateway_message(msg,native(msg.header))
                    if ch['command_group'] not in parts:continue
                    stamp=stamp_ns(msg)
                    value=parts[ch['command_group']]
                elif ch.get('adapter')=='hand_command':
                    from .mcap_actions import joint_message
                    stamp=stamp_ns(msg)
                    value=joint_message(msg,native(msg.header),ch['command_group'])
                elif ch.get('adapter')=='depth_rvl':
                    from .recording_options import depth_payload
                    stamp=stamp_ns(msg)
                    value=dict(header=native(msg.header),format='16UC1; compressedDepth rvl',data=depth_payload(msg))
                elif kind==VIDEO_TYPE:
                    stamp=stamp_ns(msg)
                    value=prepared.result()
                elif ch.get('adapter')=='tactile_sample_info':
                    if len(msg.data)!=3:raise ValueError('Invalid tactile sample metadata')
                    stamp=int(msg.data[1]);value=native(msg)
                elif ch.get('adapter')=='arm_command':
                    value=dict(header=native(msg.header),name=list(msg.joint_names),position=list(msg.positions),velocity=[],effort=[])
                    stamp=stamp_ns(msg)
                elif kind=='std_msgs/msg/String':
                    value=(recording_event(msg.data,rx) if topic=='/recording/events' else
                           {'data':json.dumps(native(msg))} if ch.get('adapter')=='diagnostic_event' else
                           None if self.serialize is not None else native(msg));stamp=rx
                elif kind=='tf2_msgs/msg/TFMessage':
                    stamp=rx
                    if topic=='/tf_static' and self.profile.get('static_extrinsics'):
                        from .calibrated_tf import merge_static
                        value=merge_static(native(msg),self.profile['static_extrinsics'],stamp)
                    else:value=None if self.serialize is not None else native(msg)
                else:value=None;stamp=stamp_ns(msg)
                if self.serialize is not None and (not ch.get('adapter') or ch.get('adapter')=='tactile_sample_info') and kind!=VIDEO_TYPE and topic!='/recording/events' and not (topic=='/tf_static' and self.profile.get('static_extrinsics')):
                    self.writer.write_serialized(topic,self.serialize(msg),log_time=rx,publish_time=stamp)
                else:
                    if value is None:value=native(msg)
                    self.writer.write(topic,value,log_time=rx,publish_time=stamp)
        except Exception as error:self.fail(error)

    def close(self):
        with self.close_lock:
            return self._close()

    def _close(self):
        with self.lock:self.active=False;self.finished.set()
        if self.worker and self.worker.is_alive():
            self.queue.put(None,timeout=5);self.worker.join(timeout=30)
            if self.worker.is_alive():raise RuntimeError('MCAP writer failed to close')
        for worker in self.video_workers.values():worker.shutdown(wait=True)
        if self.writer and not self.writer.closed and self.started is None:
            for encoder in self.encoders.values():encoder.close()
            self.writer.cancel_prepared()
        if self.writer and not self.writer.closed:
            try:
                for encoder in self.encoders.values():encoder.close()
                missing = [ch['source_topic'] for ch in self.channels if channel_role(ch) != 'event'
                           and not self.writer.stats[ch['topic']]['message_count']]
                if not self.error and missing:
                    self.error='MCAP required channel contains no samples: '+', '.join(missing)
                mismatch={topic:(count,self.writer.stats[topic]['message_count'])
                          for topic,count in self.accepted_counts.items() if count!=self.writer.stats[topic]['message_count']}
                if not self.error and mismatch:self.error='MCAP accepted/written message count mismatch: '+str(mismatch)
                self.writer.close(self.error)
                self.report['status']='FAILED' if self.error else 'COMPLETE'
            except Exception as error:
                self.error=str(error);self.report['status']='FAILED'
        elif not self.started:self.report['status']='FAILED' if self.error else 'NOT_STARTED'
        self.report['ended_unix_ns']=time.time_ns();self.persist()

