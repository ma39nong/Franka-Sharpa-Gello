"""Replay local recorded camera content at native rates; no ROS node/device opened.

Requires the existing ROS Python environment for native CDR serialization. Reports
received/written counts without dropping, decimation or a larger queue.
"""
import argparse
from array import array
import heapq
import json
from pathlib import Path
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',required=True,type=Path)
    parser.add_argument('--out',required=True,type=Path)
    parser.add_argument('--seconds',type=int,default=20)
    parser.add_argument('--tactile',action='store_true')
    parser.add_argument('--profile',action='store_true')
    parser.add_argument('--depth',action='store_true')
    args=parser.parse_args()
    if args.seconds<1:parser.error('--seconds must be positive')
    import av
    import numpy as np
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image,JointState
    from std_msgs.msg import String,UInt64MultiArray
    from geometry_msgs.msg import WrenchStamped
    from .mcap_capture import Capture
    from .mcap_storage import VIDEO_TYPE
    out=args.out.resolve();out.mkdir(parents=True,exist_ok=False)
    model=out/'fixture-model';model.mkdir();(model/'robot.xml').write_text('<robot name="software_fixture"/>')
    cal=out/'fixture-calibration';cal.mkdir();(cal/'info.yaml').write_text('synthetic: true')
    directory=out/'run';directory.mkdir();(directory/'run.json').write_text('{}')
    (out/'episodes/pending').mkdir(parents=True)
    videos={};decoders={}
    with args.source.open('rb') as stream:
        for schema,channel,message,value in make_reader(stream,decoder_factories=[DecoderFactory()]).iter_decoded_messages():
            if schema.name!=VIDEO_TYPE:continue
            decoder=decoders.setdefault(channel.topic,av.CodecContext.create('h264','r'))
            for frame in decoder.decode(av.Packet(bytes(value.data))):
                if len(videos.get(channel.topic,[]))<30:videos.setdefault(channel.topic,[]).append((frame.width,frame.height,frame.to_ndarray(format='rgb24').tobytes()))
            if len(videos)==3 and all(len(v)>=30 for v in videos.values()):break
    if len(videos)!=3:raise ValueError('Replay fixture must contain three real camera streams')
    channels=[];factories=[];rates=[]
    def add(channel,factory,hz):
        channels.append(channel);factories.append(factory);rates.append(hz)
    def stamp(message,ns):
        message.header.stamp.sec=ns//10**9;message.header.stamp.nanosec=ns%10**9
        message.header.frame_id='fixture_frame';return message
    for i,(topic,frames) in enumerate(sorted(videos.items())):
        role='head' if 'cam0' in topic else 'wrist_right' if 'cam1' in topic else 'wrist_left'
        hz=20 if role=='head' else 30;w,h,_=frames[0]
        channel=dict(source_topic=f'/fixture/cam{i}',topic=topic,type=VIDEO_TYPE,recording='enabled',role=role,crf=18 if role=='head' else 16,
            width=w,height=h,fps=hz,color=dict(range='limited',matrix='bt709',primaries='bt709',transfer='iec61966-2-1'))
        def image(index,ns,frames=frames):
            w,h,data=frames[index%len(frames)];msg=Image();msg.width=w;msg.height=h;msg.encoding='rgb8';msg.step=w*3;msg.data=array('B',data)
            return stamp(msg,ns)
        add(channel,image,hz)
    for side in ('left','right'):
        channel=dict(source_topic=f'/fixture/{side}/joints',topic=f'/devices/{side}/state/joints',type='sensor_msgs/msg/JointState',recording='enabled')
        def joint(index,ns):
            msg=JointState();msg.name=[f'joint{i}' for i in range(7)];msg.position=[index*.00001]*7;msg.velocity=[0.]*7;msg.effort=[0.]*7
            return stamp(msg,ns)
        add(channel,joint,201)
    for group,width in [('left_hand',22),('right_hand',22),('left_arm_command',7),('right_arm_command',7),('left_hand_command',22),('right_hand_command',22)]:
        channel=dict(source_topic=f'/fixture/{group}',topic=f'/devices/{group}/state/joints',type='sensor_msgs/msg/JointState',recording='enabled')
        def joint(index,ns,width=width):
            msg=JointState();msg.name=[f'joint{i}' for i in range(width)];msg.position=[index*.00001]*width
            return stamp(msg,ns)
        add(channel,joint,201 if width==7 else 30)
    event=dict(source_topic='/fixture/event',topic='/recording/events',type='std_msgs/msg/String',recording='event_driven')
    def status(index,ns):msg=String();msg.data=json.dumps(dict(event_id=str(index),event_type='fixture',phase='instant',severity='info',stamp_ns=ns,description='fixture'));return msg
    add(event,status,120)
    if args.tactile:
        for side in ('left','right'):
            for finger in ('thumb','index','middle','ring','pinky'):
                for signal,width in (('raw',320),('deformation',240)):
                    # Fixed native-sized textures, not empty arrays or discarded frames.
                    pixels=np.random.default_rng(len(channels)).integers(0,256,(240,width),dtype=np.uint8).tobytes()
                    channel=dict(source_topic=f'/fixture/{side}/{finger}/{signal}',topic=f'/devices/{side}/tactile/{finger}/image/{signal}',type='sensor_msgs/msg/Image',recording='enabled')
                    def tactile(index,ns,pixels=pixels,width=width):
                        msg=Image();msg.width=width;msg.height=240;msg.step=width;msg.encoding='mono8';msg.data=array('B',pixels)
                        return stamp(msg,ns)
                    add(channel,tactile,30)
                channel=dict(source_topic=f'/fixture/{side}/{finger}/wrench',topic=f'/devices/{side}/tactile/{finger}/wrench',type='geometry_msgs/msg/WrenchStamped',recording='enabled')
                def wrench(index,ns):
                    msg=WrenchStamped();msg.wrench.force.x=index*.00001;return stamp(msg,ns)
                add(channel,wrench,30)
    if args.tactile:
        for side in ('left','right'):
            for finger in ('thumb','index','middle','ring','pinky'):
                channel=dict(source_topic=f'/fixture/{side}/{finger}/sample_info',topic=f'/recording/tactile/{side}/{finger}/sample_info',
                    type='std_msgs/msg/UInt64MultiArray',adapter='tactile_sample_info',recording='enabled')
                def sample_info(index,ns):
                    msg=UInt64MultiArray();msg.data=[index,ns,ns];return msg
                add(channel,sample_info,30)
    if args.depth:
        for camera,h,hz in [('cam0',400,20),('cam1',480,30),('cam2',480,30)]:
            pixels=np.random.default_rng(len(channels)).integers(0,5000,(h,640),dtype=np.uint16).tobytes()
            channel=dict(source_topic=f'/fixture/{camera}/depth',topic=f'/devices/{camera}/cameras/depth/depth/image_raw/compressedDepth',type='sensor_msgs/msg/CompressedImage',adapter='depth_rvl',recording='enabled')
            def depth(index,ns,pixels=pixels,h=h):
                msg=Image();msg.width=640;msg.height=h;msg.step=1280;msg.encoding='16UC1';msg.data=array('B',pixels)
                return stamp(msg,ns)
            add(channel,depth,hz)
    # A real ROS subscription loads these C extensions before receiving callbacks.
    for message_type in (Image,JointState,String,WrenchStamped,UInt64MultiArray):serialize_message(message_type())
    profile=dict(model_root=str(model),calibration_root=str(cal),session_id='software-replay',devices=[dict(device_id='fixture',channels=channels)],clock_mapping={})
    if args.profile:
        import cProfile
        original=Capture.work
        def profiled(self):
            stats=cProfile.Profile()
            try:return stats.runcall(original,self)
            finally:stats.dump_stats(str(out/'writer.prof'))
        Capture.work=profiled
    capture=Capture(directory,profile,channels,serialize=serialize_message)
    capture.root=out/'episodes';capture.target=capture.root/'pending/episode0'
    capture.identity=dict(episode_id='episode0',episode_uid='software-benchmark',session_id='software-replay',
        recording_id='software-benchmark',task_id='software-replay',task_description='Software throughput test; not training data.',
        source_type='teleoperation',source_id='gello',end_reason='software benchmark',recorder_commit='0'*40)
    capture.prepare_resources()
    origin=time.time_ns()
    for channel,factory in zip(channels,factories):
        capture.accept(channel,factory(0,origin-2),origin-2);capture.accept(channel,factory(1,origin-1),origin-1)
    capture.request('start')
    origin=time.time_ns();started=time.monotonic();heap=[(0.,i,0) for i in range(len(channels))];heapq.heapify(heap)
    expected={channel['topic']:0 for channel in channels};max_producer_lag_s=0.
    try:
        while heap:
            due,i,index=heapq.heappop(heap)
            if due>=args.seconds:continue
            remaining=started+due-time.monotonic()
            if remaining>0:time.sleep(remaining)
            max_producer_lag_s=max(max_producer_lag_s,time.monotonic()-started-due)
            message=factories[i](index,origin+round(due*1e9))
            capture.accept(channels[i],message,time.time_ns());expected[channels[i]['topic']]+=1
            if capture.error:raise RuntimeError(capture.error)
            heapq.heappush(heap,((index+1)/rates[i],i,index+1))
    finally:
        producer_elapsed=time.monotonic()-started
        capture.close()
    written={topic:row['message_count'] for topic,row in capture.writer.stats.items()}
    assert written==expected,(expected,written)
    assert capture.report['status']=='COMPLETE',capture.report
    summary=dict(seconds=args.seconds,tactile=args.tactile,depth=args.depth,accepted=sum(expected.values()),written=sum(written.values()),
        producer_elapsed_s=producer_elapsed,total_elapsed_s=time.monotonic()-started,max_producer_lag_s=max_producer_lag_s,
        queue_capacity=capture.queue.maxsize,queue_high_watermark=capture.queue_high_watermark,counts_match=True,
        input_rates_hz=rates,source=str(args.source.resolve()),episode=str(capture.writer.target),
        note='Software replay, not a live hardware S1 certification')
    (out/'benchmark.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
