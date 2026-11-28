"""Software-only native ROS messages through the actual encoder/writer pipeline."""
import json
import time
from pathlib import Path
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('av')
pytest.importorskip('mcap')
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, JointState, CameraInfo
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from teleop_data_collector.mcap_capture import Capture
from teleop_data_collector.mcap_storage import digest
from teleop_core.contract import LEFT_COMMAND_JOINT_NAMES,RIGHT_COMMAND_JOINT_NAMES,SHARPA_JOINT_NAMES


def fixture(tmp_path):
    model=tmp_path/'fixture-model';model.mkdir();(model/'robot.urdf').write_text('<robot name="software_fixture"><link name="base"/></robot>')
    calib=tmp_path/'fixture-calibration';calib.mkdir();(calib/'fixture.txt').write_text('SOFTWARE TEST ONLY - NOT A REAL CALIBRATION')
    channels=[]
    for i,role in enumerate(('head','wrist_right','wrist_left')):
        channels.append(dict(source_topic=f'/cam{i}/color/image_raw',topic=f'/devices/cam{i}/cameras/color/color/image_h264',type='foxglove_msgs/msg/CompressedVideo',recording='enabled',width=64,height=48,fps=30,role=role,crf=18 if i==0 else 16))
        channels.append(dict(source_topic=f'/cam{i}/color/camera_info',topic=f'/devices/cam{i}/cameras/color/color/camera_info',type='sensor_msgs/msg/CameraInfo',recording='enabled'))
    for group in ('left_arm','right_arm','left_hand','right_hand'):
        side=group.split('_')[0];device=('fr3_' if group.endswith('arm') else 'sharpa_')+side
        channels.append(dict(source_topic='/teleop/validated_arm_commands' if group.endswith('arm') else f'/sharpa/{side}/command',
            topic=f'/devices/{device}/action/command',type='sensor_msgs/msg/JointState',recording='enabled',
            adapter='split_gateway_command' if group.endswith('arm') else 'hand_command',command_group=group))
        channels.append(dict(source_topic=f'/{side}/franka/joint_states' if group.endswith('arm') else f'/sharpa/{side}/joint_states',topic=f'/devices/{device}/state/joints',type='sensor_msgs/msg/JointState',recording='enabled'))
    channels.append(dict(source_topic='/tf_static',topic='/tf_static',type='tf2_msgs/msg/TFMessage',recording='enabled',snapshot=True))
    channels.extend([
        dict(source_topic='/tf',topic='/tf',type='tf2_msgs/msg/TFMessage',recording='event_driven'),
        dict(source_topic='/teleop/arm_commands',topic='/devices/rig/action/source',type='sensor_msgs/msg/JointState',source_type='teleop_interfaces/msg/ArmCommand',adapter='arm_command',recording='enabled'),
        dict(source_topic='/teleop/arm_command_status',topic='/recording/gateway_feedback',type='std_msgs/msg/String',source_type='teleop_interfaces/msg/ArmCommandStatus',adapter='diagnostic_event',recording='event_driven')])
    profile=dict(model_root=str(model),calibration_root=str(calib),session_id='software-fixture',clock_mapping={'software_fixture':True},devices=[dict(device_id='rig',channels=channels)])
    runtime=tmp_path/'runtime';runtime.mkdir()
    capture=Capture(runtime,profile,channels,serialize=serialize_message)
    root=tmp_path/'archive';(root/'pending').mkdir(parents=True)
    capture.target=root/'pending/episode0';capture.root=root
    capture.identity=dict(episode_id='episode0',episode_uid='software-fixture',session_id='software-fixture',recording_id='software-fixture',task_id='test',task_description='Software-only integration fixture.',source_type='teleoperation',source_id='gello',end_reason='software test',recorder_commit='0'*40)
    return capture,channels


def message(ch,stamp,index):
    kind=ch.get('source_type',ch['type']);source=ch['source_topic']
    if ch['type']=='foxglove_msgs/msg/CompressedVideo':
        msg=Image();msg.width=64;msg.height=48;msg.step=64*3;msg.encoding='rgb8';msg.data=bytes([index%256,70,160])*(64*48)
    elif kind=='teleop_interfaces/msg/ArmCommand':
        from teleop_interfaces.msg import ArmCommand
        msg=ArmCommand();msg.joint_names=list(LEFT_COMMAND_JOINT_NAMES+RIGHT_COMMAND_JOINT_NAMES);msg.positions=[.01*index]*14
    elif kind=='teleop_interfaces/msg/ArmCommandStatus':
        from teleop_interfaces.msg import ArmCommandStatus
        msg=ArmCommandStatus()
    elif kind=='sensor_msgs/msg/CameraInfo':
        msg=CameraInfo();msg.width=64;msg.height=48;msg.k=[50.,0.,32.,0.,50.,24.,0.,0.,1.]
    elif kind=='tf2_msgs/msg/TFMessage':
        msg=TFMessage();edge=TransformStamped();edge.header.frame_id='base';edge.child_frame_id='camera';edge.transform.rotation.w=1.;msg.transforms=[edge];return msg
    else:
        msg=JointState()
        msg.name=list(LEFT_COMMAND_JOINT_NAMES+RIGHT_COMMAND_JOINT_NAMES) if source=='/teleop/validated_arm_commands' else list(SHARPA_JOINT_NAMES) if '/sharpa/' in source else list(LEFT_COMMAND_JOINT_NAMES if '/left/' in source else RIGHT_COMMAND_JOINT_NAMES)
        msg.position=[.01*index]*len(msg.name)
    msg.header.stamp.sec=stamp//10**9;msg.header.stamp.nanosec=stamp%10**9;msg.header.frame_id='camera'
    return msg


def test_gate_encoder_counts_crc_and_relocatable_assets(tmp_path):
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    capture,channels=fixture(tmp_path);base=time.time_ns()
    # Idle samples provide readiness/snapshots, but never enter the recording.
    for index in (0,1):
        for ch in channels:capture.accept(ch,message(ch,base+index*33_333_333,index),time.time_ns())
    assert not any(capture.accepted_counts.values())
    capture.request('start')
    for index in range(2,37):
        for ch in channels:
            if ch['type'] in ('sensor_msgs/msg/CameraInfo','tf2_msgs/msg/TFMessage'):continue
            capture.accept(ch,message(ch,base+index*33_333_333,index),time.time_ns())
        time.sleep(.004)
    capture.close()
    assert capture.error is None,capture.error
    episode=capture.target
    with (episode/'data.mcap').open('rb') as stream:
        reader=make_reader(stream,validate_crcs=True,decoder_factories=[DecoderFactory()])
        rows=list(reader.iter_decoded_messages(log_time_order=False))
        assert reader.get_header().profile=='ros2'
        assert reader.get_summary().statistics.message_count==len(rows)
    for ch in channels:
        selected=[r for r in rows if r.channel.topic==ch['topic']]
        assert len(selected)==(0 if ch['source_topic']=='/tf' else 1 if ch['type'] in ('sensor_msgs/msg/CameraInfo','tf2_msgs/msg/TFMessage') else 35)
        if ch['type']=='foxglove_msgs/msg/CompressedVideo':
            import av
            codec=av.CodecContext.create('h264','r');frames=[]
            for row in selected:frames.extend(codec.decode(av.Packet(bytes(row.decoded_message.data))))
            frames.extend(codec.decode(None))
            assert len(frames)==35 and all((f.width,f.height)==(64,48) for f in frames)
            assert selected[0].message.publish_time==base+2*33_333_333
    info=json.loads((episode/'episode.json').read_text())
    for ref in info['assets'].values():assert digest(episode/ref['path'])==ref['sha256']
    manifest=json.loads((episode/'manifest.json').read_text());assert manifest['status']=='complete'
    for item in manifest['files']:assert digest(episode/item['path'])==item['sha256']
    # Closed gate excludes subsequent Home/idle samples.
    counts=dict(capture.accepted_counts)
    for ch in channels:capture.accept(ch,message(ch,base+40*33_333_333,40),time.time_ns())
    assert capture.accepted_counts==counts


def test_backward_source_time_fails_without_complete(tmp_path):
    capture,channels=fixture(tmp_path);base=time.time_ns()
    for index in (0,1):
        for ch in channels:capture.accept(ch,message(ch,base+index*1000,index),time.time_ns())
    capture.request('start')
    ch=channels[0]
    capture.accept(ch,message(ch,base+10_000,3),time.time_ns())
    capture.accept(ch,message(ch,base+9_000,4),time.time_ns())
    capture.close()
    assert capture.error
    assert json.loads((capture.target/'manifest.json').read_text())['status']=='failed'


def test_controller_archive_regrade_and_format_switch(tmp_path,monkeypatch):
    import yaml
    from types import SimpleNamespace as NS
    from teleop_data_collector.recording_backend import RecordingBackend
    from teleop_data_collector.config import CollectorConfig,TopicConfig
    from teleop_data_collector.collector_control import CollectorController
    capture,channels=fixture(tmp_path)
    profile=capture.profile
    profile['clock_mapping']=dict(valid_from_ns=1,sync_state='sdk_mapped')
    for ch in profile['devices'][0]['channels']:
        if ch['type']=='foxglove_msgs/msg/CompressedVideo':
            ch.update(source_type='sensor_msgs/msg/Image',color=dict(range='limited',matrix='bt709',primaries='bt709',transfer='iec61966-2-1'),color_source='SOFTWARE FIXTURE ONLY')
    config=tmp_path/'profile.yaml';config.write_text(yaml.safe_dump(profile))
    prefs=tmp_path/'recording.json';prefs.write_text(json.dumps(dict(format='mcap',head_depth=False,wrist_depth=False,tactile=False)))
    monkeypatch.setenv('COLLECTION_RECORDING_CONFIG',str(prefs));monkeypatch.setenv('COLLECTION_MCAP_PROFILE',str(config))
    logger=NS(info=lambda _:None,warn=lambda _:None,error=lambda _:None)
    callbacks=[]
    node=NS(config=CollectorConfig(str(tmp_path/'output'),(TopicConfig('state','/left/franka/joint_states','sensor_msgs/msg/JointState'),),(),0.,0.,{}),
        output_dir=tmp_path/'output',bag_prefix='episode',stop_timeout_sec=30,rosbag_record_args=[],rosbag_record_default_qos={},
        preflight_failures=lambda:(),postflight=lambda path:((),{}),get_logger=lambda:logger,
        create_subscription=lambda *args,**kwargs:callbacks.append(args) or args,destroy_subscription=lambda sub:None)
    recorder=RecordingBackend(node)
    assert recorder.error is None,recorder.error
    node.configure_recording=recorder.configure
    controller=CollectorController(node,recorder,quality_dir=tmp_path/'quality');controller.set_ready()
    base=time.time_ns()
    for index in (0,1):
        for ch in recorder.backend.channels:recorder.backend.capture.accept(ch,message(ch,base+index*1000,index),time.time_ns())
    parent=tmp_path/'saved'
    controller.start(storage_parent=str(parent),task_metadata=dict(task_id='assembly',task_description='Assemble the parts.'))
    for index in range(2,8):
        for ch in recorder.backend.channels:
            if ch['type'] in ('sensor_msgs/msg/CameraInfo','tf2_msgs/msg/TFMessage'):continue
            recorder.backend.capture.accept(ch,message(ch,base+index*1_000_000,index),time.time_ns())
    stopped=controller.stop()
    assert stopped['finalized'] and stopped['quality_pending'],stopped
    controller.rate_quality(recorder.last_bag_dir.name,'优等')
    bag=recorder.last_bag_dir
    assert bag.parent.name=='A' and not (bag/'collection_state.json').exists()
    document=json.loads((bag/'episode.json').read_text())
    assert document['task_description']=='Assemble the parts.'
    assert document['quality']['grade']=='A' and document['collection']['transfer_ready']
    controller.dispatch('configure_recording',dict(format='rosbag',head_depth=False,wrist_depth=False,tactile=False))
    # Regrading the previous episode uses its original backend, even after selection changes.
    controller.rate_quality(bag.name,'一般')
    bag=recorder.last_bag_dir
    assert bag.parent.name=='B'
    document=json.loads((bag/'episode.json').read_text())
    assert document['quality']['grade']=='B' and document['quality']['revision']==2
    manifest=json.loads((bag/'manifest.json').read_text())
    for item in manifest['files']:assert digest(bag/item['path'])==item['sha256']
    controller.rate_quality(bag.name,'放弃')
    bag=recorder.last_bag_dir
    assert bag.parent.name=='D'
    assert json.loads((bag/'manifest.json').read_text())['status']=='complete'
    assert json.loads((bag/'episode.json').read_text())['quality']['grade']=='D'
    recorder.close()
