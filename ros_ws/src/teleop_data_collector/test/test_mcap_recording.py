"""Offline contract tests. No devices, robot graph or command publishers."""
import itertools
import json
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
from teleop_data_collector.mcap_actions import joint_message,split_gateway_message,GROUPS
from teleop_core.contract import LEFT_COMMAND_JOINT_NAMES,SHARPA_JOINT_NAMES


def test_split_gateway_partial_side_and_hand_names():
    message=NS(name=list(LEFT_COMMAND_JOINT_NAMES),position=list(range(7)),velocity=[],effort=[])
    result=split_gateway_message(message,{})
    assert set(result)=={'left_arm'}
    assert result['left_arm']['name']==list(GROUPS['left_arm'][1])
    message.name.pop()
    with pytest.raises(ValueError):split_gateway_message(message,{})
    message=NS(name=list(SHARPA_JOINT_NAMES),position=list(range(22)),velocity=[],effort=[])
    assert joint_message(message,{},'right_hand')['name']==list(GROUPS['right_hand'][1])


@pytest.mark.parametrize('head,wrist,tactile',itertools.product((False,True),repeat=3))
def test_selected_source_contract(head,wrist,tactile):
    pytest.importorskip('rclpy')
    from teleop_data_collector.config import CollectorConfig,TopicConfig
    from teleop_data_collector.recording_backend import select_config
    base=CollectorConfig('/tmp/unused',(TopicConfig('rgb','/cam0/color/image_raw','sensor_msgs/msg/Image'),),(),1.,.2,{})
    selected=select_config(base,dict(format='mcap',head_depth=head,wrist_depth=wrist,tactile=tactile))
    paths={t.topic for t in (*selected.topics,*selected.static_topics)}
    assert ('/cam0/depth/image_raw' in paths)==head
    assert ('/cam1/depth/image_raw' in paths)==wrist
    assert ('/cam2/depth/image_raw' in paths)==wrist
    assert len([p for p in paths if '/tactile/' in p])==(50 if tactile else 0)
    assert all(not t.required for t in selected.static_topics if t.topic.endswith('/contact_point'))


def test_depth_rvl_native_roundtrip(monkeypatch):
    np=pytest.importorskip('numpy')
    from teleop_data_collector.recording_options import depth_payload
    tools=Path('/conformance/recorder-conformance/scripts')
    if not tools.is_dir():pytest.skip('Independent conformance decoder not mounted')
    import sys
    sys.path.insert(0,str(tools))
    from conformance.rvl import decode_payload
    rng=np.random.default_rng(7)
    pixels=rng.integers(0,65536,size=(13,17),dtype=np.uint16);pixels[::2,::3]=0
    for endian in ('<','>'):
        padded=np.zeros((13,20),dtype=endian+'u2');padded[:,:17]=pixels
        message=NS(encoding='16UC1',is_bigendian=endian=='>',height=13,width=17,step=40,data=padded.tobytes())
        header,decoded=decode_payload(depth_payload(message))
        assert header['cols']==17 and header['rows']==13
        np.testing.assert_array_equal(decoded.reshape(13,17),pixels)


def test_rosbag_selected_sensor_gate_requires_real_progress():
    pytest.importorskip('rclpy')
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image,CameraInfo
    from teleop_data_collector.sensor_readiness import SensorReadiness
    callbacks={}
    node=NS(create_subscription=lambda cls,topic,callback,*a,**kw:callbacks.setdefault(topic,callback),destroy_subscription=lambda sub:None)
    gate=SensorReadiness(node,dict(head_depth=True,wrist_depth=False,tactile=False))
    assert not gate.snapshot()['ready']
    info=CameraInfo();info.header.stamp.sec=1
    callbacks['/cam0/depth/camera_info'](serialize_message(info))
    image=Image();image.header.stamp.sec=1;image.height=1;image.width=1;image.step=2;image.encoding='16UC1';image.data=bytes(2)
    callbacks['/cam0/depth/image_raw'](serialize_message(image))
    assert not gate.snapshot()['ready']
    # Repeated timestamps are not progress.
    callbacks['/cam0/depth/image_raw'](serialize_message(image))
    assert not gate.snapshot()['ready']
    image.header.stamp.nanosec=1
    callbacks['/cam0/depth/image_raw'](serialize_message(image))
    assert gate.snapshot()['ready']
    gate.close()
