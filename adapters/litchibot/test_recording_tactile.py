"""Native tactile payload integrity, no SDK or hardware initialization."""
import time
import numpy as np
import pytest
from adapters.litchibot.recording_tactile import decode_payload,image_fields,channel_index


def payload():
    stamp=time.time_ns()
    content=dict(RAW=np.arange(240*320,dtype=np.uint8).reshape(240,320),
        DEFORM=np.arange(240*240,dtype=np.uint8).reshape(240,240),F6=np.array([1.,2.,3.,4.,5.,6.],dtype=np.float64),
        CONTACT_POINT=np.array([[1.,2.,np.nan]],dtype=np.float32))
    return dict(channel=channel_index('left','index'),ts=stamp/1e9,frame_id=17,content=content),stamp


def test_sdk_capture_time_pixels_force_and_conditional_contacts():
    source,received=payload()
    stamp,index,messages=decode_payload(source,'left','index',received)
    assert index==17 and abs(stamp-received)<1000
    assert messages['raw']['data']==source['content']['RAW'].tobytes()
    assert messages['deformation']['data']==source['content']['DEFORM'].tobytes()
    assert messages['wrench']==[1.,2.,3.,4.,5.,6.]
    assert messages['contact_point']['data']==source['content']['CONTACT_POINT'].tobytes()
    source['content']['CONTACT_POINT']=None
    assert 'contact_point' not in decode_payload(source,'left','index',received)[2]


def test_tactile_rejects_wrong_route_dtype_and_clock_domain():
    source,received=payload();source['channel']=99
    with pytest.raises(ValueError):decode_payload(source,'left','index',received)
    source,received=payload();source['content']['DEFORM']=np.zeros((240,240),dtype=np.uint16)
    with pytest.raises(ValueError):decode_payload(source,'left','index',received)
    source,received=payload();source['ts']=123.
    with pytest.raises(ValueError):decode_payload(source,'left','index',received)


def test_array_encoding_preserves_nonfinite_bits_and_layout():
    bits=np.array([[0x7fc00011,0xff800000],[0x3f800000,0]],dtype='<u4')
    values=bits.view('<f4')
    encoded=image_fields(values)
    assert encoded['encoding']=='32FC1' and encoded['step']==8
    assert encoded['data']==bits.tobytes()


def test_recording_sdk_reads_remain_on_owner_and_publication_is_off_thread():
    pytest.importorskip('rclpy')
    from types import SimpleNamespace as NS
    import threading
    from adapters.litchibot.recording_tactile import TactileTelemetry
    owner=threading.get_ident();reads=[];published=[]
    def read(channel,timeout):
        assert timeout==0.
        reads.append(threading.get_ident());source,stamp=payload()
        source.update(channel=channel,frame_id=len(reads),ts=(time.time_ns()+len(reads)*1_000_000)/1e9)
        source['content']['CONTACT_POINT']=None
        return source
    node=NS(create_publisher=lambda *args:NS(publish=lambda msg:published.append(threading.get_ident())),
        get_logger=lambda:NS(error=lambda text:None))
    telemetry=TactileTelemetry(node,{'left':NS(tactile_available=True,_wave=NS(fetch_tactile_frame=read))},batch_size=1)
    telemetry.start()
    try:
        for _ in range(6):telemetry.poll()
        deadline=time.monotonic()+2
        while len(published)<24 and time.monotonic()<deadline:time.sleep(.005)
        assert reads==[owner]*6
        assert telemetry.error is None,telemetry.error
        assert len(published)==24 and all(thread!=owner for thread in published)
    finally:telemetry.close()
