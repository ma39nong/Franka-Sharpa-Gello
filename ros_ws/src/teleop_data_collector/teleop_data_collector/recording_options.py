"""Selected native channels and lossless integer depth encoding."""
import ctypes
import os
from pathlib import Path
import struct
import numpy as np


def optional_channels(options):
    channels=[]
    cameras=(['cam0'] if options['head_depth'] else [])+(['cam1','cam2'] if options['wrist_depth'] else [])
    for camera in cameras:
        channels.extend([
            dict(source_topic=f'/{camera}/depth/image_raw',topic=f'/devices/{camera}/cameras/depth/depth/image_raw/compressedDepth',
                 type='sensor_msgs/msg/CompressedImage',source_type='sensor_msgs/msg/Image',adapter='depth_rvl',recording='enabled',unit='mm',invalid_policy='zero_invalid'),
            dict(source_topic=f'/{camera}/depth/camera_info',topic=f'/devices/{camera}/cameras/depth/depth/camera_info',type='sensor_msgs/msg/CameraInfo',recording='enabled')])
    if options['tactile']:
        from .tactile_channels import channels as tactile
        channels.extend(tactile())
    return channels


_library=None

def depth_payload(message):
    global _library
    if message.encoding!='16UC1':raise ValueError('Native millimetre 16UC1 depth required; no quantization')
    rows=np.frombuffer(bytes(message.data),dtype='>u2' if message.is_bigendian else '<u2').reshape(message.height,message.step//2)
    pixels=np.ascontiguousarray(rows[:,:message.width],dtype=np.uint16).reshape(-1)
    if _library is None:
        path=os.environ.get('COLLECTION_RVL_LIBRARY',str(Path(__file__).with_name('mcap-rvl.so')))
        _library=ctypes.CDLL(path)
        _library.sharpa_rvl.argtypes=[ctypes.c_void_p,ctypes.c_size_t,ctypes.c_void_p]
        _library.sharpa_rvl.restype=ctypes.c_size_t
    output=np.empty(pixels.size*2+32,dtype='<u4')
    length=_library.sharpa_rvl(pixels.ctypes.data,pixels.size,output.ctypes.data)
    return struct.pack('<iffII',0,0.,0.,message.width,message.height)+output.tobytes()[:length]
