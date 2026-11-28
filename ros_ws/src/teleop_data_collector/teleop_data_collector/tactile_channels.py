"""Lossless tactile telemetry on the already-owned SDK handles; never sends commands."""
from array import array
import math
import queue
import threading
import time

import numpy as np

FINGERS=('thumb','index','middle','ring','pinky')
MANUAL='https://sharpa-robotics.github.io/sharpa-docs/#tactile-sensing'
FRAMES='https://github.com/sharpa-robotics/sharpa-urdf-usd-xml#model-description'
# Verified with both installed Wave hands, SDK 5.0.6, 2026-11-28.
IMAGE_SIGNALS={'RAW':('raw',(240,320)), 'DEFORM':('deformation',(240,240))}
# Preserve additional arrays if this SDK/device exposes them. Sparse contacts
# are conditional. Distributed variants require a verified descriptor before use.


def channel_index(side,finger):
    index=FINGERS.index(finger)
    return 9-index if side=='left' else index


def channels():
    result=[]
    for side in ('left','right'):
        for finger in FINGERS:
            base=f'/sharpa/{side}/tactile/{finger}'
            target=f'/devices/sharpa_{side}/tactile/{finger}'
            frame=f'{side}_{finger}_elastomer'
            common=dict(recording='enabled',value_origin='measured',frame_id=frame,
                timestamp_source='SDK frame.ts Unix seconds converted to integer nanoseconds',
                timestamp_limitation='SDK timestamp; physical exposure and device-host synchronization not independently certified',
                nominal_rate_hz=30,source_reference=MANUAL,coordinate_reference=FRAMES,
                axes='manufacturer right-handed elastomer frame; no axis permutation or sign change')
            for key,(name,shape) in IMAGE_SIGNALS.items():
                result.append(dict(common,source_topic=base+'/'+name,topic=target+'/image/'+name,
                    type='sensor_msgs/msg/Image',sdk_signal=key,encoding='mono8',native_dtype='uint8',
                    value_origin='measured' if key=='RAW' else 'derived',
                    native_shape=list(shape),height=shape[0],width=shape[1],dtype='uint8',unit='SDK grayscale code',compression='lossless MCAP Chunk Zstd',
                    upstream_codec='SDK JPEG transport decoded by SDK' if key=='RAW' else 'SDK native deformation image',
                    grid_axes=['row','column'],sdk_layout='leading batch singleton omitted; pixel order unchanged',
                    value_mapping='identity for RAW; nonlinear SDK deform_map_value(code) in mm for DEFORM'))
            result.append(dict(common,source_topic=base+'/wrench',topic=target+'/wrench',
                type='geometry_msgs/msg/WrenchStamped',sdk_signal='F6',value_origin='estimated',
                components=['Fx','Fy','Fz','Tx','Ty','Tz'],units=['N','N','N','N*m','N*m','N*m'],
                unit={'force':'N','torque':'N*m'},dtype='float64',
                native_dtype='float64 as exposed by installed Python SDK; underlying firmware dtype not exposed',
                torque_origin='thumb IP joint axis center' if finger=='thumb' else 'DIP joint axis center',
                conversion='identity; SDK engineering units, no raw-count conversion'))
            result.append(dict(common,source_topic=base+'/sample_info',topic=f'/recording/tactile/{side}/{finger}/sample_info',
                type='std_msgs/msg/UInt64MultiArray',adapter='tactile_sample_info',
                components=['sdk_frame_index','sdk_timestamp_ns','host_sdk_read_timestamp_ns'],
                units=['count','ns','ns'],native_dtype='uint64',value_origin='derived'))
            result.append(dict(common,source_topic=base+'/contact_point',topic=target+'/array/contact_point',
                type='sensor_msgs/msg/Image',sdk_signal='CONTACT_POINT',value_origin='estimated',recording='event_driven',presence_policy='conditional',
                native_dtype='preserved from SDK array; Image.encoding declares actual dtype',
                native_shape='SDK shape, stored in Image height/width/channels without resampling',
                components='SDK columns in original order; first two are deformation-map column,row',
                unit='pixel for column,row; any additional SDK components remain uninterpreted',
                semantics='raw SDK contact-point array, not a 3D point or a force measurement'))
    return result

