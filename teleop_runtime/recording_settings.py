"""Shared host/collector recording preferences; no ROS or device imports."""
from __future__ import annotations
import argparse
import fcntl
import json
import os
from pathlib import Path
from .transfer_settings import atomic_json

DEFAULTS = dict(format='mcap', head_depth=True, wrist_depth=True, tactile=False)


def config_path():
    return Path(os.environ.get('COLLECTION_RECORDING_CONFIG', str(Path.home()/'.config/gello-teleop/recording.json')))


def profile_path():
    return Path(os.environ.get('COLLECTION_MCAP_PROFILE', str(config_path().with_name('mcap-profile.yaml'))))


def validate_options(value):
    if not isinstance(value, dict) or set(value)-set(DEFAULTS):
        raise ValueError('Invalid recording options')
    result = dict(DEFAULTS, **value)
    if result['format'] not in ('mcap', 'rosbag') or any(type(result[k]) is not bool for k in ('head_depth','wrist_depth','tactile')):
        raise ValueError('Invalid recording format/sensor selection')
    return result


def read_settings():
    try:
        return validate_options(json.loads(config_path().read_text()))
    except FileNotFoundError:
        return dict(DEFAULTS)


def update_settings(**changes):
    path=config_path();path.parent.mkdir(parents=True,exist_ok=True)
    with path.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        result=validate_options(dict(read_settings(),**changes))
        atomic_json(path,result)
    return result


def camera_config(source):
    """Generate deterministic private profiles without editing installed camera files."""
    import yaml
    source=Path(source).resolve();options=read_settings()
    repo=Path(__file__).resolve().parents[1]
    config=yaml.safe_load(source.read_text())
    profiles={}
    for camera,entry in config['camera_bringup'].items():
        if camera not in ('cam0','cam1','cam2'):continue
        original=Path(entry['config_file'])
        if not original.is_absolute():
            original=repo/'ros_ws/src/teleop_camera_bringup/config'/original
        params=yaml.safe_load(original.read_text())
        enabled=options['head_depth'] if camera=='cam0' else options['wrist_depth']
        params['enable_depth']=enabled
        if enabled:
            params.update(depth_width=640,depth_height=400 if camera=='cam0' else 480,
                          depth_fps=20 if camera=='cam0' else 30,depth_format='Y16')
        profiles[camera]=params
    import hashlib
    signature=hashlib.sha256(json.dumps(dict(config=config,profiles=profiles),sort_keys=True).encode()).hexdigest()
    output=config_path().parent/'camera-recording'/signature
    output.mkdir(parents=True,exist_ok=True)
    for camera,params in profiles.items():
        target=output/(camera+'.yaml');text=yaml.safe_dump(params,sort_keys=False)
        if not target.exists():target.write_text(text)
        elif target.read_text()!=text:raise ValueError('Existing camera profile changed: '+str(target))
        config['camera_bringup'][camera]['config_file']=str(target)
    target=output/'cameras.yaml';text=yaml.safe_dump(config,sort_keys=False)
    if not target.exists() or target.read_text()!=text:target.write_text(text)
    return target


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--camera-config',type=Path)
    args=parser.parse_args()
    print(camera_config(args.camera_config) if args.camera_config else json.dumps(read_settings()))
