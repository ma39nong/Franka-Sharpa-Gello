#!/usr/bin/env python3
"""Copy verified reference assets into this recorder's own host configuration.

Read-only reference checkout; does not open ROS nodes, devices, or edit episodes.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil
import sys
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from teleop_runtime.recording_settings import profile_path


def prepare(reference, output):
    reference=Path(reference).resolve();output=Path(output).resolve()
    profile=yaml.safe_load(reference.read_text())
    for key in ('model_root','calibration_root'):
        source=Path(profile[key]);source=source if source.is_absolute() else reference.parent/source
        if not source.is_dir():raise ValueError('Missing real asset: '+str(source))
        fingerprint=hashlib.sha256()
        for p in sorted(source.rglob('*')):
            if p.is_symlink():raise ValueError('Materialized assets required: '+str(p))
            if p.is_file():
                fingerprint.update(p.relative_to(source).as_posix().encode()+b'\0')
                with p.open('rb') as stream:
                    for block in iter(lambda:stream.read(1024*1024),b''):fingerprint.update(block)
        target=output.parent/'mcap-assets'/key/fingerprint.hexdigest()
        if not target.exists():
            target.parent.mkdir(parents=True,exist_ok=True);shutil.copytree(source,target)
        profile[key]=str(target)
    # Preserve real timestamp/color/calibration evidence, not DAgger execution settings.
    profile['session_id']='gello-'+profile['session_id']
    profile['asset_provenance']=dict(reference_profile=str(reference),reference_sha256=hashlib.sha256(reference.read_bytes()).hexdigest(),
        applicability='Same installed devices and mounting confirmed by operator; live serials and dimensions checked before capture')
    output.parent.mkdir(parents=True,exist_ok=True)
    text=yaml.safe_dump(profile,sort_keys=False,allow_unicode=True)
    if output.exists() and output.read_text()!=text:raise FileExistsError('Existing profile differs; choose --output for a new version: '+str(output))
    output.write_text(text)
    return output


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference',type=Path,required=True)
    parser.add_argument('--output',type=Path,default=profile_path())
    args=parser.parse_args();print(prepare(args.reference,args.output))
