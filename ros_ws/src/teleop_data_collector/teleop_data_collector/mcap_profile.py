"""Validate real assets and adapt the DAgger recording contract to teleoperation."""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from teleop_core.contract import SOURCE_COMMAND_TOPIC, VALIDATED_COMMAND_TOPIC, COMMAND_STATUS_TOPIC, ARM_STATE_TOPIC, SHARPA_COMMAND_TOPIC, SHARPA_STATE_TOPIC
from .mcap_actions import GROUPS
from .recording_options import optional_channels


def load_profile(path, options, identities=None):
    path=Path(path)
    if not path.is_file():raise ValueError(f'MCAP配置缺失：{path}；请运行 ops/setup/prepare_mcap.py')
    profile=deepcopy(yaml.safe_load(path.read_text()))
    for key in ('model_root','calibration_root'):
        value=Path(profile[key]).expanduser()
        if not value.is_absolute():value=path.parent/value
        value=value.resolve()
        if not value.is_dir() or not any(p.is_file() for p in value.rglob('*')):raise ValueError('MCAP真实资产缺失：'+key)
        profile[key]=str(value)
    for model in Path(profile['model_root']).rglob('*'):
        if model.suffix in ('.urdf','.xml','.sdf'):
            for element in ET.parse(model).iter():
                ref=element.attrib.get('filename') or (element.text.strip() if element.tag=='uri' and element.text else None)
                if ref and (Path(ref).is_absolute() or '://' in ref or not (model.parent/ref).is_file()):raise ValueError('Unmaterialized model reference: '+str(ref))
    clock=profile.get('clock_mapping',{})
    if clock.get('valid_from_ns',0)<=0 or clock.get('sync_state') not in ('locked','sdk_mapped'):raise ValueError('Verified clock mapping required')
    if not profile.get('session_id') or str(profile['session_id']).startswith('REQUIRED'):raise ValueError('Real session_id required')
    channels=[]
    for device in profile['devices']:
        for ch in device['channels']:
            source=ch['source_topic']
            if source.startswith(('/dagger/','/openpi/','/recording/')) or '/tactile/' in source or '/depth/' in source:continue
            if source==VALIDATED_COMMAND_TOPIC:continue
            if source in ('/sharpa/left/command','/sharpa/right/command'):continue
            if source==SOURCE_COMMAND_TOPIC:
                ch.update(adapter='arm_command',source_type='teleop_interfaces/msg/ArmCommand',type='sensor_msgs/msg/JointState')
            if ch.get('adapter') in ('dagger_ownership','split_joint_command','split_gateway_command','dagger_dispatch','dagger_snapshot','dagger_context'):continue
            if ch['type']=='foxglove_msgs/msg/CompressedVideo':
                camera=source.split('/')[1]
                actual=(identities or {}).get(camera+'_serial')
                if actual and actual not in ('REPLACE_AT_LAUNCH','not-recorded') and actual!=ch.get('serial_number'):raise ValueError('MCAP camera serial mismatch: '+camera)
                expected=18 if ch['role']=='head' else 16
                if ch.get('crf')!=expected or ch['width']%2 or ch['height']%2:raise ValueError('Invalid native RGB profile')
                if ch.get('color')!=dict(range='limited',matrix='bt709',primaries='bt709',transfer='iec61966-2-1') or not ch.get('color_source') or ch['color_source'].startswith('REQUIRED'):raise ValueError('Explicit RGB color evidence required')
            if ch['type']=='sensor_msgs/msg/CameraInfo':ch['storage_policy']='first_snapshot_and_calibration_changes'
            if ch.get('recording') in ('enabled','event_driven'):channels.append(ch)
    for group,(device,names) in GROUPS.items():
        arm=group.endswith('_arm');side=group.split('_')[0]
        channels.append(dict(source_topic=VALIDATED_COMMAND_TOPIC if arm else SHARPA_COMMAND_TOPIC.format(side=side),
            topic=f'/devices/{device}/action/command',type='sensor_msgs/msg/JointState',recording='enabled',
            adapter='split_gateway_command' if arm else 'hand_command',command_group=group,joint_names=list(names),
            source_type='sensor_msgs/msg/JointState',unit='rad',action_basis='absolute',value_origin='derived',
            command_semantics='gateway_validated_target_not_measured_state' if arm else 'sdk_submitted_target_not_measured_state'))
    required_sources={SOURCE_COMMAND_TOPIC,COMMAND_STATUS_TOPIC,'/tf_static','/tf',
        *[ARM_STATE_TOPIC.format(side=s) for s in ('left','right')],
        *[SHARPA_STATE_TOPIC.format(side=s) for s in ('left','right')],
        *[f'/cam{i}/color/{suffix}' for i in range(3) for suffix in ('image_raw','camera_info')]}
    missing=required_sources-{ch['source_topic'] for ch in channels}
    if missing:raise ValueError('MCAP profile missing native channels: '+', '.join(sorted(missing)))
    channels.extend(optional_channels(options))
    if len({c['topic'] for c in channels})!=len(channels):raise ValueError('Duplicate MCAP output topic')
    from .calibrated_tf import validate_specs
    validate_specs(profile.get('static_extrinsics',[]))
    hardware_identity=deepcopy(profile['devices'][0].get('hardware_identity',{}))
    for side in ('left','right'):
        actual=(identities or {}).get(side+'_sharpa_serial')
        if actual and hardware_identity.get(side+'_hand')!=actual:raise ValueError('MCAP hand serial mismatch: '+side)
    profile['devices']=[dict(device_id='rig',channels=channels,hardware_identity=hardware_identity,
        action_space=dict(control_mode='joint_position',basis='absolute',unit='rad',command_semantics='per_group_sources_declared_below',arm_command_source=VALIDATED_COMMAND_TOPIC,command_order=[f'{g}_{len(GROUPS[g][1])}' for g in ('left_arm','left_hand','right_arm','right_hand')],source_semantics='teleoperation arm source target',storage_layout='four-joint-state-command-streams-v2',command_topics={g:f'/devices/{d}/action/command' for g,(d,n) in GROUPS.items()},
            command_semantics_by_group={g:'gateway_validated_target' if g.endswith('_arm') else 'sdk_submitted_target' for g in GROUPS}))]
    profile['recording_options']=dict(options)
    return profile,channels
