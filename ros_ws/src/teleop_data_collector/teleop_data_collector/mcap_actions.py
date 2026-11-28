"""Standard command streams mapped from the canonical teleop contract."""
from teleop_core.contract import (LEFT_COMMAND_JOINT_NAMES, RIGHT_COMMAND_JOINT_NAMES,
    DATA_LEFT_ARM_JOINT_NAMES, DATA_RIGHT_ARM_JOINT_NAMES, SHARPA_JOINT_NAMES,
    dataset_arm_joint_name)

GROUPS={'left_arm':('fr3_left',DATA_LEFT_ARM_JOINT_NAMES),
        'right_arm':('fr3_right',DATA_RIGHT_ARM_JOINT_NAMES),
        'left_hand':('sharpa_left',tuple('left_'+n for n in SHARPA_JOINT_NAMES)),
        'right_hand':('sharpa_right',tuple('right_'+n for n in SHARPA_JOINT_NAMES))}


def joint_message(message, header, group):
    expected=GROUPS[group][1]
    names=list(message.name)
    if group.endswith('_arm'):
        names=[dataset_arm_joint_name(n) if n in LEFT_COMMAND_JOINT_NAMES+RIGHT_COMMAND_JOINT_NAMES else n for n in names]
    elif set(names)==set(SHARPA_JOINT_NAMES):
        names=[group.split('_')[0]+'_'+n for n in names]
    allowed=set(DATA_LEFT_ARM_JOINT_NAMES+DATA_RIGHT_ARM_JOINT_NAMES) if group.endswith('_arm') else set(expected)
    if not names or len(set(names))!=len(names) or not set(names)<=allowed or len(message.position)!=len(names):
        raise ValueError('Invalid command joint names/positions')
    present=set(names)&set(expected)
    if not present:return None
    if present!=set(expected):raise ValueError('Incomplete command side')
    indices={n:i for i,n in enumerate(names)}
    result=dict(header=header,name=list(expected))
    for field in ('position','velocity','effort'):
        values=list(getattr(message,field))
        if len(values) not in (0,len(names)):raise ValueError('Invalid JointState '+field)
        result[field]=[float(values[indices[n]]) for n in expected] if values else []
    return result


def split_gateway_message(message, header):
    return {g:v for g in ('left_arm','right_arm') if (v:=joint_message(message,header,g)) is not None}
