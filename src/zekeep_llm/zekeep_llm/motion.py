"""Preflight every endpoint before sending a joint motion."""
import math
from .planner import validate_arguments

JOINT_NAMES=[f'joint{i}' for i in range(1,7)]
LIMITS=[(-2.58,2.58),(0,3.7),(-.01,3.7),(-1.57,1.57),(-1.57,1.57),(-1.57,1.57)]
MAX_SPEED=.30


def checked_joints(values):
    if len(values)!=6 or any(type(v) not in (float,int) or not math.isfinite(v) or not low<=v<=high for v,(low,high) in zip(values,LIMITS)):
        raise ValueError('六关节反馈/目标不是有效限位内 rad 数值')
    return list(map(float,values))


def joint_segments(current,arguments):
    validate_arguments('move_joints',arguments)
    current=checked_joints(current)
    delta=arguments['positions_rad']
    target=[x+y for x,y in zip(current,delta)] if arguments.get('relative') else delta
    targets=[]
    for _ in range(arguments.get('repeat',1)):
        if arguments.get('oscillate'):targets.append([x-y for x,y in zip(current,delta)])
        targets.append(target)
        if arguments.get('relative') and not arguments.get('oscillate'):targets.append(current)
    if arguments.get('oscillate'):targets.append(current)
    segments=[];previous=current
    for point in targets:
        point=checked_joints(point)
        duration=max(arguments.get('duration_s',2),max(abs(x-y) for x,y in zip(previous,point))*1.5/MAX_SPEED)
        segments.append((point,duration));previous=point
    if sum(t for _,t in segments)>1800:raise ValueError('按低速限制计算后超过 1800 秒')
    return segments
