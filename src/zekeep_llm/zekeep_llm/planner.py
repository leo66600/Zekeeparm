"""Strict, shared LLM/MCP contracts for bounded robot tools."""
from dataclasses import dataclass, field
import json
import math
from typing import Any


def number(low, high):
    return {'type':'number','minimum':low,'maximum':high}


def vector(count, low, high):
    return {'type':'array','items':number(low,high),'minItems':count,'maxItems':count}


def schema(properties=None, required=()):
    return {'type':'object','properties':properties or {},'required':list(required),'additionalProperties':False}


POSE_PROPERTIES={'position_m':vector(3,-.7,.8),'rpy_rad':vector(3,-math.pi,math.pi)}
TOOL_SCHEMAS={name:schema() for name in (
    'get_status','get_robot_status','diagnose_ros','gravity_compensation_status','detect_blocks',
    'stop','safe_home','return_ready','open_gripper','grasp','enable_robot','disable_robot',
    'gravity_compensation_start','gravity_compensation_stop','record_start','record_stop','record_clear','record_replay','place_object')}
TOOL_SCHEMAS.update({
    'ik_check':schema(POSE_PROPERTIES,('position_m','rpy_rad')),
    'move_to_pose':schema({**POSE_PROPERTIES,'duration_s':number(.2,60)},('position_m','rpy_rad')),
    'set_gripper_opening_mm':schema({'opening_mm':number(0,70*1.35/1.45)},('opening_mm',)),
    'move_joints':schema({'positions_rad':vector(6,-3.7,3.7),'duration_s':number(.2,60),
                          'relative':{'type':'boolean'},'repeat':{'type':'integer','minimum':1,'maximum':20},
                          'oscillate':{'type':'boolean'}},('positions_rad',)),
    'pick_color':schema({'color':{'type':'string','enum':['red','blue','green','yellow','purple']},
                         'place_after':{'type':'boolean'}},('color',)),
})
TOOLS=set(TOOL_SCHEMAS)
READ_ONLY_TOOLS={'get_status','get_robot_status','diagnose_ros','gravity_compensation_status','ik_check','detect_blocks'}
MUTATING_TOOLS=TOOLS-READ_ONLY_TOOLS
LEASED_TOOLS={'stop','safe_home','return_ready','open_gripper','grasp','set_gripper_opening_mm',
              'move_to_pose','move_joints','record_replay','gravity_compensation_start'}
MAX_STEPS=32


def validate_value(value, rule, label):
    kind=rule['type']
    if kind=='object':
        if not isinstance(value,dict) or set(value)-set(rule['properties']) or not set(rule.get('required',()))<=set(value):
            raise ValueError(f'{label}: 参数缺失或包含未知字段')
        for key,item in value.items():validate_value(item,rule['properties'][key],f'{label}.{key}')
    elif kind=='array':
        if not isinstance(value,list) or not rule['minItems']<=len(value)<=rule['maxItems']:raise ValueError(f'{label}: 数组长度无效')
        for item in value:validate_value(item,rule['items'],label)
    elif kind in ('number','integer'):
        if type(value) not in (int,float) or (kind=='integer' and type(value) is not int) or not math.isfinite(value) or not rule['minimum']<=value<=rule['maximum']:
            raise ValueError(f'{label}: 数值必须有限且在允许范围内')
    elif kind=='boolean':
        if type(value) is not bool:raise ValueError(f'{label}: 必须是布尔值')
    elif kind=='string':
        if not isinstance(value,str) or value not in rule['enum']:raise ValueError(f'{label}: 不支持的值')


def validate_arguments(tool, arguments):
    if tool not in TOOLS:raise ValueError(f'工具不允许: {tool}')
    validate_value(arguments,TOOL_SCHEMAS[tool],tool)
    if tool=='move_joints':
        if arguments.get('oscillate') and not arguments.get('relative'):raise ValueError('往复摆动必须使用 relative=true')
        if arguments.get('repeat',1)>1 and not arguments.get('relative'):raise ValueError('重复运动必须指定相对偏移')
        if arguments.get('duration_s',2)*arguments.get('repeat',1)*2>1800:raise ValueError('运动总时长超过 1800 秒')
    if tool in ('ik_check','move_to_pose'):
        x,y,z=arguments['position_m']
        if x*x+y*y+z*z>.7**2 or z<0:raise ValueError('目标超出受限工作范围')
    return arguments


@dataclass(frozen=True)
class Step:
    tool: str
    arguments: dict=field(default_factory=dict)


@dataclass(frozen=True)
class Plan:
    summary: str
    steps: tuple[Step,...]
    @property
    def requires_confirmation(self):return any(step.tool in MUTATING_TOOLS for step in self.steps)


def parse_plan(raw):
    try:value=json.loads(raw)
    except json.JSONDecodeError as exc:raise ValueError(f'模型返回的不是有效 JSON: {exc.msg}') from exc
    if not isinstance(value,dict) or set(value)!={'summary','steps'}:raise ValueError('计划必须且只能包含 summary 和 steps')
    if not isinstance(value['summary'],str) or not 1<=len(value['summary'].strip())<=500:raise ValueError('summary 必须是 1 到 500 字符')
    if not isinstance(value['steps'],list) or len(value['steps'])>MAX_STEPS:raise ValueError(f'最多 {MAX_STEPS} 步')
    steps=[]
    for item in value['steps']:
        if not isinstance(item,dict) or set(item)!={'tool','arguments'} or not isinstance(item['tool'],str):raise ValueError('步骤格式无效')
        steps.append(Step(item['tool'],validate_arguments(item['tool'],item['arguments'])))
    if any(step.tool in ('stop','disable_robot') for step in steps) and len(steps)!=1:raise ValueError('停止/失能必须是唯一动作')
    return Plan(value['summary'].strip(),tuple(steps))


def plan_as_dict(plan) -> dict[str,Any]:
    return {'summary':plan.summary,'steps':[{'tool':step.tool,'arguments':step.arguments} for step in plan.steps]}
