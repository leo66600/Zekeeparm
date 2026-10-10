"""ROS adapter exposing a deliberately small robot tool allowlist."""

from __future__ import annotations

import time
import json
import math
import threading
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import tempfile
from collections import deque

from ament_index_python.packages import get_package_prefix, PackageNotFoundError
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from zekeep_msgs.action import GripperGrasp, WebGrasp
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetStateValidity
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from .motion import JOINT_NAMES, MAX_SPEED, checked_joints, joint_segments
from .detection_history import recent_observations, WINDOW_S
from .planner import READ_ONLY_TOOLS, validate_arguments
from .startup import RobotStartup
from zekeep_msgs.msg import ArmStatus, JointMotorState
from zekeep_msgs.srv import GripperCommand, MoveToPoseIK, SetGripper, WebTeachCommand


class RobotTools(Node):
    def __init__(self, namespace: str = "zekeep", *, command_namespace=None, cancel_event=None, autostart=False) -> None:
        super().__init__("zekeep_llm")
        ns = namespace.strip("/")
        self.cancel_event = cancel_event
        self._autostart = autostart
        self._startup = RobotStartup(self, ns)
        self._positions = {}
        self._recording = False
        self._recorded = []
        self._record_started = 0.0
        self._record_lock = threading.Lock()
        self._detections = None
        self._detection_history = deque(maxlen=60)
        self._detection_received = 0.0
        self._vision_namespace = ns
        self._vision_topic = f"/{ns}/web/vision/detections"
        self._vision_process = None
        self._vision_log = None
        self._vision_file_lock = None
        self._vision_lock = threading.Lock()
        self.hold_unconfirmed = False
        self._ai_gravity = False
        self._delegated_busy = False
        self._heartbeat_future = None
        self._last_task_heartbeat = 0.0
        command_ns = (command_namespace or ns).strip("/")
        self._status = None
        self._joint_state_received = 0.0
        self._motor_status = {}
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            ArmStatus, f"/{ns}/arm_status", self._on_status, status_qos
        )
        self.create_subscription(
            JointState,
            f"/{ns}/joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        for joint_name in (f"joint{index}" for index in range(1, 7)):
            self.create_subscription(
                JointMotorState,
                f"/{ns}/joints/{joint_name}/state",
                self._on_motor_state,
                qos_profile_sensor_data,
            )
        self._triggers = {
            name: self.create_client(Trigger, endpoint)
            for name, endpoint in {
                "stop": f"/{ns}/stop",
                "enable_robot": f"/{ns}/enable",
                "disable_robot": f"/{ns}/disable",
                "gravity_compensation_start": f"/{command_ns}/gravity_compensation/start",
                "gravity_compensation_stop": f"/{command_ns}/gravity_compensation/stop",
                "gravity_compensation_status": f"/{ns}/gravity_compensation/status",
                "safe_home": f"/{command_ns}/safe_home",
                "return_ready": f"/{command_ns}/return_ready",
            }.items()
        }
        self._open = self.create_client(GripperCommand, f"/{command_ns}/gripper/open")
        self._grasp = ActionClient(self, GripperGrasp, f"/{command_ns}/gripper/grasp")

        self._trajectory = ActionClient(self, FollowJointTrajectory, f"/{command_ns}/follow_joint_trajectory")
        self._pick = ActionClient(self, WebGrasp, f"/{ns}/web/grasp")
        self._pick_only = ActionClient(self, WebGrasp, f"/{ns}/web/pick")
        self._place = ActionClient(self, WebGrasp, f"/{ns}/web/place")
        self._ik = self.create_client(MoveToPoseIK, f"/{ns}/move_to_pose_ik")
        self._set_gripper = self.create_client(SetGripper, f"/{command_ns}/gripper/set")
        self._gripper_parameters = self.create_client(GetParameters, '/ZekeepController/get_parameters')
        self._validity = self.create_client(GetStateValidity, '/check_state_validity')
        self._task_heartbeat = self.create_client(WebTeachCommand, f"/{ns}/teach/command")
        self._ordinary_gravity_stop = self.create_client(Trigger, f"/{ns}/gravity_compensation/stop")
        self.create_subscription(String, self._vision_topic, self._on_detections, 1)
        self._executor = SingleThreadedExecutor(context=self.context)
        self._executor.add_node(self)
        self._spin_lock = threading.Lock()

    def _spin_once(self, timeout_sec):
        # HTTP execution and cancellation may both wait on this node.
        with self._spin_lock:
            self._executor.spin_once(timeout_sec=timeout_sec)

    def destroy_node(self):
        with self._vision_lock:
            self._stop_vision()
        with self._spin_lock:
            self._executor.remove_node(self)
            self._executor.shutdown()
            return super().destroy_node()

    def _on_status(self, message: ArmStatus) -> None:
        self._status = message

    def _on_joint_state(self, message: JointState) -> None:
        values=dict(zip(message.name,message.position))
        try:checked_joints([values[name] for name in JOINT_NAMES])
        except (KeyError,ValueError):return
        now=time.monotonic()
        self._positions=values
        self._joint_state_received=now
        with self._record_lock:
            if self._recording and (not self._recorded or now-self._record_started-self._recorded[-1]['time']>=.05):
                if len(self._recorded)>=10000:
                    self._recording=False
                else:
                    self._recorded.append({'time':now-self._record_started,'positions':[values[name] for name in JOINT_NAMES],
                                           'width_m':2*values['gripper_joint'] if 'gripper_joint' in values else None})

    def _on_motor_state(self, message: JointMotorState) -> None:
        self._motor_status[message.joint_name] = (
            int(message.status_code),
            time.monotonic(),
        )

    def execute(self, tool: str, arguments=None) -> str:
        arguments=validate_arguments(tool,arguments or {})
        if tool not in READ_ONLY_TOOLS | {'stop','disable_robot','gravity_compensation_stop'} and self.cancel_event is not None and self.cancel_event.is_set():
            raise RuntimeError('AI execution cancelled')
        self.prepare(tool)
        if tool in ('get_status','get_robot_status'):return self.status_text()
        if tool=='diagnose_ros':
            return json.dumps({'nodes':self.get_node_names(),'services':dict(self.get_service_names_and_types()),
                               'topics':dict(self.get_topic_names_and_types()),'joint_age_s':self._heartbeat_age()},ensure_ascii=False)
        if tool=='gravity_compensation_status':
            response=self._call_service(self._triggers[tool],Trigger.Request(),tool,monitor_motion=False)
            if not response.success:raise RuntimeError(response.message)
            return response.message
        if tool=='detect_blocks':return json.dumps(self.detect_blocks(),ensure_ascii=False)
        if tool=='ik_check':return json.dumps({'success':True,'positions_rad':self.ik(arguments)},ensure_ascii=False)
        if tool in ('stop','disable_robot','gravity_compensation_stop'):
            if tool=='gravity_compensation_stop' and not self._ai_gravity:
                response=self._call_service(self._ordinary_gravity_stop,Trigger.Request(),tool)
                if not response.success:raise RuntimeError(response.message)
                return response.message
            result=self._call_trigger(tool)
            self._ai_gravity=False
            return result
        if tool=='enable_robot':
            self._wait_for_fresh_telemetry()
            return self._call_trigger(tool)
        if tool=='record_clear':
            with self._record_lock:
                if self._recording:raise RuntimeError('先 record_stop，再清除记录')
                self._recorded=[]
            return 'record cleared (memory only)'
        if tool=='record_stop':
            with self._record_lock:self._recording=False
            return f'record stopped: {len(self._recorded)} samples'
        if tool=='record_start':
            self._wait_for_fresh_telemetry()
            with self._record_lock:
                if self._recording or self._recorded:raise RuntimeError('已有记录；停止并明确清除后再录制，不静默覆盖')
                self._record_started=time.monotonic();self._recording=True
            return 'record started: real joint feedback; gravity mode unchanged'
        self._require_ready_status()
        if tool in self._triggers:
            result=self._call_trigger(tool)
            if tool=='gravity_compensation_start':self._ai_gravity=True
            return result
        if tool=='set_gripper_opening_mm':return self.set_opening(arguments['opening_mm'])
        if tool=='move_joints':return self.move_joints(arguments)
        if tool=='move_to_pose':
            q=self.ik(arguments)
            return self.move_joints({'positions_rad':q,'duration_s':arguments.get('duration_s',2)})
        if tool=='pick_color':return self.pick_color(arguments['color'],place_after=arguments.get('place_after',True))
        if tool=='place_object':return self.run_web_task(self._place,WebGrasp.Goal(),'place_object')
        if tool=='record_replay':return self.replay_record()
        if tool=='open_gripper':
            response=self._call_service(self._open,GripperCommand.Request(position=0.0,timeout=3.0),'gripper/open')
            if not response.success:raise RuntimeError(response.message or '打开夹爪失败')
            return response.message or f'夹爪已打开到 {response.reached_position:.3f} rad'
        if tool=='grasp':return self._call_grasp()
        raise ValueError(f'未知工具: {tool}')

    def prepare(self, tool):
        if not self._autostart or tool in ('diagnose_ros', 'stop', 'disable_robot',
                                         'gravity_compensation_stop', 'record_stop', 'record_clear'):
            return
        if tool in ('enable_robot', 'detect_blocks', 'move_joints', 'move_to_pose',
                    'safe_home', 'return_ready', 'record_replay', 'pick_color', 'place_object'):
            self._startup.ensure('moveit', self._validity.service_is_ready)
        self._startup.ensure('driver', self._triggers['enable_robot'].service_is_ready)
        self._spin_until(self._telemetry_is_fresh, 10, '驱动已启动但未收到新鲜关节/电机反馈')

    def poll(self):
        for _ in range(16):self._spin_once(timeout_sec=0)
        if self._recording and self._heartbeat_age()>.5:
            self._recording=False
            raise RuntimeError('记录反馈过期；录制已停止')

    def _on_detections(self,message):
        try:
            body=json.loads(message.data)
            if not isinstance(body,dict):return
            self._detections=body;self._detection_received=time.monotonic()
            if body.get('calibration_status') != 'CALIBRATED_STATIONARY':
                self._detection_history.clear()
            elif isinstance(body.get('detections'), list):
                self._detection_history.append(body)
        except (ValueError,TypeError):pass

    def _start_vision(self):
        with self._vision_lock:
            if self.count_publishers(self._vision_topic) or self._vision_process is not None:
                return
            try:
                prefix = Path(get_package_prefix('zekeep_shadow'))
            except PackageNotFoundError as exc:
                raise RuntimeError('自动感知需要已构建并 source 的 zekeep_shadow 包') from exc
            workspace = os.environ.get('ZKEEP_WS')
            if not workspace:
                workspace = next((p for p in prefix.parents if (p/'src/zekeep_grasp/config/default.yaml').is_file()), None)
            if workspace is None:
                raise RuntimeError('无法定位感知工作区；请设置 ZKEEP_WS')
            workspace = Path(workspace).expanduser().resolve()
            miniforge_dir = Path(os.environ.get('ZKEEP_MINIFORGE_DIR') or str(Path.home()/'miniforge3')).expanduser()
            python = Path(os.environ.get('ZKEEP_VISION_PYTHON', str(miniforge_dir/'envs/rebotarm/bin/python'))).expanduser()
            entry = prefix/'lib/zekeep_shadow/web_vision'
            if not python.is_file() or not os.access(python, os.X_OK) or not entry.is_file():
                raise RuntimeError('视觉 Python 或 web_vision 不可用；检查 ZKEEP_VISION_PYTHON 和 zekeep_shadow 构建')
            directory = Path.home()/'.local/share/zekeep'
            directory.mkdir(parents=True, exist_ok=True)
            lock = (directory/'llm_vision.lock').open('a+b')
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                lock.close()
                return  # Another assistant is already starting the same camera.
            self._vision_file_lock = lock
            try:
                self._vision_log = tempfile.TemporaryFile()
                # Reset SDK ABI paths, then load only ROS and this workspace for vision.
                command = ('set -e; unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH PYTHONNOUSERSITE; '
                           'source /opt/ros/humble/setup.bash; source "$1/install/setup.bash"; '
                           'exec "$2" "$3" --ros-args -p "perception_root:=$1/src/zekeep_grasp" '
                           '-p "namespace:=$4" -p grasp_backend:=yolo-graspnet')
                env = os.environ.copy()
                for name in ('ZKEEP_LLM_API_KEY', 'ZKEEP_VLM_API_KEY'):
                    env.pop(name, None)
                self._vision_process = subprocess.Popen(
                    ['/bin/bash', '-c', command, 'zekeep-vision', str(workspace), str(python), str(entry), self._vision_namespace],
                    env=env, stdout=self._vision_log, stderr=subprocess.STDOUT,
                )
                self.get_logger().info('正在自动启动相机/YOLO 感知；首次加载最多等待 45 秒')
            except Exception:
                self._stop_vision()
                raise

    def _stop_vision(self):
        if self._vision_process is not None:
            if self._vision_process.poll() is None:
                self._vision_process.send_signal(signal.SIGINT)
                try:
                    self._vision_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self._vision_process.kill()
                    self._vision_process.wait(timeout=3)
            self._vision_process = None
        for name in ('_vision_log', '_vision_file_lock'):
            value = getattr(self, name)
            if value is not None:
                value.close()
                setattr(self, name, None)

    def _vision_ready(self):
        with self._vision_lock:
            if self._vision_process is not None and self._vision_process.poll() is not None:
                self._vision_log.seek(0, os.SEEK_END)
                self._vision_log.seek(max(0, self._vision_log.tell()-4096))
                detail = self._vision_log.read().decode(errors='replace').strip()
                self._stop_vision()
                raise RuntimeError(f'相机/感知启动失败: {detail}')
            return (self._detections is not None and time.monotonic()-self._detection_received <= 1
                    and self._detections.get('calibration_status') != 'LOADING_YOLO_GRASPNET')

    def detect_blocks(self):
        deadline=time.monotonic()+WINDOW_S
        self._spin_until(lambda: time.monotonic() >= deadline, WINDOW_S+1, '检测观测窗口超时')
        if not self._vision_ready():
            self._start_vision()
            self._spin_until(self._vision_ready, 45, '相机/感知未在 45 秒内产生检测；检查相机占用和视觉环境')
            deadline=time.monotonic()+WINDOW_S
            self._spin_until(lambda: time.monotonic() >= deadline, WINDOW_S+1, '检测观测窗口超时')
        with self._spin_lock:
            body=self._detections
            frames=tuple(self._detection_history)
        if not body or time.monotonic()-self._detection_received>1:raise RuntimeError('没有新鲜物块检测；需启动相机/感知后端')
        stamp=body.get('stamp',{})
        age=self.get_clock().now().nanoseconds/1e9-(stamp.get('sec',0)+stamp.get('nanosec',0)/1e9)
        if not 0<=age<=1 or body.get('calibration_status')!='CALIBRATED_STATIONARY':
            raise RuntimeError(f"检测过期或标定/静止状态无效: {body.get('calibration_status')}；检测年龄 {age:.2f} 秒")
        targets=body.get('detections',[])
        if not isinstance(targets,list):raise RuntimeError('检测数据格式无效')
        observations=recent_observations(frames, self.get_clock().now().nanoseconds/1e9, body.get('calibration_identity'))
        return {'frame_id':body.get('frame_id'),'detections':targets,'calibration_status':body['calibration_status'],
                'observations':observations,'observation_window_s':WINDOW_S}

    def ik(self,arguments):
        self._wait_for_fresh_telemetry()
        pose=Pose();pose.position.x,pose.position.y,pose.position.z=map(float,arguments['position_m'])
        r,p,y=[value/2 for value in arguments['rpy_rad']]
        cr,cp,cy,sr,sp,sy=math.cos(r),math.cos(p),math.cos(y),math.sin(r),math.sin(p),math.sin(y)
        pose.orientation.w=cr*cp*cy+sr*sp*sy;pose.orientation.x=sr*cp*cy-cr*sp*sy
        pose.orientation.y=cr*sp*cy+sr*cp*sy;pose.orientation.z=cr*cp*sy-sr*sp*cy
        result=self._call_service(self._ik,MoveToPoseIK.Request(target_pose=pose),'ik_check',monitor_motion=False)
        if not result.success:raise RuntimeError(result.message or 'IK 失败；不执行近似解')
        return checked_joints(result.q_solution)

    def set_opening(self,millimeters):
        params=self._call_service(self._gripper_parameters,GetParameters.Request(names=['web_gripper_open_rad','web_gripper_close_rad','web_gripper_max_width_m']),
                                  'gripper parameters',monitor_motion=False)
        if len(params.values)!=3 or any(value.type!=3 for value in params.values):raise RuntimeError('夹爪标定参数不可用')
        opened,closed,width=[value.double_value for value in params.values]
        if not all(math.isfinite(value) for value in (opened,closed,width)) or not 0<width<=.07*1.35/1.45 or opened<=closed or millimeters/1000>width:
            raise RuntimeError('夹爪映射/开口超出实际标定范围')
        result=self._call_service(self._set_gripper,SetGripper.Request(position=closed+(opened-closed)*millimeters/1000/width,max_effort=0.0),'gripper/set')
        if not result.success:raise RuntimeError('设置夹爪失败')
        return f'opening {millimeters} mm completed'

    def collision_check(self,current,segments):
        if not self._validity.wait_for_service(timeout_sec=1):raise RuntimeError('MoveIt /check_state_validity 不可用；拒绝关节/姿态运动')
        previous=current;seen=set()
        for target,_ in segments:
            count=max(1,math.ceil(max(abs(x-y) for x,y in zip(previous,target))/.03))
            for index in range(count+1):
                q=[a+(b-a)*index/count for a,b in zip(previous,target)];key=tuple(round(v,6) for v in q)
                if key in seen:continue
                seen.add(key)
                request=GetStateValidity.Request();request.group_name='arm';request.robot_state.joint_state.name=JOINT_NAMES+['gripper_joint','right_joint']
                request.robot_state.is_diff=True  # Keep held-object geometry from the planning scene.
                finger=float(self._positions.get('gripper_joint',.035))
                request.robot_state.joint_state.position=q+[finger,finger]
                result=self._call_service(self._validity,request,'collision_check',monitor_motion=False)
                if not result.valid:raise RuntimeError('碰撞检查失败；整个关节任务未发送')
            previous=target

    def run_action(self,client,goal,label,timeout):
        if self.cancel_event is not None and self.cancel_event.is_set():raise RuntimeError('execution cancelled before Action')
        if not client.wait_for_server(timeout_sec=1):raise RuntimeError(f'Action 不可用: {label}')
        handle=None;terminal=False;sent=None;future=None
        try:
            sent=client.send_goal_async(goal);self._spin_until(sent.done,3,label+' goal timeout',monitor_motion=True)
            handle=sent.result()
            if not handle or not handle.accepted:raise RuntimeError(label+' rejected')
            future=handle.get_result_async();self._spin_until(future.done,timeout,label+' timeout',monitor_motion=True)
            response=future.result();terminal=True
            if not response:raise RuntimeError(label+' Action 无结果')
            result=response.result
            if response.status!=4:
                detail=getattr(result,'message','') or getattr(result,'error_string','') or f'Action status={response.status}'
                raise RuntimeError(f'{label}: {detail}')
            if getattr(result,'success',True) is not True or getattr(result,'error_code',0)!=0:raise RuntimeError(getattr(result,'message',getattr(result,'error_string','Action failed')))
            return result
        except Exception:
            if sent is not None and handle is None:
                self.hold_unconfirmed=True
                def cancel_late(future):
                    try:
                        late=future.result()
                        if late and late.accepted:late.cancel_goal_async()
                    except Exception:pass
                sent.add_done_callback(cancel_late)
            if handle and handle.accepted and not terminal:
                try:
                    cancellation=handle.cancel_goal_async();self._spin_until(cancellation.done,3,label+' cancel timeout')
                    if not cancellation.result().goals_canceling:raise RuntimeError(label+' cancel unconfirmed')
                    terminal_future=future or handle.get_result_async()
                    self._spin_until(terminal_future.done,8,label+' cancellation result timeout')
                    final=terminal_future.result()
                    message=getattr(final.result,'message','') if final else 'hold unconfirmed'
                    if 'hold unconfirmed' in message or 'ownership retained' in message:raise RuntimeError(message)
                except Exception:
                    self.hold_unconfirmed=True
                    raise
            raise

    def move_joints(self,arguments):
        self._wait_for_fresh_telemetry();current=checked_joints([self._positions[name] for name in JOINT_NAMES])
        segments=joint_segments(current,arguments);self.collision_check(current,segments)
        for target,duration in segments:
            self._require_ready_status()
            self.collision_check([self._positions[name] for name in JOINT_NAMES],[(target,duration)])
            goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=JOINT_NAMES
            point=JointTrajectoryPoint();point.positions=target;point.velocities=[0.0]*6
            point.time_from_start.sec=int(duration);point.time_from_start.nanosec=int((duration-int(duration))*1e9)
            goal.trajectory.points=[point];self.run_action(self._trajectory,goal,'move_joints',duration+10)
        return f'{len(segments)} joint segments completed; peak speed <= {MAX_SPEED:.2f} rad/s'

    def replay_record(self):
        with self._record_lock:
            if self._recording:raise RuntimeError('先停止录制')
            points=list(self._recorded)
        if not points:raise RuntimeError('没有记录')
        # Validate all endpoints before the first motion; reuse the same checked action path.
        self.collision_check([self._positions[name] for name in JOINT_NAMES],[(checked_joints(point['positions']),2) for point in points])
        expected=[self._positions[name] for name in JOINT_NAMES];total=0.0;previous=0.0
        for point in points:
            if point['width_m'] is not None and (not math.isfinite(point['width_m']) or not 0<=point['width_m']<=.07*1.35/1.45):raise RuntimeError('记录夹爪反馈无效')
            segment=joint_segments(expected,{'positions_rad':point['positions'],'duration_s':min(60,max(.2,point['time']-previous))})[0]
            total+=segment[1]+(3 if point['width_m'] is not None else 0)
            expected=point['positions'];previous=point['time']
        if total>1800:raise RuntimeError('回放预计超过1800秒；请录制更短轨迹')
        previous=0.0
        for point in points:
            self.move_joints({'positions_rad':point['positions'],'duration_s':min(60,max(.2,point['time']-previous))})
            if point['width_m'] is not None:self.set_opening(point['width_m']*1000)
            previous=point['time']
        return f'record replay completed: {len(points)} samples (low-speed retiming)'

    def pick_color(self,color,*,place_after=True):
        detected=self.detect_blocks()
        targets=[item for item in detected['detections'] if item.get('class_name', item.get('color')) in (color, f'{color} block')]
        if not targets:raise RuntimeError(f'当前没有可执行的 {color} 物块检测')
        known=[item for item in detected.get('observations', []) if item['class_name'] in (color, f'{color} block')]
        if len(targets)!=1 or len(known)>1:raise RuntimeError('多个同色目标；请分开物块后重新检测，不能任意猜测')
        client=self._pick if place_after else self._pick_only
        return self.run_web_task(client,WebGrasp.Goal(detection_id=targets[0]['id']),'pick_color')

    def run_web_task(self,client,goal,label):
        self._delegated_busy=True
        try:
            result=self.run_action(client,goal,label,600)
            return result.message
        finally:self._delegated_busy=False

    def status_text(self) -> str:
        self._spin_until(lambda: self._status is not None, 3.0, "arm_status 不可用")
        self._wait_for_fresh_telemetry()
        status = self._status
        heartbeat_age = self._heartbeat_age()
        return (
            f"enabled={status.enabled}, control_loop={status.control_loop_active}, "
            f"state={status.state_machine}, errors={list(status.error_codes)}, "
            f"joint_state_age={heartbeat_age:.2f}s"
        )

    def _require_ready_status(self) -> None:
        self.poll()
        self._spin_until(lambda: self._status is not None, 3.0, "arm_status 不可用")
        self._wait_for_fresh_telemetry()
        status = self._status
        self._require_live_telemetry()
        if not status.enabled or not status.control_loop_active:
            raise RuntimeError("机械臂未使能或控制循环未运行；LLM 不会自动使能")
        if status.error_codes:
            raise RuntimeError(f"机械臂存在错误: {list(status.error_codes)}")
        if status.state_machine != "IDLE":
            raise RuntimeError(f"机械臂当前非空闲: {status.state_machine}")

    def _wait_for_fresh_telemetry(self) -> None:
        try:
            self._spin_until(
                self._telemetry_is_fresh,
                3.0,
                "未收到新鲜的关节/电机状态",
            )
        except TimeoutError:
            self._require_live_telemetry()
            raise

    def _telemetry_is_fresh(self) -> bool:
        now = time.monotonic()
        if self._joint_state_received <= 0.0 or now - self._joint_state_received > 0.5:
            return False
        return all(
            sample is not None and now - sample[1] <= 0.5
            for sample in (
                self._motor_status.get(f"joint{index}") for index in range(1, 7)
            )
        )

    def _require_live_telemetry(self) -> None:
        heartbeat_age = self._heartbeat_age()
        if heartbeat_age > 0.5:
            raise RuntimeError(f"joint_states 已过期: {heartbeat_age:.2f}s")
        self._require_healthy_motors()

    def _require_healthy_motors(self) -> None:
        now = time.monotonic()
        missing = []
        stale = []
        faults = []
        for name in (f"joint{index}" for index in range(1, 7)):
            sample = self._motor_status.get(name)
            if sample is None:
                missing.append(name)
            elif now - sample[1] > 0.5:
                stale.append(name)
            # Damiao status low nibble: 0=disabled, 1=enabled; other values are faults.
            elif sample[0] not in (0, 1):
                faults.append(f"{name}={sample[0]}")
        if missing:
            raise RuntimeError(f"缺少电机状态: {missing}")
        if stale:
            raise RuntimeError(f"电机状态已过期: {stale}")
        if faults:
            raise RuntimeError(f"电机故障状态: {faults}")

    def _heartbeat_age(self) -> float:
        if self._joint_state_received <= 0.0:
            return float("inf")
        return time.monotonic() - self._joint_state_received

    def _call_trigger(self, name: str) -> str:
        timeout = 60.0 if name in {"safe_home", "return_ready"} else 10.0
        response = self._call_service(
            self._triggers[name], Trigger.Request(), name, timeout
        )
        if not response.success:
            raise RuntimeError(response.message or f"{name} 失败")
        return response.message or f"{name} 成功"

    def _call_service(self, client, request, label: str, timeout_s: float = 10.0, *, monitor_motion=True):
        if monitor_motion and label not in ('stop','disable_robot','gravity_compensation_stop') and self.cancel_event is not None and self.cancel_event.is_set():
            raise RuntimeError('execution cancelled before service')
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"ROS 服务不可用: {label}")
        future = client.call_async(request)
        try:
            self._spin_until(
                future.done,
                timeout_s,
                f"ROS 服务超时: {label}",
                monitor_motion=monitor_motion and label != "stop",
            )
        except (KeyboardInterrupt, RuntimeError, TimeoutError):
            if monitor_motion and label != "stop":
                self.stop_best_effort()
            raise
        response = future.result()
        if response is None:
            raise RuntimeError(f"ROS 服务无响应: {label}")
        return response

    def _call_grasp(self) -> str:
        goal=GripperGrasp.Goal(closing_torque=0.0,hold_torque=0.0,timeout=0.0)
        result=self.run_action(self._grasp,goal,'gripper/grasp',15)
        return result.message or '已检测并夹持物体'

    def stop_best_effort(self) -> None:
        try:
            client = self._triggers["stop"]
            if not client.wait_for_service(timeout_sec=1.0):
                return
            future = client.call_async(Trigger.Request())
            self._spin_until(future.done, 3.0, "stop 超时")
            response = future.result()
            if response is None or not response.success:
                message = response.message if response is not None else "无响应"
                self.get_logger().error(f"紧急停止请求失败: {message}")
        except Exception as exc:
            self.get_logger().error(f"紧急停止请求失败: {exc}")

    def _spin_until(
        self,
        predicate,
        timeout_s: float,
        message: str,
        *,
        monitor_motion: bool = False,
    ) -> None:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not predicate():
            if self._delegated_busy and time.monotonic()-self._last_task_heartbeat>.5:
                if self._heartbeat_future is None or self._heartbeat_future.done():
                    self._heartbeat_future=self._task_heartbeat.call_async(WebTeachCommand.Request(command='status'))
                    self._last_task_heartbeat=time.monotonic()
            if time.monotonic() >= deadline:
                raise TimeoutError(message)
            self._spin_once(timeout_sec=0.05)
            if monitor_motion:
                if self.cancel_event is not None and self.cancel_event.is_set():
                    raise RuntimeError("AI execution cancelled; hold requested")
                self._require_live_telemetry()
        if not predicate():
            raise RuntimeError("ROS 已停止")
