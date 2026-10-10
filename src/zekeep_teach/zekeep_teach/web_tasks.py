"""ROS session API and cancelable single-target grasp Action for the web console."""
from __future__ import annotations

import json
from pathlib import Path
import sys
import threading
import time

import numpy as np

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionIK
from tf2_ros import Buffer, TransformListener
from zekeep_msgs.action import WebGrasp
from zekeep_msgs.srv import WebTeachCommand, WebTaskLease
from zekeep_msgs.msg import ArmStatus

from .cli import build_teach_config, default_config_path, load_config
from .robot import RosTeachRobot
from .ros_client import TeachRosClient
from .web_session import DetectionStore, WebSession


class WebTasks(Node):
    def __init__(self):
        super().__init__('zekeep_web_tasks')
        for name, value in (
            ('motion_authorized', False), ('namespace', 'zekeep'),
            ('teaching_config', str(default_config_path())),
            ('session_directory', str(Path.home()/'.local/share/zekeep/teach_sessions')),
            ('perception_root', str(Path(__file__).resolve().parents[2]/'zekeep_grasp')),
            ('perception_config', 'config/default.yaml'),
            ('grasp_backend', 'fixed'),
            ('grasp_geometry_confirmed', False), ('grasp_rpy_rad', [0.0,0.0,0.0]),
            ('object_size_m', [0.03,0.03,0.03]), ('grasp_surface_offset_m', 0.0),
            ('grasp_clearance_m', 0.08), ('grasp_max_velocity_rad_s', 0.3),
        ):
            self.declare_parameter(name, value)
        self.namespace = str(self.get_parameter('namespace').value).strip('/')
        self.backend = str(self.get_parameter('grasp_backend').value)
        if self.backend not in ('fixed', 'yolo-graspnet'):
            raise ValueError('unknown grasp backend')
        self.group = ReentrantCallbackGroup()
        self.lease_client = self.create_client(WebTaskLease, f"/{self.namespace}/web_task/lease", callback_group=self.group)
        self.joints_received = 0.0
        self.arm_received = 0.0
        self.arm_enabled = False
        self.create_subscription(JointState, f'/{self.namespace}/joint_states', self.joints_callback, qos_profile_sensor_data)
        self.create_subscription(ArmStatus, f'/{self.namespace}/arm_status', self.arm_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.moveit_clients = [self.create_client(kind, name) for kind, name in (
            (GetCartesianPath, '/compute_cartesian_path'), (GetMotionPlan, '/plan_kinematic_path'),
            (GetPositionIK, '/compute_ik'))]
        self.session = WebSession(self.make_robot, self.get_parameter('session_directory').value,
            authorized=bool(self.get_parameter('motion_authorized').value),
            acquire=lambda owner: self.lease(owner, True), release=lambda owner: self.lease(owner, False))
        self.detections = DetectionStore()
        self.detection_lock = threading.RLock()
        self.grasp_lock = threading.Lock()
        self.runner = None
        self.last_status_at = 0.0
        self.lease_renewal = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.status_pub = self.create_publisher(String, f'/{self.namespace}/web_tasks/status', 1)
        self.create_subscription(String, f'/{self.namespace}/web/vision/detections', self.detection_callback, 1)
        self.create_service(WebTeachCommand, f'/{self.namespace}/teach/command', self.teach_command, callback_group=self.group)
        self.create_service(Trigger, f'/{self.namespace}/web_tasks/cancel', self.cancel, callback_group=self.group)
        self.grasp_action = ActionServer(self, WebGrasp, f'/{self.namespace}/web/grasp',
            execute_callback=self.grasp, goal_callback=self.accept_grasp,
            cancel_callback=self.cancel_grasp, callback_group=self.group)
        self.pick_action = ActionServer(self, WebGrasp, f'/{self.namespace}/web/pick',
            execute_callback=lambda handle: self.grasp(handle, pick_only=True),
            goal_callback=self.accept_grasp, cancel_callback=self.cancel_grasp, callback_group=self.group)
        self.place_action = ActionServer(self, WebGrasp, f'/{self.namespace}/web/place',
            execute_callback=lambda handle: self.grasp(handle, place_only=True),
            goal_callback=lambda request: self.accept_grasp(request, place_only=True),
            cancel_callback=self.cancel_grasp, callback_group=self.group)
        self.create_timer(0.05, self.tick)

    def lease(self, owner, acquire):
        if not self.lease_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('controller task lease service unavailable')
        request = WebTaskLease.Request(owner=owner, acquire=acquire)
        result = TeachRosClient._wait_future(self.lease_client.call_async(request), 3.0)
        if not result or not result.success:
            raise RuntimeError(result.message if result else 'task lease returned no result')

    def make_robot(self):
        values = load_config(self.get_parameter('teaching_config').value)
        config, runtime = build_teach_config(values.get('teaching', {}))
        settings = values.get('robot', {})
        client = TeachRosClient(namespace=self.namespace + "/web_task", joint_state_topic=f'/{self.namespace}/joint_states',
            timeout_s=settings.get('ros_timeout_s',8.0), joint_state_max_age_s=settings.get('joint_state_max_age_s',0.5),
            safe_home_timeout_s=runtime['safe_home_timeout_s'])
        return RosTeachRobot(client, config)

    def teach_command(self, request, response):
        try:
            if request.command == 'release_object':
                if not self.runner or not self.runner.held:
                    raise RuntimeError('no held object')
                self.session.reserve('grasp')
                def release():
                    self.session.checkpoint()
                    self.runner.release()
                    self.lease('grasp', False)
                    self.session.owner = ''
                self.session.run(release, 'RELEASE_OBJECT')
            else:
                self.session.command(request.command, request.name, request.supervised)
            response.success = True
            response.message = self.session.message
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        response.status_json = json.dumps(self.status(), allow_nan=False)
        return response

    def status(self):
        status = self.session.status()
        status['held_object'] = bool(self.runner and self.runner.held)
        status['grasp_geometry_confirmed'] = bool(self.get_parameter('grasp_geometry_confirmed').value)
        status['grasp_backend'] = self.backend
        # ArmStatus is latched and published on state changes, not a heartbeat.
        # Fresh joints plus the lease service establish driver availability.
        status['driver_ready'] = (time.monotonic()-self.joints_received < 0.5
                                  and self.arm_received > 0
                                  and self.lease_client.service_is_ready())
        status['moveit_ready'] = all(client.service_is_ready() for client in self.moveit_clients)
        status['arm_enabled'] = self.arm_enabled
        conditions = (
            (status['driver_ready'], 'WAITING_DRIVER'),
            (status['moveit_ready'], 'WAITING_MOVEIT'),
            (status['arm_enabled'], 'NOT_ENABLED'),
            (status['motion_authorized'], 'MOTION_NOT_AUTHORIZED'),
            (self.backend == 'yolo-graspnet' or status['grasp_geometry_confirmed'], 'GEOMETRY_NOT_CONFIRMED'),
            (bool(self.detections.latest) and time.monotonic()-self.detections.received < 1.0, 'WAITING_TARGET'),
        )
        status['grasp_readiness'] = next((reason for ready, reason in conditions if not ready), 'READY')
        status['grasp_ready'] = status['grasp_readiness'] == 'READY'
        return status

    def joints_callback(self, msg):
        expected = {f'joint{index}' for index in range(1, 7)}
        if (expected.issubset(msg.name) and len(msg.name) == len(msg.position)
                and all(np.isfinite(value) for value in msg.position)):
            self.joints_received = time.monotonic()

    def arm_callback(self, msg):
        self.arm_received = time.monotonic()
        self.arm_enabled = bool(msg.enabled and msg.control_loop_active)

    def cancel(self, _request, response):
        try:
            self.session.cancel()
            deadline = time.monotonic() + 8.0
            while (self.session.worker and self.session.worker.is_alive()) or self.grasp_lock.locked():
                if time.monotonic() > deadline:
                    raise TimeoutError('task cancellation result not received; hold unconfirmed')
                time.sleep(0.01)
            if 'hold unconfirmed' in self.session.message:
                raise RuntimeError(self.session.message)
            response.success, response.message = True, self.session.message
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def cancel_grasp(self, _handle):
        self.session.cancelled.set()
        return CancelResponse.ACCEPT

    def detection_callback(self, msg):
        try:
            with self.detection_lock:
                self.detections.update(json.loads(msg.data), self.get_clock().now().nanoseconds*1e-9)
        except Exception as exc:
            self.get_logger().warning(str(exc))
            with self.detection_lock:
                self.detections.latest = []
                self.detections.sequence += 1

    def accept_grasp(self, request, *, place_only=False):
        if not self.grasp_lock.acquire(blocking=False):
            return GoalResponse.REJECT
        try:
            readiness = self.status()['grasp_readiness']
            if readiness != 'READY' and not (place_only and readiness == 'WAITING_TARGET'):
                raise RuntimeError(readiness)
            if self.backend == 'fixed' and not self.get_parameter('grasp_geometry_confirmed').value:
                raise RuntimeError('grasp geometry is not confirmed')
            if place_only:
                if self.backend != 'yolo-graspnet' or not self.runner or not self.runner.held:
                    raise RuntimeError('no held visual grasp object')
                if not self.runner.lift_completed:
                    raise RuntimeError('complete the visual lift before placement')
                if request.detection_id:
                    raise RuntimeError('placement does not accept a detection target')
            else:
                if self.runner and self.runner.held:
                    raise RuntimeError('an object is already held')
                with self.detection_lock:
                    target = self.detections.get(request.detection_id)
                    if target.get('backend', 'fixed') != self.backend:
                        raise RuntimeError('detection and execution backend mismatch')
            self.session.reserve('grasp')
            return GoalResponse.ACCEPT
        except Exception as exc:
            self.get_logger().warning(str(exc))
            self.grasp_lock.release()
            return GoalResponse.REJECT

    def transform(self, target, source):
        from utils.transforms import quat_to_mat4
        tf = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time()).transform
        p,q = tf.translation, tf.rotation
        return quat_to_mat4(p.x,p.y,p.z,q.x,q.y,q.z,q.w)

    def verify_target(self, target, checkpoint):
        deadline = time.monotonic() + 2.0
        with self.detection_lock:
            seen = self.detections.sequence
        stable = 0
        reason = 'fresh calibrated target unavailable; recapture and retry'
        while time.monotonic() < deadline:
            checkpoint()
            with self.detection_lock:
                sequence = self.detections.sequence
                if sequence != seen:
                    if sequence != seen + 1:
                        stable = 0  # Skipped observations cannot confirm continuity.
                    seen = sequence
                    try:
                        self.detections.verify(target)
                    except RuntimeError as exc:
                        stable, reason = 0, str(exc)
                    else:
                        stable += 1
                        if stable == 3:
                            return
            time.sleep(0.02)
        raise RuntimeError(reason)

    def make_runner(self):
        root = Path(self.get_parameter('perception_root').value).expanduser().resolve()
        sys.path.insert(0,str(root))
        from utils.camera_utils import load_config as load_perception
        from zekeep_shadow.web_grasp import WebGraspRunner
        config = Path(self.get_parameter('perception_config').value)
        cfg = load_perception(config if config.is_absolute() else root/config)
        if cfg['robot'].get('namespace','zekeep').strip('/') != self.namespace:
            raise ValueError('perception/controller namespace mismatch')
        if self.backend == 'yolo-graspnet':
            from zekeep_shadow.web_pipeline_grasp import RosPipelineGraspRunner
            return RosPipelineGraspRunner(root, cfg, self.transform,
                speed=self.get_parameter('grasp_max_velocity_rad_s').value)
        return WebGraspRunner(root,cfg,self.transform, rpy=self.get_parameter('grasp_rpy_rad').value,
            object_size=self.get_parameter('object_size_m').value,
            surface_offset=self.get_parameter('grasp_surface_offset_m').value,
            clearance=self.get_parameter('grasp_clearance_m').value,
            speed=self.get_parameter('grasp_max_velocity_rad_s').value)

    def grasp(self, handle, *, pick_only=False, place_only=False):
        result = WebGrasp.Result()
        try:
            if not place_only:
                with self.detection_lock:
                    target = self.detections.get(handle.request.detection_id)
            self.session.checkpoint()
            if self.runner is None:
                self.runner = self.make_runner()
            self.session.cancel_external = self.runner.cancel
            def check():
                if handle.is_cancel_requested:
                    self.session.cancelled.set()
                self.session.checkpoint()
            def verify():
                self.verify_target(target, check)
            def feedback(phase):
                self.session.phase = phase
                handle.publish_feedback(WebGrasp.Feedback(phase=phase))
            if place_only:
                self.runner.place(check, feedback, self.session.cancelled)
            elif pick_only and self.backend == 'yolo-graspnet':
                self.runner.execute(target, check, feedback, self.session.cancelled, verify, place_after=False)
            else:
                self.runner.execute(target, check, feedback, self.session.cancelled, verify)
            check()
            result.success, result.message = True, getattr(self.runner, 'completed_message', 'grasp and lift completed; object held')
        except Exception as exc:
            message = str(exc)
            try:
                if self.runner:
                    self.runner.cancel()
            except Exception as hold_error:
                message += '; hold unconfirmed: ' + str(hold_error)
            if self.runner and not getattr(self.runner, 'started', True):
                self.runner.client.close()
                self.runner = None
            result.success, result.message = False, message
        finally:
            self.session.cancel_external = None
            # A failed hold retains ownership, preventing a new task from masking the fault.
            if 'hold unconfirmed' not in result.message:
                try:
                    self.lease('grasp', False)
                    self.session.owner = ''
                except Exception as exc:
                    result.success = False
                    result.message += '; task ownership retained: ' + str(exc)
            self.session.phase = 'COMPLETED' if result.success else 'FAILED'
            self.session.message = result.message
            self.grasp_lock.release()
        if result.success:
            handle.succeed()
        elif handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        return result

    def tick(self):
        try:
            self.session.watchdog()
            self.session.sample()
        except Exception as exc:
            self.session.message = 'hold unconfirmed: ' + str(exc)
        if time.monotonic() - self.last_status_at >= 0.5:
            self.last_status_at = time.monotonic()
            if self.session.owner and (self.lease_renewal is None or self.lease_renewal.done()):
                self.lease_renewal = self.lease_client.call_async(
                    WebTaskLease.Request(owner=self.session.owner, renew=True))
                generation = self.session.generation
                self.lease_renewal.add_done_callback(lambda future: self.renewal_result(future, generation))
            self.status_pub.publish(String(data=json.dumps(self.status(), allow_nan=False)))

    def renewal_result(self, future, generation):
        if generation != self.session.generation or not self.session.owner:
            return
        try:
            result = future.result()
            if not result or not result.success:
                self.session.cancelled.set()
                self.session.message = result.message if result else 'task lease renewal failed'
        except Exception as exc:
            self.session.cancelled.set()
            self.session.message = str(exc)


def main(args=None):
    import signal
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(_signal, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = WebTasks()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown requests hold only. It never performs automatic home/disable.
        try:
            node.session.cancel()
            if node.session.worker:
                node.session.worker.join(timeout=5.0)
        finally:
            executor.shutdown()
            node.tf_listener.unregister()
            node.destroy_node()
            rclpy.try_shutdown()
