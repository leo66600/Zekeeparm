"""Read-only RGB-D publisher for the web console; never opens a motor port."""
from __future__ import annotations

import json
import hashlib
import ctypes
from pathlib import Path
import sys
import threading
import time
import uuid
from types import SimpleNamespace

# Select the visual environment's C++ ABI before ROS/OpenCV load a system copy.
runtime_library = Path(sys.prefix) / 'lib/libstdc++.so.6'
if runtime_library.is_file():
    ctypes.CDLL(str(runtime_library), mode=ctypes.RTLD_GLOBAL)

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from zekeep_msgs.srv import WebTeachCommand

from .tracking import detect_red_blocks, estimate_camera_point
from .paths import default_perception_root


class WebVision(Node):
    def __init__(self):
        super().__init__('zekeep_web_vision')
        self.declare_parameter('perception_root', default_perception_root())
        self.declare_parameter('config', 'config/default.yaml')
        self.declare_parameter('namespace', 'zekeep')
        self.declare_parameter('grasp_backend', 'fixed')
        self.root = Path(self.get_parameter('perception_root').value).expanduser().resolve()
        sys.path.insert(0, str(self.root))
        from drivers.camera import make_camera
        from utils.camera_utils import load_config
        from drivers.robot.ros_robot_client import JointStateCache
        config = Path(self.get_parameter('config').value)
        self.cfg = load_config(config if config.is_absolute() else self.root / config)
        self.namespace = str(self.get_parameter('namespace').value).strip('/')
        self.cache = JointStateCache()
        self.create_subscription(JointState, f'/{self.namespace}/joint_states', self.joints, qos_profile_sensor_data)
        self.images = self.create_publisher(Image, f'/{self.namespace}/web/camera/image_raw', 1)
        self.detections = self.create_publisher(String, f'/{self.namespace}/web/vision/detections', 1)
        self.backend = str(self.get_parameter('grasp_backend').value)
        if self.backend not in ('fixed', 'yolo-graspnet'):
            raise ValueError('unknown grasp backend')
        self.captures = {}
        self.capture_lock = threading.Lock()
        self.plan_lock = threading.Lock()
        self.pipeline = None
        self.perception_error = ''
        if self.backend == 'yolo-graspnet':
            self.detections.publish(String(data=json.dumps(dict(version=1, detections=[],
                calibration_status='LOADING_YOLO_GRASPNET', backend=self.backend))))
            try:
                from .perception_pipeline import PerceptionPipeline
                self.pipeline = PerceptionPipeline(self.root, self.cfg)
            except Exception as exc:
                self.perception_error = 'YOLO/GraspNet unavailable: ' + str(exc)
                self.get_logger().error(self.perception_error)
            self.create_service(WebTeachCommand, f'/{self.namespace}/web/vision/plan', self.plan,
                                callback_group=ReentrantCallbackGroup())
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.camera = make_camera(self.cfg)
        self.camera.open()
        self.sequence = 0

    def plan(self, request, response):
        if not self.plan_lock.acquire(blocking=False):
            response.message = 'perception planning already in progress'
            return response
        try:
            if request.command != 'plan' or self.pipeline is None:
                raise RuntimeError(self.perception_error or 'invalid perception command')
            with self.capture_lock:
                captured_at, capture, detection = self.captures[request.name]
            if time.monotonic() - captured_at > 1.0:
                raise RuntimeError('RGB-D target snapshot is stale')
            sample = self.cache.latest(max_age_s=0.5)
            if np.max(np.abs(sample.positions - capture['joints'])) > 0.003 or np.max(np.abs(sample.velocities)) > 0.02:
                raise RuntimeError('robot moved since the selected RGB-D capture')
            plan = self.pipeline.plan(capture, detection)
            plan.update(detection_id=request.name, calibration_identity=capture['calibration_identity'])
            response.status_json = json.dumps(plan, allow_nan=False)
            response.success, response.message = True, 'original perception filters passed'
        except Exception as exc:
            response.message = str(exc)
        finally:
            self.plan_lock.release()
        return response

    def joints(self, msg):
        self.cache.update(msg.name, msg.position, msg.velocity, received_monotonic=time.monotonic())

    def camera_transform(self, stamp):
        from utils.camera_utils import load_hand_eye, compose_cam_to_base_transform
        from utils.calibration_identity import validate_calibration_identity, calibration_identity
        from utils.transforms import quat_to_mat4
        validate_calibration_identity(self.root, self.cfg, self.camera.serial)
        self.calibration_identity = hashlib.sha256(json.dumps(calibration_identity(self.root, self.cfg, self.camera.serial), sort_keys=True).encode()).hexdigest()
        reference = self.cfg['robot']['end_effector_frame']
        hand_eye, mode = load_hand_eye(self.root, self.cfg['camera']['type'], expected_reference_frame=reference)
        if hand_eye is None or mode != 'eye_in_hand':
            raise RuntimeError('valid eye-in-hand calibration is required')
        tf = self.tf_buffer.lookup_transform('base_link', reference, rclpy.time.Time.from_msg(stamp), timeout=Duration(seconds=0.2)).transform
        p, q = tf.translation, tf.rotation
        return compose_cam_to_base_transform(quat_to_mat4(p.x, p.y, p.z, q.x, q.y, q.z, q.w), hand_eye, self.cfg)

    def frame(self):
        started = time.monotonic()
        before = None
        try:
            before = self.cache.latest(max_age_s=0.5)
        except RuntimeError:
            pass  # Preview remains available without robot feedback.
        settings = self.cfg.get('safety') or {}
        retries = int(settings.get('capture_frame_retries', 3))
        delay = float(settings.get('capture_frame_retry_interval_s', 0.05))
        if retries < 0 or not np.isfinite(delay) or delay < 0:
            raise ValueError('invalid RGB-D frame retry configuration')
        for attempt in range(retries + 1):
            color, depth = self.camera.get_frame()
            if color is not None and depth is not None:
                break
            if attempt < retries and delay > 0:
                time.sleep(delay)
        stamp = self.get_clock().now().to_msg()
        if color is None or depth is None:
            reason = getattr(self.camera, 'last_frame_failure_reason', '')
            raise RuntimeError(f'no aligned RGB-D frame after {retries + 1} attempts: {reason}')
        display = color.copy()
        if self.backend == 'yolo-graspnet':
            detections = self.pipeline.detect(color) if self.pipeline is not None else []
            if self.pipeline is not None:
                display = self.pipeline.route.draw_detections_overlay(color, detections, None, None)
            raw = [SimpleNamespace(mask=item.mask, bbox=item.bbox_xyxy, confidence=item.conf,
                                   class_name=item.class_name, detection=item) for item in detections]
        else:
            raw = detect_red_blocks(color)
        status = 'PREVIEW_ONLY'
        targets = []
        try:
            if self.perception_error:
                raise RuntimeError(self.perception_error)
            from utils.camera_utils import validate_runtime_intrinsics, validate_runtime_distortion
            calibration = self.root / 'config/calibration' / self.cfg['camera']['type'] / 'intrinsics.npz'
            with np.load(calibration, allow_pickle=False) as saved:
                K = saved['camera_matrix'] if 'camera_matrix' in saved else saved['K']
                D = saved['dist_coeffs'] if 'dist_coeffs' in saved else saved['D']
                # Use the saved calibration dimensions; do not silently assume a profile match.
                resolution = tuple(map(int, saved['resolution']))
                validate_runtime_intrinsics(self.camera.K, K, runtime_resolution=(color.shape[1], color.shape[0]), calibrated_resolution=resolution)
                validate_runtime_distortion(self.camera.D, D)
            if before is None:
                raise RuntimeError('capture requires stationary, fresh joint feedback')
            transform = self.camera_transform(stamp)
            after = self.cache.latest(max_age_s=0.5)
            if time.monotonic() - started > 0.25 or np.max(np.abs(before.positions-after.positions)) > 0.003 or max(np.max(np.abs(before.velocities)), np.max(np.abs(after.velocities))) > 0.02:
                raise RuntimeError('capture requires stationary, fresh joint feedback')
            status = 'CALIBRATED_STATIONARY'
            capture = dict(color=color.copy(), depth=depth.copy(), K=self.camera.K.copy(),
                           camera_to_base=transform.copy(), joints=after.positions.copy(),
                           calibration_identity=self.calibration_identity)
            for item in raw[:10]:
                if self.backend == 'yolo-graspnet':
                    base = self.pipeline.measure_position(item.detection, depth, self.camera.K, transform)
                    if base is None:
                        continue
                else:
                    camera_point = estimate_camera_point(item, depth, self.camera.K)
                    if camera_point is None:
                        continue
                    K = self.camera.K
                    uv = np.array([[[camera_point[0] / camera_point[2] * K[0,0] + K[0,2], camera_point[1] / camera_point[2] * K[1,1] + K[1,2]]]])
                    normalized = cv2.undistortPoints(uv, K, self.camera.D).reshape(2)
                    camera_point[:2] = normalized * camera_point[2]
                    base = transform[:3, :3] @ camera_point + transform[:3, 3]
                target = dict(id=uuid.uuid4().hex, color=getattr(item, 'class_name', 'red'),
                              position=base.tolist(), confidence=item.confidence, bbox=list(item.bbox))
                if self.backend == 'yolo-graspnet':
                    target.update(backend=self.backend, class_name=item.class_name,
                                  position_reference='measured_object_center')
                    with self.capture_lock:
                        self.captures[target['id']] = (started, capture, item.detection)
                targets.append(target)
        except Exception as exc:
            status = 'PREVIEW_ONLY: ' + str(exc)
        for item in raw:
            if self.backend == 'fixed':
                x1,y1,x2,y2 = item.bbox
                cv2.rectangle(display, (x1,y1), (x2,y2), (0,180,255), 2)
        with self.capture_lock:
            self.captures = {key: value for key, value in self.captures.items() if time.monotonic()-value[0] <= 1.0}
        scale = min(1.0, 640 / display.shape[1])
        display = cv2.resize(display, (int(display.shape[1]*scale), int(display.shape[0]*scale)))
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.height, msg.width = display.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = display.tobytes()
        self.images.publish(msg)
        self.sequence += 1
        body = dict(version=1, sequence=self.sequence, frame_id='base_link', stamp=dict(sec=stamp.sec, nanosec=stamp.nanosec),
                    calibration_status=status, calibration_identity=getattr(self, 'calibration_identity', None), source='hardware-rgbd', capture_timing='receipt-stationary', detections=targets, backend=self.backend)
        # Text queries can describe 2D observations without creating grasp targets.
        body['visual_detections'] = [dict(class_name=getattr(item, 'class_name', 'red'),
            confidence=item.confidence, bbox=list(item.bbox)) for item in raw]
        self.detections.publish(String(data=json.dumps(body, allow_nan=False)))

    def close(self):
        self.camera.close()
        self.tf_listener.unregister()


def main(args=None):
    rclpy.init(args=args)
    node = WebVision()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    import threading
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        while rclpy.ok():
            try:
                node.frame()
            except Exception as exc:
                node.get_logger().warning(str(exc), throttle_duration_sec=5.0)
                node.detections.publish(String(data=json.dumps(dict(version=1, detections=[], calibration_status='NO_FRAME: ' + str(exc)))))
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        executor.shutdown()
        thread.join(timeout=2)
        node.destroy_node()
        rclpy.try_shutdown()
