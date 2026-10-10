"""Reuse the validated CLI perception route; this module never connects motors."""
from __future__ import annotations

import importlib.util
from itertools import product
from pathlib import Path
import threading
from types import SimpleNamespace

import numpy as np


class PerceptionPipeline:
    def __init__(self, root, cfg):
        self.root, self.cfg = Path(root), cfg
        self.lock = threading.Lock()
        spec = importlib.util.spec_from_file_location(
            'zekeep_grasp_pipeline', self.root / 'scripts/main.py')
        self.route = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.route)
        self.yolo, self.yolo_opts = self.route.load_yolo(cfg, project_root=self.root)
        gp = cfg.get('graspnet') or {}
        checkpoint = self.route.resolve_checkpoint_path(
            str(gp.get('checkpoint', 'checkpoint-rs.tar')), project_root=self.root)
        self.net = self.route.build_net(checkpoint, num_view=int(gp.get('num_view', 300)))
        from drivers.robot.official_sdk_robot import OfficialKinematics
        model = Path(cfg['official_sdk']['urdf'])
        self.kinematics = OfficialKinematics(
            model if model.is_absolute() else self.root / model,
            multistart_retries=int(cfg.get('grasp_pipeline', {}).get('ik', {}).get('multistart_retries', 0)))
        model = Path(cfg['robot']['urdf_path'])
        self.collision_model = None
        if cfg.get('safety', {}).get('table_collision_enabled', True):
            self.collision_model = self.route.UrdfGripperCollisionModel(
                model if model.is_absolute() else (self.root / model).resolve(),
                open_width_m=float(cfg['robot'].get('gripper_max_width_m', 0.06517241379310346)), end_frame='grasp_tcp')

    def detect(self, color):
        with self.lock:
            return self.route.detect_objects(self.yolo, color, self.yolo_opts)[1]

    def measure_position(self, detection, depth, K, camera_to_base):
        mask = self.route.graspnet_utils.build_target_sample_mask(
            depth.shape, detection, margin_px=0, expand_ratio=1.0)
        settings = self.cfg.get('graspnet') or {}
        points = self.route.masked_depth_points_base(depth, mask, K, camera_to_base,
            min_depth_m=float(settings.get('min_depth', 0.05)),
            max_depth_m=float(settings.get('max_depth', 1.0)))
        geometry = self.route.assess_cube_geometry(points,
            table_z_m=float(self.cfg['safety']['table_z_m']), enforce_size_bounds=False)
        return geometry.center_base if geometry.valid else None

    def plan(self, capture, detection):
        cfg, route = self.cfg, self.route
        q = np.asarray(capture['joints'], dtype=float)
        transform = capture['camera_to_base']
        # The selected capture bypasses YOLO; GraspNet must not block live detection.
        result = route.infer_configured_frame(
            self.net, capture['color'], capture['depth'], capture['K'], cfg,
            None, None, detection.class_name, detection)
        min_z, pre, retreat, insertion, _, _ = route._simple_cfg(cfg)
        if result.best is None:
            raise RuntimeError('GraspNet: ' + result.status)
        if not route._anchor_grasps_to_measured_center(
                result, capture['depth'], capture['K'], transform, cfg, min_z=min_z):
            raise RuntimeError('no reliable measured grasp center')
        # Only FK/IK are reused from the SDK route. No OfficialSdkRobot exists here.
        robot = SimpleNamespace(kinematics=self.kinematics, current_joints=lambda: q.copy())
        candidate = route._choose_candidate(
            result, robot, transform, min_z=min_z, pre_offset=pre, retreat_offset=retreat,
            insertion_depth=insertion, config=cfg, collision_model=self.collision_model)
        if candidate is None:
            raise RuntimeError('no candidate passed the original pose, IK and collision filters')
        _, poses, solutions = candidate
        reference = self.kinematics.fk(q)
        end = self.kinematics.fk_frame(q, cfg['robot']['end_effector_frame'])
        grasp_frame = self.kinematics.fk_frame(q, cfg['robot']['grasp_reference_frame'])
        official_to_end = np.linalg.inv(reference) @ end
        end_poses = [route.pose6d_to_mat4(*pose) @ official_to_end for pose in poses]
        # Bound the visible target plus its table-to-top volume in the actual tool frame.
        points = result.target_points_base
        low, high = np.quantile(points, [0.01, 0.99], axis=0)
        low[2] = float(cfg['safety']['table_z_m'])
        high[2] = low[2] + result.geometry.height_m
        corners = np.array(list(product(*zip(low, high))))
        grasp_pose = end_poses[1] @ np.linalg.inv(end) @ grasp_frame
        local = route.transform_points(corners, np.linalg.inv(grasp_pose))
        padding = float(cfg.get('place', {}).get('object_padding_m', 0.005))
        if not np.isfinite(padding) or padding < 0:
            raise ValueError('invalid payload collision padding')
        dimensions = 2 * np.max(np.abs(local), axis=0) + 2 * padding
        if not np.all(np.isfinite(dimensions)) or np.any(dimensions <= 0):
            raise RuntimeError('invalid measured payload geometry')
        return dict(version=1, backend='yolo-graspnet', joints=q.tolist(),
                    end_frame=cfg['robot']['end_effector_frame'],
                    end_poses=[pose.tolist() for pose in end_poses],
                    joint_solutions=[np.asarray(item).tolist() for item in solutions[-3:]],
                    object_size_m=[result.geometry.length_m, result.geometry.width_m, result.geometry.height_m],
                    payload_size_m=dimensions.tolist())
