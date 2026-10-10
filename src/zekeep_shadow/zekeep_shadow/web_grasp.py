"""One ROS/MoveIt grasp task using configured geometry and confirmed calibration."""
from __future__ import annotations

import sys
import hashlib
import json
import numpy as np


class WebGraspRunner:
    def __init__(self, root, cfg, transform, *, rpy, object_size, surface_offset, clearance, speed):
        sys.path.insert(0, str(root))
        from utils.transforms import pose6d_to_mat4
        from utils.calibration_identity import calibration_identity, validate_calibration_identity
        validate_calibration_identity(root, cfg, cfg['camera']['serial'])
        self.calibration_identity = hashlib.sha256(json.dumps(
            calibration_identity(root, cfg, cfg['camera']['serial']), sort_keys=True).encode()).hexdigest()
        self.cfg, self.transform = cfg, transform
        self.pose_matrix = pose6d_to_mat4
        self.rpy = np.asarray(rpy, dtype=float)
        self.object_size = np.asarray(object_size, dtype=float)
        self.surface_offset, self.clearance, self.speed = surface_offset, clearance, speed
        if self.rpy.shape != (3,) or self.object_size.shape != (3,) or not np.all(np.isfinite(self.rpy)) or not np.all(np.isfinite(self.object_size)) or np.any(self.object_size <= 0) or np.any(self.object_size > 0.07):
            raise ValueError('finite orientation and measured object size <= 70 mm are required')
        if not 0.03 <= clearance <= 0.15 or not 0 < speed <= 0.3 or not np.isfinite(surface_offset) or abs(surface_offset) > 0.03:
            raise ValueError('invalid grasp clearance, offset or conservative speed')
        self._initialize_client(cfg, transform)

    def _initialize_client(self, cfg, transform):
        from drivers.robot.ros_robot_client import RosRobotClient
        robot_cfg = cfg['robot']
        self.end_frame = robot_cfg['end_effector_frame']
        self.grasp_frame = robot_cfg['grasp_reference_frame']
        self.client = RosRobotClient(namespace=robot_cfg.get('namespace', 'zekeep') + '/web_task',
            joint_state_topic=robot_cfg.get('joint_state_topic', '/zekeep/joint_states'),
            timeout_s=robot_cfg.get('ros_timeout_s', 8.0), joint_state_max_age_s=robot_cfg.get('joint_state_max_age_s', 0.5),
            T_link6_to_end=transform('link6', self.end_frame))
        self.started = False
        self.held = False

    def start(self):
        if not self.started:
            self.client.start(require_gripper=True, require_moveit=True, require_direct_joint=True)
            self.started = True

    def cancel(self):
        self.client.cancel_active()
        self.client.stop_and_hold()

    def execute(self, detection, checkpoint, feedback, cancel_event, verify_target):
        if detection.get('calibration_identity') != self.calibration_identity:
            raise RuntimeError('detection calibration differs from task configuration')
        self.start()
        checkpoint()
        if self.held:
            raise RuntimeError('release the held object before another grasp')
        if not self.client.is_stationary(0.02):
            raise RuntimeError('robot must be stationary before planning')
        surface = np.asarray(detection['position'], dtype=float)
        if surface.shape != (3,) or not np.all(np.isfinite(surface)):
            raise ValueError('invalid detection position')
        workspace = self.cfg['robot']['workspace']
        for index, axis in enumerate(('x','y','z')):
            lo, hi = workspace[axis]
            if not lo <= surface[index] <= hi:
                raise ValueError('detection outside configured workspace')
        moveit = self.cfg['robot']['moveit']
        table_z = float(self.cfg['safety']['table_z_m'])
        self.client.apply_worktable_top(table_z, width_m=moveit['worktable_width_m'], length_m=moveit['worktable_length_m'],
            mount_inset_m=moveit['worktable_mount_inset_m'], thickness_m=moveit['worktable_thickness_m'])
        center = surface.copy()
        center[2] += self.surface_offset - self.object_size[2]/2
        if center[2] - self.object_size[2]/2 < table_z - 0.005:
            raise ValueError('target object geometry penetrates configured table')
        grasp = self.pose_matrix(*center, *self.rpy)
        above = grasp.copy(); above[2,3] += self.clearance
        if above[2,3] > workspace['z'][1] or grasp[2,3] < self.cfg['robot'].get('min_tcp_z_m', 0.005):
            raise ValueError('grasp or approach violates TCP height limits')
        tool = self.transform(self.grasp_frame, self.end_frame)
        above_end, grasp_end = above @ tool, grasp @ tool
        feedback('PLANNING')
        start = self.client.latest_sample().positions
        q_above, code = self.client.solve_end_pose_ik(above_end, start, timeout_s=1.0, attempts=5)
        if q_above is None:
            raise RuntimeError(f'pregrasp IK failed: {code}')
        approach = self.client.plan_joint_goal(start, q_above, pipeline_id='ompl', planner_id='RRTConnectkConfigDefault',
            num_planning_attempts=5, allowed_planning_time_s=3.0, max_velocity_scaling_factor=0.05,
            max_acceleration_scaling_factor=0.05, joint_tolerance_rad=0.005, label='web pregrasp', gripper_open=True)
        # Bound joint speed explicitly rather than relying only on model-dependent scaling.
        self.client.time_parameterize_trajectory(approach, start_positions=start, max_joint_velocity_rad_s=self.speed / 1.5)
        descent = self.client.plan_cartesian_end_path(q_above, [grasp_end], max_step_m=0.003,
            revolute_jump_threshold_rad=0.25, max_joint_velocity_rad_s=min(self.speed, 0.1) / 1.5, gripper_open=True, label='web descent')
        checkpoint()
        verify_target()
        feedback('OPENING'); self.client.open_gripper(); checkpoint()
        verify_target()
        feedback('PREGRASP'); self.client.execute_planned_joint_trajectory(approach, cancel_event=cancel_event); checkpoint()
        feedback('DESCENDING'); self.client.execute_planned_joint_trajectory(descent, cancel_event=cancel_event); checkpoint()
        feedback('GRASPING')
        if not self.client.grasp(cancel_event=cancel_event):
            raise RuntimeError('gripper reported no object contact; lift rejected')
        self.held = True
        checkpoint()
        feedback('PLANNING_LIFT')
        self.client.apply_attached_collision_box('web_grasp_object', link_name=self.grasp_frame,
            dimensions_m=self.object_size.tolist())
        lift = self.client.plan_cartesian_end_path(self.client.latest_sample().positions, [above_end], max_step_m=0.003,
            revolute_jump_threshold_rad=0.25, max_joint_velocity_rad_s=min(self.speed, 0.1) / 1.5, label='web lift')
        checkpoint()
        feedback('LIFTING'); self.client.execute_planned_joint_trajectory(lift, cancel_event=cancel_event); checkpoint()
        feedback('COMPLETED')

    def release(self):
        if not self.held:
            raise RuntimeError('no held object')
        self.client.open_gripper()
        self.client.remove_attached_collision_box('web_grasp_object', link_name=self.grasp_frame)
        self.held = False
