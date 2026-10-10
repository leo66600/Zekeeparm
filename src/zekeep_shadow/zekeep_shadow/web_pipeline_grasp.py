"""Execute the original visual grasp candidates through ROS and MoveIt only."""
from __future__ import annotations

import hashlib
import json
import sys

import numpy as np
from zekeep_msgs.srv import WebTeachCommand

from .web_grasp import WebGraspRunner


class RosPipelineGraspRunner(WebGraspRunner):
    def __init__(self, root, cfg, transform, *, speed):
        sys.path.insert(0, str(root))
        from utils.calibration_identity import calibration_identity, validate_calibration_identity
        validate_calibration_identity(root, cfg, cfg['camera']['serial'])
        self.calibration_identity = hashlib.sha256(json.dumps(
            calibration_identity(root, cfg, cfg['camera']['serial']), sort_keys=True).encode()).hexdigest()
        self.cfg, self.transform, self.speed = cfg, transform, float(speed)
        self.root = root
        if not np.isfinite(self.speed) or not 0 < self.speed <= 0.3:
            raise ValueError('invalid supervised joint speed')
        place = cfg.get('place') or {}
        if place.get('enabled', False) and place.get('mode') != 'fixed_joint':
            raise ValueError('ROS visual placement currently requires place.mode=fixed_joint')
        self._initialize_client(cfg, transform)
        self.perception = self.client._node.create_client(
            WebTeachCommand, f"/{cfg['robot'].get('namespace', 'zekeep')}/web/vision/plan")
        self.completed_message = ''
        self.lift_completed = False
        self.placement_class_name = ''

    def joint_plan(self, start, target, label, *, gripper_open=False):
        settings = self.cfg['robot']['moveit']
        plan = self.client.plan_joint_goal(
            start, target, pipeline_id=settings.get('planning_pipeline_id', 'ompl'),
            planner_id=settings.get('planner_id', 'RRTConnectkConfigDefault'),
            num_planning_attempts=int(settings.get('planning_attempts', 5)),
            allowed_planning_time_s=float(settings.get('planning_time_s', 8.0)),
            max_velocity_scaling_factor=float(settings.get('max_velocity_scaling_factor', 0.05)),
            max_acceleration_scaling_factor=float(settings.get('max_acceleration_scaling_factor', 0.05)),
            joint_tolerance_rad=float(settings.get('joint_goal_tolerance_rad', 0.005)),
            label=label, gripper_open=gripper_open)
        self.client.time_parameterize_trajectory(
            plan, start_positions=start, max_joint_velocity_rad_s=self.speed / 1.5)
        return plan

    def cartesian_plan(self, start, pose, label, *, gripper_open=False):
        settings = self.cfg['robot']['moveit']
        return self.client.plan_cartesian_end_path(
            start, [pose], max_step_m=float(settings.get('cartesian_max_step_m', 0.003)),
            revolute_jump_threshold_rad=float(settings.get('revolute_jump_threshold_rad', 0.25)),
            max_joint_velocity_rad_s=min(self.speed / 1.5, 0.1 / 1.5, float(settings.get('approach_max_joint_velocity_rad_s', 0.2))),
            gripper_open=gripper_open, label=label)

    def execute_trajectory(self, trajectory, phase, checkpoint, feedback, cancel_event):
        checkpoint()
        feedback(phase)
        self.client.execute_planned_joint_trajectory(trajectory, cancel_event=cancel_event)
        checkpoint()

    def wait(self, name, checkpoint, cancel_event):
        duration = float((self.cfg.get('place') or {}).get(name, 1.0))
        if not np.isfinite(duration) or duration < 0:
            raise ValueError('invalid grasp/place wait duration')
        cancel_event.wait(duration)
        checkpoint()

    def execute(self, detection, checkpoint, feedback, cancel_event, verify_target, *, place_after=True):
        """Execute the selected calibrated RGB-D snapshot without live reacquisition."""
        from utils.fixed_placement import resolve_joint_placement_target
        if detection.get('calibration_identity') != self.calibration_identity:
            raise RuntimeError('detection calibration differs from task configuration')
        place = self.cfg.get('place') or {}
        class_name = detection.get('class_name', '')
        if place.get('enabled', False):
            resolve_joint_placement_target(place, class_name)
        self.start()
        checkpoint()
        if self.held or not self.client.is_stationary(0.02):
            raise RuntimeError('release the held object and keep the robot stationary')
        self.lift_completed = False
        if not self.perception.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('original YOLO/GraspNet perception backend unavailable')
        feedback('GRASPNET')
        response = self.client._wait_future(self.perception.call_async(
            WebTeachCommand.Request(command='plan', name=detection['id'])),
            float(self.cfg.get('grasp_pipeline', {}).get('planning_budget_s', 25.0)) + 10.0,
            cancel_event=cancel_event)
        checkpoint()
        if response is None or not response.success:
            raise RuntimeError(response.message if response else 'no perception result')
        plan = json.loads(response.status_json)
        if (plan.get('version') != 1 or plan.get('backend') != 'yolo-graspnet'
                or plan.get('detection_id') != detection['id']
                or plan.get('calibration_identity') != self.calibration_identity
                or plan.get('end_frame') != self.end_frame):
            raise ValueError('perception result identity or frame mismatch')
        start = np.asarray(plan['joints'], dtype=float)
        poses = np.asarray(plan['end_poses'], dtype=float)
        seeds = np.asarray(plan['joint_solutions'], dtype=float)
        dimensions = np.asarray(plan['payload_size_m'], dtype=float)
        if (start.shape != (6,) or seeds.shape != (3, 6) or poses.shape != (3, 4, 4)
                or dimensions.shape != (3,) or not all(np.all(np.isfinite(item)) for item in (start, poses, seeds, dimensions))
                or np.any(dimensions <= 0) or np.any(dimensions > 0.5)
                or not np.allclose(poses[:, 3, :], [0, 0, 0, 1])):
            raise ValueError('invalid visual grasp plan')
        rotations = poses[:, :3, :3]
        if (not np.allclose(rotations.transpose(0, 2, 1) @ rotations, np.eye(3), atol=1e-5)
                or not np.allclose(np.linalg.det(rotations), 1.0, atol=1e-5)):
            raise ValueError('invalid visual grasp rotations')
        if poses[2, 2, 3] <= poses[1, 2, 3] or not np.allclose(rotations[2], rotations[1], atol=1e-5):
            raise ValueError('payload must lift upward with unchanged orientation')
        if np.max(np.abs(start - self.client.latest_sample().positions)) > 0.003:
            raise RuntimeError('robot moved since visual capture')
        settings = self.cfg['robot']['moveit']
        self.client.apply_worktable_top(float(self.cfg['safety']['table_z_m']),
            width_m=settings['worktable_width_m'], length_m=settings['worktable_length_m'],
            mount_inset_m=settings['worktable_mount_inset_m'], thickness_m=settings['worktable_thickness_m'])
        feedback('PLANNING')
        q_above, code = self.client.solve_end_pose_ik(poses[0], seeds[0], timeout_s=1.0, attempts=5)
        if q_above is None:
            raise RuntimeError(f'visual pregrasp IK rejected by MoveIt: {code}')
        approach = self.joint_plan(start, q_above, 'visual pregrasp', gripper_open=True)
        descent = self.cartesian_plan(q_above, poses[1], 'visual descent', gripper_open=True)
        checkpoint()
        if np.max(np.abs(start - self.client.latest_sample().positions)) > 0.003:
            raise RuntimeError('robot moved during visual planning')

        def execute(trajectory, phase):
            self.execute_trajectory(trajectory, phase, checkpoint, feedback, cancel_event)

        def wait(name):
            self.wait(name, checkpoint, cancel_event)

        feedback('OPENING')
        self.client.open_gripper()
        checkpoint()
        # shortcut: object stays fixed after capture; use visual servoing for moving targets.
        if not self.client.is_stationary(0.02) or np.max(np.abs(start - self.client.latest_sample().positions)) > 0.003:
            raise RuntimeError('robot moved before visual execution')
        execute(approach, 'PREGRASP')
        wait('pre_grasp_wait_s')
        execute(descent, 'DESCENDING')
        feedback('GRASPING')
        if not self.client.grasp(cancel_event=cancel_event):
            raise RuntimeError('gripper reported no object contact; lift rejected')
        self.held = True
        self.placement_class_name = class_name
        wait('post_grasp_wait_s')
        self.client.apply_attached_collision_box('web_grasp_object', link_name=self.grasp_frame,
            dimensions_m=dimensions.tolist(), touch_links=(self.grasp_frame, 'gripper_base', 'left_link', 'right_link'))
        feedback('PLANNING_LIFT')
        # A picked object starts in contact with the table. Only that pair is
        # allowed during the upward lift; arm/table and all other checks remain.
        self.client.set_payload_table_contact('web_grasp_object', True)
        try:
            lift = self.cartesian_plan(self.client.latest_sample().positions, poses[2], 'visual lift')
            execute(lift, 'LIFTING')
        finally:
            self.client.set_payload_table_contact('web_grasp_object', False)
        self.lift_completed = True
        self.completed_message = 'visual grasp and lift completed; object held'
        if place_after and place.get('enabled', False):
            self.place(checkpoint, feedback, cancel_event)
            self.completed_message = 'visual grasp and placement completed; object released'
        feedback('COMPLETED')

    def place(self, checkpoint, feedback, cancel_event):
        from utils.calibration_identity import calibration_identity, validate_calibration_identity
        from utils.fixed_placement import resolve_joint_placement_target
        checkpoint()
        place = self.cfg.get('place') or {}
        if not self.held:
            raise RuntimeError('no held object')
        if not self.lift_completed:
            raise RuntimeError('complete the visual lift before placement')
        if not place.get('enabled', False) or place.get('mode') != 'fixed_joint':
            raise RuntimeError('placement requires enabled fixed_joint configuration')
        validate_calibration_identity(self.root, self.cfg, self.cfg['camera']['serial'])
        identity = hashlib.sha256(json.dumps(calibration_identity(
            self.root, self.cfg, self.cfg['camera']['serial']), sort_keys=True).encode()).hexdigest()
        if identity != self.calibration_identity:
            raise RuntimeError('placement calibration differs from the grasp configuration')
        self.client.wait_until_stationary(velocity_tolerance_rad_s=0.02,
            required_samples=3, sample_interval_s=0.02, timeout_s=2.0, cancel_event=cancel_event)
        place_name, target = resolve_joint_placement_target(place, self.placement_class_name)
        feedback('PLANNING_PLACE')
        transfer = self.joint_plan(self.client.latest_sample().positions, target, f'visual placement {place_name}')
        self.execute_trajectory(transfer, 'PLACING', checkpoint, feedback, cancel_event)
        self.wait('pre_release_wait_s', checkpoint, cancel_event)
        feedback('RELEASING')
        self.release()
        self.wait('post_release_wait_s', checkpoint, cancel_event)
        if place.get('return_ready_after_release', True):
            feedback('PLANNING_READY')
            ready = self.joint_plan(self.client.latest_sample().positions,
                np.asarray(self.cfg['official_sdk']['ready_joints'], dtype=float), 'visual return ready', gripper_open=True)
            self.execute_trajectory(ready, 'RETURNING_READY', checkpoint, feedback, cancel_event)
        self.completed_message = f'visual placement at {place_name} completed; object released'
