"""Visual grasp integration checks with no camera, ROS node, or motor connection."""
import json
import hashlib
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from pathlib import Path

import numpy as np
import yaml

from zekeep_shadow.perception_pipeline import PerceptionPipeline
from zekeep_shadow.web_pipeline_grasp import RosPipelineGraspRunner
from zekeep_shadow.web_vision import WebVision
from zekeep_teach.web_session import DetectionStore
from zekeep_teach.web_tasks import WebTasks


def make_runner(holding=True):
    runner = RosPipelineGraspRunner.__new__(RosPipelineGraspRunner)
    runner.root = Path(__file__).resolve().parents[3] / 'src/zekeep_grasp'
    runner.cfg = {'robot': {'moveit': dict(worktable_width_m=0.9, worktable_length_m=1.2,
        worktable_mount_inset_m=0.062, worktable_thickness_m=0.02)},
        'safety': {'table_z_m': 0.0}, 'place': dict(enabled=True, mode='fixed_joint',
        joint_target_rad=[0.1]*6, pre_grasp_wait_s=0, post_grasp_wait_s=0,
        pre_release_wait_s=0, post_release_wait_s=0), 'official_sdk': {'ready_joints': [0]*6},
        'camera': {'serial': 'test-camera'}}
    runner.calibration_identity = hashlib.sha256(json.dumps({'test': True}, sort_keys=True).encode()).hexdigest()
    runner.end_frame, runner.grasp_frame = 'gripper_base', 'grasp_tcp'
    runner.held, runner.started, runner.speed = False, True, 0.1
    runner.lift_completed = False
    runner.placement_class_name = ''
    import sys
    sys.path.insert(0, str(runner.root))
    plan = dict(version=1, backend='yolo-graspnet', detection_id='b'*32,
        calibration_identity=runner.calibration_identity, end_frame=runner.end_frame,
        joints=[0]*6, joint_solutions=[[0]*6]*3,
        end_poses=[np.eye(4).tolist()]*3, payload_size_m=[0.03]*3)
    plan['end_poses'] = [np.eye(4).tolist() for _ in range(3)]
    plan['end_poses'][0][2][3] = 0.10
    plan['end_poses'][1][2][3] = 0.02
    plan['end_poses'][2][2][3] = 0.10
    runner.perception = Mock()
    runner.perception.wait_for_service.return_value = True
    runner.client = Mock()
    runner.client.is_stationary.return_value = True
    runner.client._wait_future.return_value = SimpleNamespace(success=True, status_json=json.dumps(plan))
    runner.client.latest_sample.return_value = SimpleNamespace(positions=np.zeros(6))
    runner.client.solve_end_pose_ik.return_value = (np.zeros(6), 1)
    runner.client.grasp.return_value = holding
    return runner, plan


class VisualExecutionTest(unittest.TestCase):
    def test_transport_speed_increases_without_speeding_up_cartesian_grasp(self):
        from builtin_interfaces.msg import Duration
        from moveit_msgs.msg import RobotTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        from drivers.robot.ros_robot_client import RosRobotClient
        runner, _ = make_runner()
        runner.speed = 0.3
        runner.joint_plan(np.zeros(6), np.ones(6), 'transfer')
        transfer_speed = runner.client.time_parameterize_trajectory.call_args.kwargs['max_joint_velocity_rad_s']
        self.assertAlmostEqual(transfer_speed, 0.2)
        runner.cartesian_plan(np.zeros(6), np.eye(4), 'descent')
        grasp_speed = runner.client.plan_cartesian_end_path.call_args.kwargs['max_joint_velocity_rad_s']
        self.assertAlmostEqual(grasp_speed, 0.1 / 1.5)
        client = RosRobotClient.__new__(RosRobotClient)
        client._Duration = Duration
        for speed, expected_duration in ((grasp_speed, 15.0), (transfer_speed, 5.0)):
            trajectory = RobotTrajectory()
            trajectory.joint_trajectory.joint_names = [f'joint{i}' for i in range(1, 7)]
            point = JointTrajectoryPoint(positions=[1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            trajectory.joint_trajectory.points = [point]
            client.time_parameterize_trajectory(trajectory, start_positions=np.zeros(6),
                max_joint_velocity_rad_s=speed)
            duration = point.time_from_start.sec + point.time_from_start.nanosec*1e-9
            self.assertAlmostEqual(duration, expected_duration)
            self.assertAlmostEqual(1.5 / duration, speed * 1.5)
            self.assertEqual(list(point.velocities), [0.0]*6)

    def test_visual_grasp_uses_frozen_capture_despite_later_detection_loss_or_noise(self):
        for change in ('missing', 'position_noise'):
            with self.subTest(change=change):
                runner, _ = make_runner()
                target = dict(id='b'*32, class_name='red block', color='red block',
                    position=[0.25, 0.0, 0.03], calibration_identity=runner.calibration_identity)
                store = DetectionStore()
                store.received = store.clock()
                store.latest = [target]
                def opening():
                    store.received = store.clock()
                    store.latest = ([] if change == 'missing'
                        else [dict(target, position=[0.27, 0.0, 0.03])])
                runner.client.open_gripper.side_effect = opening
                verify = Mock(side_effect=lambda: store.verify(target))
                runner.execute(target, lambda: None, lambda phase: None,
                    threading.Event(), verify, place_after=False)
                verify.assert_not_called()
                runner.perception.call_async.assert_called_once()
                runner.client.grasp.assert_called_once()
                self.assertTrue(runner.held)
                self.assertTrue(runner.lift_completed)

    def test_robot_movement_still_blocks_frozen_plan_execution(self):
        for stage in ('planning', 'opening', 'opening_motion'):
            with self.subTest(stage=stage):
                runner, _ = make_runner()
                sample = runner.client.latest_sample.return_value
                def move(*args, **kwargs):
                    sample.positions = np.ones(6)*0.004
                    return Mock()
                if stage == 'planning':
                    runner.client.plan_cartesian_end_path.side_effect = move
                elif stage == 'opening':
                    runner.client.open_gripper.side_effect = move
                else:
                    runner.client.is_stationary.side_effect = [True, False]
                with self.assertRaisesRegex(RuntimeError, 'robot moved'):
                    runner.execute(dict(id='b'*32, calibration_identity=runner.calibration_identity),
                        lambda: None, lambda phase: None, threading.Event(), Mock())
                runner.client.execute_planned_joint_trajectory.assert_not_called()
                runner.client.grasp.assert_not_called()

    def test_perception_requires_fresh_stationary_capture_only_at_planning_entry(self):
        for age, displacement, velocity in ((0.2, 0, 0), (1.1, 0, 0), (0.2, .004, 0), (0.2, 0, .03)):
            with self.subTest(age=age, displacement=displacement, velocity=velocity):
                capture = dict(joints=np.zeros(6), calibration_identity='a'*64)
                node = SimpleNamespace(plan_lock=threading.Lock(), capture_lock=threading.Lock(),
                    pipeline=Mock(), captures={'b'*32: (0.0, capture, object())},
                    cache=Mock(), perception_error='')
                node.cache.latest.return_value = SimpleNamespace(
                    positions=np.ones(6)*displacement, velocities=np.ones(6)*velocity)
                def plan(*args):
                    node.captures.clear()  # Live frames may expire while GraspNet plans.
                    return {'version': 1}
                node.pipeline.plan.side_effect = plan
                response = SimpleNamespace(success=False, message='', status_json='')
                with patch('zekeep_shadow.web_vision.time.monotonic', return_value=age):
                    WebVision.plan(node, SimpleNamespace(command='plan', name='b'*32), response)
                if age <= 1 and displacement == 0 and velocity == 0:
                    self.assertTrue(response.success)
                    body = json.loads(response.status_json)
                    self.assertEqual(body['detection_id'], 'b'*32)
                    self.assertEqual(body['calibration_identity'], 'a'*64)
                else:
                    self.assertFalse(response.success)
                    self.assertRegex(response.message, 'stale|robot moved')
                    node.pipeline.plan.assert_not_called()

    def test_class_placement_routes_immediate_and_separate_tasks(self):
        # Synthetic taught points keep the test independent of physical calibration.
        cfg = {'place': {
            'class_targets': {'red block': 'red_place', 'blue block': 'blue_place', 'green block': 'green_place'},
            'named_joint_targets_rad': {'red_place': [.2]*6, 'blue_place': [.3]*6, 'green_place': [.4]*6},
            'joint_target_rad': [.1]*6,
        }}
        for class_name in ('red block', 'blue block', 'green block', 'gray block', 'red pen'):
            for separate in (False, True):
                with self.subTest(class_name=class_name, separate=separate):
                    runner, _ = make_runner()
                    runner.cfg['place'].update({key: cfg['place'][key]
                        for key in ('class_targets', 'named_joint_targets_rad', 'joint_target_rad')})
                    detection = dict(id='b'*32, class_name=class_name,
                        calibration_identity=runner.calibration_identity)
                    event = threading.Event()
                    with patch('utils.calibration_identity.validate_calibration_identity'), \
                            patch('utils.calibration_identity.calibration_identity', return_value={'test': True}):
                        runner.execute(detection, lambda: None, lambda phase: None,
                            event, lambda: None, place_after=not separate)
                        if separate:
                            detection['class_name'] = 'different object'
                            runner.place(lambda: None, lambda phase: None, event)
                    placement = runner.client.plan_joint_goal.call_args_list[1]
                    name = cfg['place']['class_targets'].get(class_name)
                    expected = (cfg['place']['named_joint_targets_rad'][name] if name
                        else cfg['place']['joint_target_rad'])
                    np.testing.assert_array_equal(placement.args[1], expected)
                    self.assertIn(name or 'fixed_joint', placement.kwargs['label'])
                    self.assertFalse(runner.held)

    def test_bad_class_target_rejected_before_grasp(self):
        runner, _ = make_runner()
        runner.cfg['place']['class_targets'] = {'red block': 'missing_place'}
        with self.assertRaisesRegex(ValueError, 'unknown placement target'):
            runner.execute(dict(id='b'*32, class_name='red block',
                calibration_identity=runner.calibration_identity), lambda: None,
                lambda phase: None, threading.Event(), lambda: None)
        runner.client.grasp.assert_not_called()
        runner.client.open_gripper.assert_not_called()

    def test_moveit_start_state_preserves_attached_payload(self):
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/zekeep_grasp'))
        from drivers.robot.ros_robot_client import RosRobotClient
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState
        client = RosRobotClient.__new__(RosRobotClient)
        client._RobotState, client._JointState = RobotState, JointState
        for opening in (None, .015):
            state = client._robot_state(np.zeros(6), gripper_joint_position_m=opening)
            self.assertTrue(state.is_diff)
            self.assertEqual(state.joint_state.name[:6], [f'joint{i}' for i in range(1, 7)])
            self.assertEqual(len(state.joint_state.position), 6 if opening is None else 8)
            if opening is not None:
                self.assertEqual(list(state.joint_state.position[-2:]), [.015, .015])

    def test_original_geometry_center_rejects_local_depth_noise_and_real_displacement(self):
        import sys
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/zekeep_grasp'))
        from utils.cube_perception import assess_cube_geometry
        from utils.shape_orientation import masked_depth_points_base
        from zekeep_shadow.tracking import estimate_camera_point
        pipeline = PerceptionPipeline.__new__(PerceptionPipeline)
        pipeline.cfg = {'safety': {'table_z_m': 0.0}}
        pipeline.route = SimpleNamespace(assess_cube_geometry=assess_cube_geometry,
            masked_depth_points_base=masked_depth_points_base,
            graspnet_utils=SimpleNamespace(build_target_sample_mask=lambda shape, detection, **kw: detection.mask))
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[20:80, 20:80] = 1
        detection = SimpleNamespace(mask=mask)
        K = np.array([[1000., 0, 50], [0, 1000., 50], [0, 0, 1]])
        transform = np.diag([1., -1., -1., 1.])
        transform[2, 3] = .5
        depth = np.full(mask.shape, 470, dtype=np.uint16)
        original = pipeline.measure_position(detection, depth, K, transform)
        local_before = estimate_camera_point(detection, depth, K)
        depth[44:56, 44:56] = 490
        noisy = pipeline.measure_position(detection, depth, K, transform)
        local_after = estimate_camera_point(detection, depth, K)
        self.assertGreater(np.linalg.norm(local_after-local_before), .019)
        np.testing.assert_allclose(noisy, original, atol=1e-6)
        self.assertAlmostEqual(original[2], .015)
        moved_transform = transform.copy()
        moved_transform[0, 3] = .02
        moved = pipeline.measure_position(detection, depth, K, moved_transform)
        store = DetectionStore()
        selected = dict(position=original.tolist(), color='red block', calibration_identity='a'*64,
                        position_reference='measured_object_center')
        store.received = store.clock()
        store.latest = [dict(selected, position=noisy.tolist())]
        store.verify(selected)
        store.latest = [dict(selected, position=moved.tolist())]
        with self.assertRaisesRegex(RuntimeError, 'target changed'):
            store.verify(selected)
        self.assertIsNone(pipeline.measure_position(detection, np.zeros_like(depth), K, transform))

    def test_empty_grasp_and_cancel_never_lift_or_place(self):
        for cancelled in (False, True):
            with self.subTest(cancelled=cancelled):
                runner, _ = make_runner(holding=False)
                event = threading.Event()
                def feedback(phase):
                    if cancelled and phase == 'DESCENDING':
                        event.set()
                def checkpoint():
                    if event.is_set():
                        raise RuntimeError('cancelled')
                with self.assertRaisesRegex(RuntimeError, 'cancelled|no object contact'):
                    runner.execute(dict(id='b'*32, calibration_identity=runner.calibration_identity),
                        checkpoint, feedback, event, lambda: None)
                runner.client.apply_attached_collision_box.assert_not_called()
                self.assertFalse(runner.held)
                if cancelled:
                    runner.client.grasp.assert_not_called()

    def test_placement_and_ready_use_ros_planned_trajectories(self):
        runner, _ = make_runner()
        phases = []
        with patch('utils.calibration_identity.validate_calibration_identity'), \
                patch('utils.calibration_identity.calibration_identity', return_value={'test': True}):
            runner.execute(dict(id='b'*32, calibration_identity=runner.calibration_identity),
                lambda: None, phases.append, threading.Event(), lambda: None)
        self.assertEqual(runner.client.execute_planned_joint_trajectory.call_count, 5)
        self.assertEqual(runner.client.plan_joint_goal.call_count, 3)
        self.assertIn('PLACING', phases)
        self.assertIn('RETURNING_READY', phases)
        self.assertFalse(runner.held)
        self.assertIn('object released', runner.completed_message)
        self.assertEqual([call.args[1] for call in runner.client.set_payload_table_contact.call_args_list], [True, False])

    def test_failed_lift_restores_payload_table_collision_check(self):
        runner, _ = make_runner()
        runner.client.execute_planned_joint_trajectory.side_effect = [None, None, RuntimeError('lift failed')]
        with self.assertRaisesRegex(RuntimeError, 'lift failed'):
            runner.execute(dict(id='b'*32, calibration_identity=runner.calibration_identity),
                lambda: None, lambda phase: None, threading.Event(), lambda: None)
        self.assertTrue(runner.held)
        self.assertFalse(runner.lift_completed)
        self.assertEqual([call.args[1] for call in runner.client.set_payload_table_contact.call_args_list], [True, False])

    def test_payload_table_contact_preserves_other_collision_pairs(self):
        import sys
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/zekeep_grasp'))
        from drivers.robot.ros_robot_client import RosRobotClient
        from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix, PlanningScene
        from moveit_msgs.srv import GetPlanningScene
        client = RosRobotClient.__new__(RosRobotClient)
        matrix = AllowedCollisionMatrix(entry_names=['arm', 'wrist'],
            entry_values=[AllowedCollisionEntry(enabled=[False, True]), AllowedCollisionEntry(enabled=[True, False])])
        client._timeout_s, client._GetPlanningScene, client._PlanningScene = 1, GetPlanningScene, PlanningScene
        client._get_planning_scene_client = Mock()
        client._wait_future = Mock(return_value=SimpleNamespace(scene=SimpleNamespace(allowed_collision_matrix=matrix)))
        client._apply_planning_scene_diff = Mock()
        client.set_payload_table_contact('payload', True)
        payload, table = matrix.entry_names.index('payload'), matrix.entry_names.index('fixed_worktable')
        self.assertTrue(matrix.entry_values[0].enabled[1])
        self.assertFalse(matrix.entry_values[0].enabled[table])
        self.assertTrue(matrix.entry_values[payload].enabled[table])
        client.set_payload_table_contact('payload', False)
        self.assertFalse(matrix.entry_values[payload].enabled[table])

    def test_wrong_frame_is_rejected_before_any_motion(self):
        runner, plan = make_runner()
        plan['end_frame'] = 'official_tcp'
        runner.client._wait_future.return_value.status_json = json.dumps(plan)
        with self.assertRaisesRegex(ValueError, 'frame mismatch'):
            runner.execute(dict(id='b'*32, calibration_identity=runner.calibration_identity),
                lambda: None, lambda phase: None, threading.Event(), lambda: None)
        runner.client.open_gripper.assert_not_called()
        runner.client.execute_planned_joint_trajectory.assert_not_called()

    def test_pick_only_holds_payload_until_separate_placement(self):
        runner, _ = make_runner()
        event = threading.Event()
        phases = []
        runner.execute(dict(id='b'*32, calibration_identity=runner.calibration_identity),
            lambda: None, phases.append, event, lambda: None, place_after=False)
        self.assertTrue(runner.held)
        self.assertTrue(runner.lift_completed)
        self.assertIn('object held', runner.completed_message)
        self.assertEqual(runner.client.execute_planned_joint_trajectory.call_count, 3)
        self.assertNotIn('PLACING', phases)
        self.assertNotIn('RETURNING_READY', phases)
        runner.client.remove_attached_collision_box.assert_not_called()
        runner.client.reset_mock()
        with patch('utils.calibration_identity.validate_calibration_identity'), \
                patch('utils.calibration_identity.calibration_identity', return_value={'test': True}):
            runner.place(lambda: None, phases.append, event)
        self.assertEqual(runner.client.execute_planned_joint_trajectory.call_count, 2)
        runner.client.grasp.assert_not_called()
        runner.perception.call_async.assert_called_once()
        runner.client.open_gripper.assert_called_once()
        runner.client.remove_attached_collision_box.assert_called_once()
        self.assertFalse(runner.held)

    def test_separate_placement_rejects_missing_payload_calibration_change_and_cancel(self):
        import sys
        runner, _ = make_runner()
        sys.path.insert(0, str(runner.root))
        with self.assertRaisesRegex(RuntimeError, 'no held object'):
            runner.place(lambda: None, lambda phase: None, threading.Event())
        runner.held = True
        with self.assertRaisesRegex(RuntimeError, 'complete the visual lift'):
            runner.place(lambda: None, lambda phase: None, threading.Event())
        runner.lift_completed = True
        with patch('utils.calibration_identity.validate_calibration_identity'), \
                patch('utils.calibration_identity.calibration_identity', return_value={'test': False}):
            with self.assertRaisesRegex(RuntimeError, 'calibration differs'):
                runner.place(lambda: None, lambda phase: None, threading.Event())
        with patch('utils.calibration_identity.validate_calibration_identity'), \
                patch('utils.calibration_identity.calibration_identity', return_value={'test': True}):
            runner.client.wait_until_stationary.side_effect = TimeoutError('not stationary')
            with self.assertRaisesRegex(TimeoutError, 'not stationary'):
                runner.place(lambda: None, lambda phase: None, threading.Event())
            runner.client.plan_joint_goal.assert_not_called()
            runner.client.wait_until_stationary.side_effect = None
            event = threading.Event()
            def feedback(phase):
                if phase == 'PLACING':
                    event.set()
            def checkpoint():
                if event.is_set():
                    raise RuntimeError('cancelled')
            with self.assertRaisesRegex(RuntimeError, 'cancelled'):
                runner.place(checkpoint, feedback, event)
        self.assertTrue(runner.held)
        runner.client.open_gripper.assert_not_called()
        runner.client.remove_attached_collision_box.assert_not_called()

    def test_placement_acceptance_needs_payload_and_authorization_without_detection(self):
        from rclpy.action import GoalResponse
        node = SimpleNamespace(backend='yolo-graspnet', runner=SimpleNamespace(held=True, lift_completed=True),
            grasp_lock=threading.Lock(), status=lambda: {'grasp_readiness': 'WAITING_TARGET'},
            session=Mock(), get_logger=lambda: Mock())
        request = SimpleNamespace(detection_id='')
        self.assertEqual(WebTasks.accept_grasp(node, request, place_only=True), GoalResponse.ACCEPT)
        node.session.reserve.assert_called_once_with('grasp')
        node.grasp_lock.release()
        for readiness, held in (('MOTION_NOT_AUTHORIZED', True), ('WAITING_DRIVER', True),
                                ('NOT_ENABLED', True), ('WAITING_TARGET', False)):
            node.status = lambda: {'grasp_readiness': readiness}
            node.runner.held = held
            self.assertEqual(WebTasks.accept_grasp(node, request, place_only=True), GoalResponse.REJECT)
            self.assertFalse(node.grasp_lock.locked())

    def test_web_tasks_dispatches_pick_and_place_through_same_cancelable_session(self):
        for options in ({}, {'pick_only': True}, {'place_only': True}):
            with self.subTest(options=options):
                runner = SimpleNamespace(execute=Mock(), place=Mock(), cancel=Mock(), completed_message='done')
                session = SimpleNamespace(checkpoint=Mock(), cancelled=threading.Event(), owner='grasp')
                node = SimpleNamespace(runner=runner, session=session, backend='yolo-graspnet',
                    detections=Mock(), detection_lock=threading.RLock(), grasp_lock=threading.Lock(),
                    lease=Mock(), verify_target=Mock())
                node.grasp_lock.acquire()
                handle = Mock(is_cancel_requested=False)
                result = WebTasks.grasp(node, handle, **options)
                self.assertTrue(result.success)
                handle.succeed.assert_called_once()
                node.lease.assert_called_once_with('grasp', False)
                self.assertFalse(node.grasp_lock.locked())
                if options.get('place_only'):
                    node.detections.get.assert_not_called()
                    runner.execute.assert_not_called()
                    runner.place.assert_called_once()
                else:
                    runner.place.assert_not_called()
                    self.assertEqual(runner.execute.call_args.kwargs,
                                     {'place_after': False} if options.get('pick_only') else {})

    def test_missing_driver_keeps_status_available_and_grasp_disabled(self):
        node = SimpleNamespace(backend='yolo-graspnet', joints_received=0, arm_received=0,
            arm_enabled=False, lease_client=Mock(), moveit_clients=[Mock()], runner=None,
            get_parameter=lambda name: SimpleNamespace(value=False),
            session=SimpleNamespace(status=lambda: dict(motion_authorized=False)),
            detections=SimpleNamespace(latest=[], received=0))
        status = WebTasks.status(node)
        self.assertEqual(status['grasp_readiness'], 'WAITING_DRIVER')
        self.assertFalse(status['grasp_ready'])

    def test_latched_arm_status_does_not_expire_while_joint_feedback_is_live(self):
        now = time.monotonic()
        node = SimpleNamespace(backend='yolo-graspnet', joints_received=now,
            arm_received=now-10, arm_enabled=True, lease_client=Mock(),
            moveit_clients=[Mock()], runner=None,
            get_parameter=lambda name: SimpleNamespace(value=False),
            session=SimpleNamespace(status=lambda: dict(motion_authorized=True)),
            detections=SimpleNamespace(latest=[{}], received=now))
        status = WebTasks.status(node)
        self.assertEqual(status['grasp_readiness'], 'READY')
        self.assertTrue(status['grasp_ready'])
        node.joints_received = now-1
        self.assertEqual(WebTasks.status(node)['grasp_readiness'], 'WAITING_DRIVER')
        node.joints_received = now
        WebTasks.arm_callback(node, SimpleNamespace(enabled=False, control_loop_active=False))
        self.assertEqual(WebTasks.status(node)['grasp_readiness'], 'NOT_ENABLED')
        node.arm_received = 0
        self.assertEqual(WebTasks.status(node)['grasp_readiness'], 'WAITING_DRIVER')

    def test_camera_frames_are_published_without_robot_feedback(self):
        from pathlib import Path
        from builtin_interfaces.msg import Time
        import sys
        root = Path(__file__).resolve().parents[3] / 'src/zekeep_grasp'
        sys.path.insert(0, str(root))
        color = np.zeros((6, 6, 3), dtype=np.uint8)
        detector = SimpleNamespace(mask=np.ones((6, 6), dtype=np.uint8), bbox_xyxy=(0, 0, 5, 5),
                                   conf=0.9, class_name='blue block')
        camera = Mock(K=np.eye(3), D=np.zeros(5))
        camera.get_frame.return_value = (color, np.ones((6, 6), dtype=np.uint16)*500)
        cache = Mock()
        cache.latest.side_effect = RuntimeError('no driver joint feedback')
        node = SimpleNamespace(root=root, cfg={'camera': {'type': 'orbbec_gemini2'}}, camera=camera,
            backend='yolo-graspnet', cache=cache, get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())),
            pipeline=SimpleNamespace(detect=lambda frame: [detector],
                route=SimpleNamespace(draw_detections_overlay=lambda *args: color.copy())),
            perception_error='', capture_lock=threading.Lock(), captures={}, sequence=0,
            images=Mock(), detections=Mock())
        WebVision.frame(node)
        image = node.images.publish.call_args.args[0]
        self.assertEqual((image.height, image.width, len(image.data)), (6, 6, 108))
        body = json.loads(node.detections.publish.call_args.args[0].data)
        self.assertTrue(body['calibration_status'].startswith('PREVIEW_ONLY'))
        self.assertEqual(body['detections'], [])
        self.assertEqual(body['visual_detections'], [dict(class_name='blue block',
            confidence=.9, bbox=[0, 0, 5, 5])])
        node.cache = Mock()
        node.cache.latest.return_value = SimpleNamespace(positions=np.zeros(6), velocities=np.zeros(6))
        node.camera_transform = lambda stamp: np.eye(4)
        node.calibration_identity = 'a'*64
        node.pipeline.measure_position = lambda *args: None
        with patch('zekeep_shadow.web_vision.np.load') as load:
            load.return_value.__enter__.return_value = dict(K=np.eye(3), D=np.zeros(5), resolution=np.array([6, 6]))
            WebVision.frame(node)
        valid = json.loads(node.detections.publish.call_args.args[0].data)
        self.assertEqual(valid['calibration_status'], 'CALIBRATED_STATIONARY')
        self.assertEqual(valid['visual_detections'], body['visual_detections'])
        self.assertEqual(valid['detections'], [])
        self.assertEqual(node.captures, {})
        node.camera.last_frame_failure_reason = 'missing depth frame'
        node.camera.get_frame.side_effect = [(None, None), (color, np.ones((6, 6), dtype=np.uint16)*500)]
        with patch('zekeep_shadow.web_vision.np.load') as load, patch('zekeep_shadow.web_vision.time.sleep') as sleep:
            load.return_value.__enter__.return_value = dict(K=np.eye(3), D=np.zeros(5), resolution=np.array([6, 6]))
            WebVision.frame(node)
        sleep.assert_called_once_with(.05)
        self.assertEqual(json.loads(node.detections.publish.call_args.args[0].data)['calibration_status'],
                         'CALIBRATED_STATIONARY')
        node.camera.get_frame.reset_mock()
        node.camera.get_frame.side_effect = None
        node.camera.get_frame.return_value = (None, None)
        with patch('zekeep_shadow.web_vision.time.sleep'), self.assertRaisesRegex(
                RuntimeError, '4 attempts: missing depth frame'):
            WebVision.frame(node)
        self.assertEqual(node.camera.get_frame.call_count, 4)

    def test_yolo_targets_preserve_class_identity_and_expire(self):
        clock = [0.0]
        store = DetectionStore(clock=lambda: clock[0])
        body = dict(version=1, frame_id='base_link', source='hardware-rgbd',
            calibration_status='CALIBRATED_STATIONARY', calibration_identity='a'*64,
            stamp=dict(sec=1, nanosec=0), detections=[dict(id='b'*32, backend='yolo-graspnet',
                color='blue block', class_name='blue block', position=[0.25, 0.0, 0.03])])
        store.update(body, 1.0)
        selected = store.get('b'*32)
        self.assertEqual(selected['class_name'], 'blue block')
        body['detections'][0].update(color='red block', class_name='red block')
        store.update(body, 1.0)
        with self.assertRaisesRegex(RuntimeError, 'target changed'):
            store.verify(selected)
        clock[0] = 1.1
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            store.get('b'*32)

    def test_tf_wait_keeps_capture_time_and_stationarity_checks(self):
        from pathlib import Path
        from builtin_interfaces.msg import Time
        import sys
        root = Path(__file__).resolve().parents[3] / 'src/zekeep_grasp'
        sys.path.insert(0, str(root))
        for delay, displacement, drops in ((.15, 0, 0), (.26, 0, 0), (.15, .004, 0), (.15, 0, 1), (.2, 0, 2)):
            with self.subTest(delay=delay, displacement=displacement, drops=drops):
                clock = [0.0]
                color = np.zeros((6, 6, 3), dtype=np.uint8)
                camera = Mock(K=np.eye(3), D=np.zeros(5))
                camera.get_frame.return_value = (color, np.ones((6, 6), dtype=np.uint16)*500)
                camera.get_frame.side_effect = [(None, None)]*drops + [camera.get_frame.return_value]
                cache = Mock()
                cache.latest.side_effect = [
                    SimpleNamespace(positions=np.zeros(6), velocities=np.zeros(6)),
                    SimpleNamespace(positions=np.ones(6)*displacement, velocities=np.zeros(6))]
                def transform(stamp):
                    clock[0] += delay
                    return np.eye(4)
                def sleep(seconds):
                    clock[0] += seconds
                node = SimpleNamespace(root=root, cfg={'camera': {'type': 'orbbec_gemini2'}}, camera=camera,
                    backend='yolo-graspnet', cache=cache,
                    get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())),
                    pipeline=SimpleNamespace(detect=lambda frame: [],
                        route=SimpleNamespace(draw_detections_overlay=lambda *args: color.copy())),
                    camera_transform=transform, calibration_identity='a'*64,
                    perception_error='', capture_lock=threading.Lock(), captures={}, sequence=0,
                    images=Mock(), detections=Mock())
                saved = dict(K=np.eye(3), D=np.zeros(5), resolution=np.array([6, 6]))
                with patch('zekeep_shadow.web_vision.time.monotonic', side_effect=lambda: clock[0]), \
                        patch('zekeep_shadow.web_vision.time.sleep', side_effect=sleep), \
                        patch('zekeep_shadow.web_vision.np.load') as load:
                    load.return_value.__enter__.return_value = saved
                    WebVision.frame(node)
                body = json.loads(node.detections.publish.call_args.args[0].data)
                self.assertEqual(body['calibration_status'] == 'CALIBRATED_STATIONARY',
                                 delay + drops*.05 <= .25 and displacement <= .003)

    def test_target_confirmation_requires_three_new_matching_frames(self):
        def confirm(observations, *, cancel_at=None):
            clock = [0.0]
            store = DetectionStore(clock=lambda: clock[0])
            body = dict(version=1, frame_id='base_link', source='hardware-rgbd',
                calibration_status='CALIBRATED_STATIONARY', calibration_identity='a'*64,
                stamp=dict(sec=1, nanosec=0), detections=[dict(id='b'*32,
                    backend='yolo-graspnet', color='red block', class_name='red block',
                    position=[0.25, 0.0, 0.03])])
            store.update(body, 1.0)
            selected = store.get('b'*32)
            node = SimpleNamespace(detections=store, detection_lock=threading.RLock())
            pending = list(observations)
            def sleep(duration):
                clock[0] += duration
                while pending and pending[0][0] <= clock[0]:
                    _, offset = pending.pop(0)
                    if offset is None:
                        store.update(dict(calibration_status='PREVIEW_ONLY'), 1.0)
                    else:
                        target = dict(body['detections'][0], position=[0.25+offset, 0.0, 0.03])
                        store.update(dict(body, detections=[target]), 1.0)
            def checkpoint():
                if cancel_at is not None and clock[0] >= cancel_at:
                    raise RuntimeError('cancelled')
            with patch('zekeep_teach.web_tasks.time.monotonic', side_effect=lambda: clock[0]), \
                    patch('zekeep_teach.web_tasks.time.sleep', side_effect=sleep):
                WebTasks.verify_target(node, selected, checkpoint)
            return clock[0]

        elapsed = confirm([(.1, None), (.3, .002), (.5, -.003), (.7, .004)])
        self.assertGreaterEqual(elapsed, .7)
        self.assertLess(elapsed, 1.0)
        # Cached frames cannot count repeatedly; real movement stays rejected.
        for observations in ([ (.1, .002) ], [(.1, .02), (.3, .02), (.5, .02)], []):
            with self.subTest(observations=observations), self.assertRaisesRegex(RuntimeError, 'unavailable|target changed'):
                confirm(observations)
        # Skipped observations reset continuity even when the last frame matches.
        elapsed = confirm([(.1, 0), (.3, None), (.3, 0), (.5, 0), (.7, 0)])
        self.assertGreaterEqual(elapsed, .7)
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            confirm([], cancel_at=.2)

    def test_original_tcp_is_converted_to_ros_end_frame_without_sdk_connection(self):
        pipeline = PerceptionPipeline.__new__(PerceptionPipeline)
        pipeline.cfg = {'robot': {'end_effector_frame': 'gripper_base', 'grasp_reference_frame': 'grasp_tcp'},
                        'safety': {'table_z_m': 0.0}}
        pipeline.yolo, pipeline.yolo_opts, pipeline.net = None, {}, None
        pipeline.lock, pipeline.collision_model = threading.Lock(), None
        def frame(z):
            pose = np.eye(4)
            pose[2, 3] = z
            return pose
        pipeline.kinematics = SimpleNamespace(fk=lambda q: frame(0.10),
            fk_frame=lambda q, name: frame(0.02 if name == 'gripper_base' else 0.09))
        result = SimpleNamespace(best=object(), geometry=SimpleNamespace(
            length_m=0.03, width_m=0.03, height_m=0.03),
            target_points_base=np.array([[-0.015, -0.015, 0.03], [0.015, 0.015, 0.03], [0, 0, 0.03]]))
        def infer(*args):
            self.assertIsNone(args[5])
            self.assertIsNone(args[6])
            self.assertEqual(args[8].class_name, 'red block')
            worker = threading.Thread(target=lambda: pipeline.detect(None), daemon=True)
            worker.start()
            worker.join(timeout=.5)
            self.assertFalse(worker.is_alive(), 'GraspNet blocked live YOLO detection')
            return result
        pipeline.route = SimpleNamespace(infer_configured_frame=infer,
            detect_objects=lambda *args: (None, []),
            _simple_cfg=lambda cfg: (0.005, 0.08, 0.08, 0, 0.01, 0.01),
            _anchor_grasps_to_measured_center=lambda *args, **kwargs: True,
            _choose_candidate=lambda *args, **kwargs: (None, [[0, 0, 0.3, 0, 0, 0]]*3, [np.zeros(6)]*3),
            pose6d_to_mat4=lambda *args: frame(args[2]),
            transform_points=lambda points, pose: points @ pose[:3, :3].T + pose[:3, 3])
        plan = pipeline.plan(dict(joints=np.zeros(6), camera_to_base=np.eye(4),
            color=None, depth=None, K=np.eye(3)), SimpleNamespace(class_name='red block'))
        self.assertAlmostEqual(plan['end_poses'][0][2][3], 0.22)
        self.assertTrue(all(value > 0 for value in plan['payload_size_m']))


if __name__ == '__main__':
    unittest.main()
