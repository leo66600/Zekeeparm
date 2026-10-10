"""Exercise query results and target matching with fake tools, never motors."""
import json
from types import SimpleNamespace
import unittest

from zekeep_llm.planner import Plan, Step, validate_arguments
from zekeep_llm.robot import RobotTools
from zekeep_llm.web_agent import AgentSession


class DetectionResultsTest(unittest.TestCase):
    def test_joint_motion_retimes_to_peak_speed_and_preserves_requested_duration(self):
        from zekeep_llm.motion import MAX_SPEED, joint_segments
        current = [0.0]*6
        target = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.assertEqual(MAX_SPEED, 0.3)
        for requested, expected in ((2, 5), (8, 8)):
            segments = joint_segments(current, {'positions_rad': target, 'duration_s': requested})
            self.assertEqual(segments, [(target, expected)])
            self.assertLessEqual(1.5 / expected, MAX_SPEED)
        with self.assertRaises(ValueError):
            joint_segments(current, {'positions_rad': [3.0, 0.0, 0.0, 0.0, 0.0, 0.0]})

    def test_aborted_action_preserves_backend_failure_reason(self):
        message = 'target changed during planning; recapture and retry'
        response = SimpleNamespace(status=6, result=SimpleNamespace(success=False, message=message))
        future = SimpleNamespace(done=lambda: True, result=lambda: response)
        handle = SimpleNamespace(accepted=True, get_result_async=lambda: future)
        sent = SimpleNamespace(done=lambda: True, result=lambda: handle)
        client = SimpleNamespace(wait_for_server=lambda **kw: True, send_goal_async=lambda goal: sent)
        robot = SimpleNamespace(cancel_event=None, _spin_until=lambda predicate, *args, **kw: None)
        with self.assertRaisesRegex(RuntimeError, message):
            RobotTools.run_action(robot, client, object(), 'pick_color', 1)

    def test_execution_returns_only_current_results_without_removing_history(self):
        planner = SimpleNamespace(plan=lambda text: Plan('查询检测', (Step('detect_blocks'),)))
        robot = SimpleNamespace(execute=lambda name, arguments: json.dumps({'detections': []}))
        session = AgentSession(planner, robot)
        for expected_history in (1, 2):
            plan = session.plan('桌上有什么')
            self.assertFalse(plan['requires_confirmation'])
            result = session.execute(plan['plan_id'], True)
            self.assertEqual(len(result['events']), expected_history)
            self.assertEqual(len(result['execution_events']), 1)
            self.assertEqual(result['execution_events'][0]['name'], 'detect_blocks')
        self.assertEqual(len(session.status()['events']), 2)

    def test_color_matching_accepts_yolo_names_and_rejects_ambiguous_targets(self):
        calls = []
        targets = []
        robot = SimpleNamespace(
            detect_blocks=lambda: {'detections': targets},
            run_action=lambda client, goal, label, timeout: calls.append(goal.detection_id) or SimpleNamespace(message='done'),
            _pick=None, _delegated_busy=False,
        )
        robot.run_web_task = lambda *args: RobotTools.run_web_task(robot, *args)
        for color in ('red', 'blue', 'green', 'yellow', 'purple'):
            validate_arguments('pick_color', {'color': color})
            targets[:] = [{'id': color, 'class_name': f'{color} block', 'color': f'{color} block'}]
            self.assertEqual(RobotTools.pick_color(robot, color), 'done')
            self.assertEqual(calls[-1], color)
            self.assertFalse(robot._delegated_busy)
        targets[:] = [{'id': 'legacy', 'color': 'red'}]
        RobotTools.pick_color(robot, 'red')
        self.assertEqual(calls[-1], 'legacy')
        count = len(calls)
        targets[:] = [{'id': 'tape', 'class_name': 'red tape', 'color': 'red tape'}]
        with self.assertRaisesRegex(RuntimeError, '没有可执行'):
            RobotTools.pick_color(robot, 'red')
        targets[:] = [{'id': 'one', 'color': 'red'}, {'id': 'two', 'class_name': 'red block'}]
        with self.assertRaisesRegex(RuntimeError, '多个同色'):
            RobotTools.pick_color(robot, 'red')
        self.assertEqual(len(calls), count)

    def test_pick_modes_and_placement_route_to_delegated_actions(self):
        calls = []
        robot = SimpleNamespace(detect_blocks=lambda: {'detections': [{'id': 'target', 'color': 'red'}]},
            _pick='full', _pick_only='pick', _place='place', _delegated_busy=False,
            cancel_event=None, _triggers={}, _require_ready_status=lambda: None,
            prepare=lambda tool: None,
            run_action=lambda client, goal, label, timeout: calls.append((client, goal.detection_id)) or SimpleNamespace(message='done'))
        robot.run_web_task = lambda *args: RobotTools.run_web_task(robot, *args)
        robot.pick_color = lambda *args, **kwargs: RobotTools.pick_color(robot, *args, **kwargs)
        self.assertEqual(RobotTools.execute(robot, 'pick_color', {'color': 'red'}), 'done')
        self.assertEqual(RobotTools.execute(robot, 'pick_color', {'color': 'red', 'place_after': False}), 'done')
        self.assertEqual(RobotTools.execute(robot, 'place_object'), 'done')
        self.assertEqual(calls, [('full', 'target'), ('pick', 'target'), ('place', '')])
        self.assertFalse(robot._delegated_busy)
        with self.assertRaises(ValueError):
            validate_arguments('pick_color', {'color': 'red', 'place_after': 'false'})
        with self.assertRaises(ValueError):
            validate_arguments('place_object', {'position_m': [0, 0, 0]})

    def test_place_plan_requires_confirmation_and_releases_ai_lease_before_delegation(self):
        calls = []
        planner = SimpleNamespace(plan=lambda text: Plan('放置', (Step('return_ready'), Step('place_object'))))
        robot = SimpleNamespace(execute=lambda name, arguments=None: calls.append(name) or 'done')
        session = AgentSession(planner, robot, motion_authorized=True, lease=lambda acquire: calls.append(acquire))
        plan = session.plan('放置')
        self.assertTrue(plan['requires_confirmation'])
        session.execute(plan['plan_id'], True)
        self.assertEqual(calls, [True, 'return_ready', False, 'place_object'])
        session.motion_authorized = False
        plan = session.plan('放置')
        with self.assertRaises(PermissionError):
            session.execute(plan['plan_id'], True)

    def test_pick_then_ready_acquires_motion_lease_only_after_successful_pick(self):
        calls = []
        planner = SimpleNamespace(plan=lambda text: Plan('夹取后回预备位，保持夹持', (
            Step('pick_color', {'color': 'red', 'place_after': False}), Step('return_ready'))))
        def execute(name, arguments=None):
            calls.append((name, arguments))
            return 'done'
        robot = SimpleNamespace(execute=execute)
        session = AgentSession(planner, robot, motion_authorized=True, lease=lambda acquire: calls.append(acquire))
        plan = session.plan('夹取红块后回预备位，不放置')
        session.execute(plan['plan_id'], True)
        self.assertEqual(calls, [('pick_color', {'color': 'red', 'place_after': False}),
                                 True, ('return_ready', {}), False])
        calls.clear()
        def fail(name, arguments=None):
            calls.append(name)
            raise RuntimeError('grasp failed')
        robot.execute = fail
        plan = session.plan('夹取红块后回预备位，不放置')
        with self.assertRaisesRegex(RuntimeError, 'grasp failed'):
            session.execute(plan['plan_id'], True)
        self.assertEqual(calls, ['pick_color'])

    def test_joint_transport_checks_payload_collision_before_sending_motion(self):
        from zekeep_llm.motion import JOINT_NAMES
        calls, requests = [], []
        def check(client, request, label, **kwargs):
            requests.append(request)
            # The scene's held object collides halfway along this joint path.
            return SimpleNamespace(valid=not (request.robot_state.is_diff
                                              and request.robot_state.joint_state.position[0] >= .03))
        robot = SimpleNamespace(_positions={name: 0.0 for name in JOINT_NAMES},
            _validity=SimpleNamespace(wait_for_service=lambda **kwargs: True), _call_service=check,
            _wait_for_fresh_telemetry=lambda: None, _require_ready_status=lambda: None,
            _trajectory=None, run_action=lambda *args: calls.append(args))
        robot.collision_check = lambda *args: RobotTools.collision_check(robot, *args)
        target = {'positions_rad': [.1, .1, .1, 0, 0, 0], 'duration_s': 2}
        with self.assertRaisesRegex(RuntimeError, '碰撞检查失败'):
            RobotTools.move_joints(robot, target)
        self.assertEqual(calls, [])
        self.assertTrue(all(request.robot_state.is_diff for request in requests))
        robot._call_service = lambda *args, **kwargs: SimpleNamespace(valid=True)
        RobotTools.move_joints(robot, target)
        self.assertEqual(len(calls), 1)


if __name__ == '__main__':
    unittest.main()
