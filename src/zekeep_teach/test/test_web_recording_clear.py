"""Check recording replacement with fake feedback and motor calls only."""
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np

from zekeep_teach.robot import RosTeachRobot, TeachConfig, TeachState
from zekeep_teach.session import TeachingSample
from zekeep_teach.web_session import WebSession


class WebRecordingClearTest(unittest.TestCase):
    def test_clear_then_record_and_replay_uses_only_new_data(self):
        clock = [10.0]
        client = Mock()
        feedback = SimpleNamespace(positions=np.full(6, .1), velocities=np.zeros(6), sequence=7)
        client.latest_sample.return_value = feedback
        robot = RosTeachRobot(client, TeachConfig(), clock=lambda: clock[0])
        robot.state = TeachState.GUIDING
        robot._session_started_monotonic = clock[0]
        robot.capture_point()
        robot.toggle_continuous_recording()
        robot.sample_continuous()
        robot.toggle_continuous_recording()
        robot.state = TeachState.HOLD
        robot._wait_until_stationary = Mock()
        robot._wait_for_settle = Mock()
        robot.replay_points()
        np.testing.assert_allclose(client.execute_direct_joint_target.call_args.args[0], .1)
        with TemporaryDirectory() as directory:
            session = WebSession(lambda: robot, directory, authorized=True)
            session.robot, session.owner = robot, 'teach'
            session.command('save', 'original')
            saved = Path(directory) / 'original.json'
            saved_bytes = saved.read_bytes()
            clock[0] = 20.0
            client.reset_mock()
            session.command('clear')
            self.assertEqual(client.mock_calls, [], 'Clearing must not call the controller')
            self.assertEqual(session.status()['points'], 0)
            self.assertEqual(session.status()['path'], 0)
            self.assertEqual(robot.state, TeachState.HOLD)
            self.assertEqual(saved.read_bytes(), saved_bytes)
            robot.state = TeachState.GUIDING
            feedback.positions = np.full(6, .4)
            clock[0] = 20.2
            robot.capture_point()
            robot.toggle_continuous_recording()
            robot.sample_continuous()
            self.assertAlmostEqual(robot.session.points[0].captured_s, .2)
            feedback.positions = np.full(6, .6)
            feedback.sequence += 1
            clock[0] = 20.4
            robot.sample_continuous()
            robot.toggle_continuous_recording()
            robot.state = TeachState.HOLD
            robot.replay_points()
            np.testing.assert_allclose(client.execute_direct_joint_target.call_args.args[0], .4)
            robot.replay_path()
            trajectory = client.execute_joint_trajectory.call_args.args[0]
            np.testing.assert_allclose(trajectory[-1].positions, .6)
            self.assertEqual(len(robot.session.points), 1)
            self.assertEqual(len(robot.session.path), 2)

    def test_clear_rejects_recording_replay_and_active_worker(self):
        robot = RosTeachRobot(Mock(), TeachConfig())
        robot.session.points.append(TeachingSample.create([.1]*6, captured_s=0))
        original = robot.session
        for state in (TeachState.DISCONNECTED, TeachState.POINT_REPLAY, TeachState.PATH_REPLAY):
            robot.state = state
            with self.assertRaises(RuntimeError):
                robot.clear_recording()
            self.assertIs(robot.session, original)
        robot.state = TeachState.GUIDING
        robot._continuous_recording = True
        with self.assertRaisesRegex(RuntimeError, 'stop continuous'):
            robot.clear_recording()
        robot._continuous_recording = False
        with TemporaryDirectory() as directory:
            session = WebSession(lambda: robot, directory, authorized=True)
            session.robot, session.owner = robot, 'teach'
            session.worker = SimpleNamespace(is_alive=lambda: True)
            with self.assertRaisesRegex(RuntimeError, 'active task'):
                session.command('clear')
            self.assertIs(robot.session, original)


if __name__ == '__main__':
    unittest.main()
