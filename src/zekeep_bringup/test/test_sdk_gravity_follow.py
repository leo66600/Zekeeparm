"""Gravity test mode records hand-guided poses without connecting motors."""
import csv
import io
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/zekeep_grasp'))
from scripts import diagnose_joint_tracking as diagnostic


class GravityFollowTest(unittest.TestCase):
    def test_offline_mode_removes_position_spring_without_changing_file(self):
        path = ROOT / 'src/zekeep_grasp/config/default.yaml'
        original = path.read_bytes()
        cfg = yaml.safe_load(original)
        robot = Mock()
        robot.cfg = cfg
        robot.control_loop_active = False
        robot.kinematics.joint_limits = np.array([[-3.7, 3.7]] * 6)
        with patch.object(diagnostic, 'OfficialSdkRobot', return_value=robot) as factory:
            result = diagnostic.main(['--config', str(path), '--control-mode', 'mit', '--gravity-follow'])
        self.assertEqual(result, 0)
        used = factory.call_args.args[0]['official_sdk']
        self.assertEqual(used['mit_kp'], [0.] * 6)
        self.assertEqual(used['mit_kd'], cfg['official_sdk']['mit_kd'])
        self.assertEqual(used['gravity_tau_scale'], cfg['official_sdk']['gravity_tau_scale'])
        robot.connect.assert_not_called()
        robot.move_joints_and_wait.assert_not_called()
        self.assertEqual(original, path.read_bytes())

    def test_recording_tracks_new_poses_and_torque_signs_until_enter(self):
        positions = [np.array([0., .2, .3, 0., 0., 0.]),
                     np.array([0., .5, .3, 0., 0., 0.])]
        arm = Mock()
        arm.get_state.side_effect = [(q, np.zeros(6), np.ones(6)) for q in positions]
        model = SimpleNamespace(nq=8, createData=lambda: object())
        robot = SimpleNamespace(_arm=arm, _controller=SimpleNamespace(_model=model),
            control_loop_active=True, kinematics=SimpleNamespace(joint_limits=np.array([[-3.7, 3.7]] * 6)),
            gravity_tau_scale=np.array([1., 1.1, 1., 1., 1., 1.]),
            gravity_tau_scale_positive=np.array([1., 1.1, 1.1, 1., 1., 1.]))
        stream = io.StringIO()
        with patch.object(diagnostic.sys, 'stdin', io.StringIO('\n')), \
             patch.object(diagnostic.select, 'select', side_effect=[([], [], []), ([], [], []), ([1], [], [])]), \
             patch.object(diagnostic.pin, 'computeGeneralizedGravity', return_value=np.array([0., -2., 3., 0., 0., 0., 0., 0.])):
            diagnostic.record_gravity_follow(robot, .05, stream)
        rows = list(csv.DictReader(io.StringIO(stream.getvalue())))
        self.assertEqual(len(rows), 12)
        self.assertEqual([float(rows[index]['actual_rad']) for index in (1, 7)], [.2, .5])
        self.assertAlmostEqual(float(rows[1]['model_gravity_ff_nm']), -2.2)
        self.assertAlmostEqual(float(rows[2]['model_gravity_ff_nm']), 3.3)
        self.assertNotIn('command_rad', rows[0])

    def test_invalid_feedback_is_rejected(self):
        arm = Mock()
        arm.get_state.return_value = (np.full(6, np.nan), np.zeros(6), np.zeros(6))
        robot = SimpleNamespace(_arm=arm, control_loop_active=True,
            _controller=SimpleNamespace(_model=SimpleNamespace(createData=lambda: object())))
        with patch.object(diagnostic.select, 'select', return_value=([], [], [])), \
             self.assertRaisesRegex(RuntimeError, 'invalid gravity-test feedback'):
            diagnostic.record_gravity_follow(robot, .05, io.StringIO())


if __name__ == '__main__':
    unittest.main()
