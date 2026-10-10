"""Keep joint3 limits consistent across motion paths; never connect hardware."""
from pathlib import Path
import re
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / 'src' / name) for name in ('zekeepcontroller', 'zekeep_llm', 'zekeep_teach')]
from zekeepcontroller.joint_mapping import joint_mapping_from_config
from zekeep_llm.motion import checked_joints
from zekeep_teach.session import TeachingSample, TeachingSession
from zekeep_teach.web_session import validate_session


class Joint3LimitsTest(unittest.TestCase):
    def test_reported_target_and_boundaries_pass_all_motion_guards(self):
        for path in (ROOT / 'src/zekeep_bringup/config').glob('zekeep_hardware*.yaml'):
            config = yaml.safe_load(path.read_text())['models']['sixaxis']['overrides']
            mapping = joint_mapping_from_config(config, [f'joint{i}' for i in range(1, 7)])
            for angle in (-0.01, -0.00362396, 0, 3.7):
                with self.subTest(config=path.name, angle=angle):
                    q = [0., 0., angle, 0., 0., 0.]
                    self.assertAlmostEqual(mapping.positions_to_motor(q.copy())[2], -angle)
                    self.assertEqual(checked_joints(q)[2], angle)
                    validate_session(TeachingSession(points=[TeachingSample.create(q, captured_s=0)]))
            for index, angle in ((2, -0.012), (2, 3.702), (1, -0.00362396)):
                q = np.zeros(6)
                q[index] = angle
                with self.subTest(config=path.name, index=index, angle=angle):
                    with self.assertRaises(ValueError):
                        mapping.validate_positions(q)
                    with self.assertRaises(ValueError):
                        checked_joints(q.tolist())
                    with self.assertRaises(ValueError):
                        validate_session(TeachingSession(points=[TeachingSample.create(q, captured_s=0)]))

    def test_planning_and_visual_models_match_hardware(self):
        for name in ('src/zekeep_bringup/description/urdf/sixaxis.urdf', 'src/zekeep_grasp/config/sixaxis.urdf'):
            tree = ET.parse(ROOT / name)
            self.assertEqual(float(tree.find(".//joint[@name='joint3']/limit").get('lower')), -0.01)
            self.assertEqual(float(tree.find(".//joint[@name='joint3']/limit").get('upper')), 3.7)
            self.assertEqual(float(tree.find(".//joint[@name='joint2']/limit").get('lower')), 0)
        limits = yaml.safe_load((ROOT / 'src/zekeep_moveit_config/config/joint_limits.yaml').read_text())['joint_limits']
        self.assertEqual((limits['joint3']['min_position'], limits['joint3']['max_position']), (-0.01, 3.7))
        self.assertEqual(limits['joint2']['min_position'], 0)
        limits = yaml.safe_load((ROOT / 'src/zekeep_grasp/config/default.yaml').read_text())['robot']['joint_mapping']
        self.assertEqual((limits['joint3']['lower'], limits['joint3']['upper']), (-0.01, 3.7))

    def test_web_limits_match_motion_guards(self):
        for name in ('server.js', 'public/js/rebot-sim.js'):
            source = (ROOT / 'src/zekeep_bringup/web' / name).read_text()
            match = re.search(r"name: 'joint3'[^}]*min: ([\d.-]+), max: ([\d.]+)", source)
            self.assertIsNotNone(match)
            self.assertEqual(tuple(map(float, match.groups())), (-0.01, 3.7))


if __name__ == '__main__':
    unittest.main()
