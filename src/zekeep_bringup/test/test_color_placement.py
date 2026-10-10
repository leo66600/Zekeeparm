"""Color placement targets and SDK execution, without camera or motor access."""
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import numpy as np
import yaml

WORKSPACE = Path(__file__).resolve().parents[3]
PROJECT = WORKSPACE / 'src/zekeep_grasp'
sys.path.insert(0, str(PROJECT))

from utils.fixed_placement import resolve_joint_placement_target


class ColorPlacementTest(unittest.TestCase):
    def setUp(self):
        self.cfg = yaml.safe_load((PROJECT / 'config/default.yaml').read_text())

    def test_named_targets_match_taught_values_srdf_and_joint_limits(self):
        captured = (
            ('red_place', [0.9042873382568359, 1.4967193603515625, 0.8607997894287109,
                           -0.2733268737792969, -0.2607383728027344, -0.10509681701660156]),
            ('blue_place', [0.7135496139526367, 1.7694740295410156, 1.2647819519042969,
                            -0.3179597854614258, -0.08258914947509766, 0.09708595275878906]),
            ('green_place', [0.5384531021118164, 2.2215232849121094, 1.835087776184082,
                             -0.3858623504638672, -0.07495975494384766, 0.10433387756347656]),
        )
        states = ET.parse(WORKSPACE / 'src/zekeep_moveit_config/config/zekeep.srdf')
        model = ET.parse((PROJECT / self.cfg['robot']['urdf_path']).resolve())
        for name, expected in captured:
            with self.subTest(name=name):
                class_name = name.removesuffix('_place') + ' block'
                chosen, joints = resolve_joint_placement_target(self.cfg['place'], class_name)
                self.assertEqual(chosen, name)
                np.testing.assert_array_equal(joints, expected)
                state = states.find(f"group_state[@name='{name}'][@group='arm']")
                self.assertIsNotNone(state)
                for i, value in enumerate(joints, 1):
                    joint = f'joint{i}'
                    self.assertEqual(float(state.find(f"joint[@name='{joint}']").get('value')), value)
                    limit = model.find(f"joint[@name='{joint}']/limit")
                    self.assertLessEqual(float(limit.get('lower')), value)
                    self.assertLessEqual(value, float(limit.get('upper')))

    def test_unmapped_objects_keep_original_fixed_target(self):
        expected = [-1.3040743, 1.6420612, 1.3582439, -1.2678337, 0.0356684, -0.0741968]
        for class_name in ('purple block', 'gray block', 'block', 'red pen', 'blue bottle', ''):
            with self.subTest(class_name=class_name):
                name, joints = resolve_joint_placement_target(self.cfg['place'], class_name)
                self.assertEqual(name, 'fixed_joint')
                np.testing.assert_array_equal(joints, expected)

    def test_malformed_special_target_never_falls_back(self):
        place = {'joint_target_rad': [0]*6, 'class_targets': {'red block': 'red_place'}}
        with self.assertRaisesRegex(ValueError, 'unknown placement target'):
            resolve_joint_placement_target(place, 'red block')
        for invalid in ([0]*5, [[0]*6], [0]*5 + [float('nan')], [0]*5 + [float('inf')]):
            place['named_joint_targets_rad'] = {'red_place': invalid}
            with self.subTest(invalid=invalid), self.assertRaisesRegex(ValueError, 'six finite'):
                resolve_joint_placement_target(place, 'red block')
        name, _ = resolve_joint_placement_target(self.cfg['place'], ' Red Block ')
        self.assertEqual(name, 'red_place')

    def test_sdk_places_detected_classes_and_returns_ready(self):
        from scripts.main import _execute_candidate
        cfg = self.cfg
        cfg['place'].update(pre_grasp_wait_s=0, post_grasp_wait_s=0,
                            pre_release_wait_s=0, post_release_wait_s=0)
        for class_name in ('red block', 'blue block', 'green block', 'gray block', 'red pen'):
            with self.subTest(class_name=class_name):
                robot = Mock()
                robot.grasp.return_value = True
                robot.ready_joints.return_value = np.zeros(6)
                with patch('scripts.main.time.sleep'):
                    self.assertTrue(_execute_candidate(robot, ([0]*6,)*3,
                        [np.zeros(6)]*3, cfg, class_name=class_name))
                name, expected = resolve_joint_placement_target(cfg['place'], class_name)
                np.testing.assert_array_equal(robot.move_joints_and_wait.call_args_list[0].args[0], expected)
                np.testing.assert_array_equal(robot.move_joints_and_wait.call_args_list[1].args[0], np.zeros(6))
                robot.release.assert_called_once()
        robot = Mock()
        cfg['place']['named_joint_targets_rad']['red_place'] = [0]*5
        with self.assertRaisesRegex(ValueError, 'six finite'):
            _execute_candidate(robot, ([0]*6,)*3, [np.zeros(6)]*3, cfg, class_name='red block')
        robot.open_gripper.assert_not_called()


if __name__ == '__main__':
    unittest.main()
