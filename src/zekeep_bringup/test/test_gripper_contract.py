"""Shared motor/width calibration, checked without hardware access."""
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/zekeep_grasp'))
from drivers.robot.grasp_driver import gripper_distance_to_motor_position


class GripperContractTest(unittest.TestCase):
    def test_mapping_and_over_limit_requests(self):
        width = .070 * 1.35 / 1.45
        self.assertAlmostEqual(gripper_distance_to_motor_position(width), 1.35)
        self.assertAlmostEqual(gripper_distance_to_motor_position(.035), .725)
        self.assertEqual(gripper_distance_to_motor_position(0), 0)
        for invalid in (-.001, .070, width + .0001, float('nan')):
            with self.subTest(width=invalid), self.assertRaises(ValueError):
                gripper_distance_to_motor_position(invalid)

    def test_all_motion_profiles_and_models_share_limits(self):
        width = .070 * 1.35 / 1.45
        for path in (ROOT / 'src/zekeep_bringup/config').glob('zekeep_hardware*.yaml'):
            cfg = yaml.safe_load(path.read_text())['models']['sixaxis']['overrides']
            self.assertEqual(cfg['joint_calibration']['gripper']['hard_upper'], 1.35)
            self.assertEqual(cfg['gripper']['position_limits'], {'open': 1.35, 'close': 0})
            self.assertAlmostEqual(cfg['gripper']['max_width_m'], width)
        cfg = yaml.safe_load((ROOT / 'src/zekeep_grasp/config/default.yaml').read_text())
        self.assertEqual(cfg['robot']['gripper']['dm']['angle_open'], 1.35)
        self.assertAlmostEqual(cfg['robot']['gripper_max_width_m'], width)
        for name in ('src/zekeep_bringup/config/carry_params.yaml',
                     'src/zekeep_moveit_config/config/pick_place_params.yaml'):
            cfg = yaml.safe_load((ROOT / name).read_text())
            params = next(iter(cfg.values()))['ros__parameters']['gripper']
            self.assertEqual(params['hardware_open_position'], 1.35)
            self.assertAlmostEqual(params['open_position'], width / 2)
        ros = ROOT / 'src/zekeep_bringup/description/urdf/sixaxis.urdf'
        sdk = ROOT / 'src/zekeep_grasp/config/sixaxis.urdf'
        self.assertEqual(ros.read_bytes(), sdk.read_bytes())
        model = ET.parse(ros)
        for joint in ('gripper_joint', 'right_joint'):
            self.assertAlmostEqual(float(model.find(f"joint[@name='{joint}']/limit").get('upper')), width / 2)

    def test_ros_feedback_and_llm_commands_use_the_same_width(self):
        from zekeepcontroller.ros_publishers import _gripper_motor_to_joint_position
        from zekeep_llm.planner import TOOL_SCHEMAS, validate_value
        width = .070 * 1.35 / 1.45
        self.assertAlmostEqual(_gripper_motor_to_joint_position(.725, 1.35, 0, width), .0175)
        self.assertAlmostEqual(_gripper_motor_to_joint_position(1.35, 1.35, 0, width), width / 2)
        rule = TOOL_SCHEMAS['set_gripper_opening_mm']['properties']['opening_mm']
        self.assertAlmostEqual(rule['maximum'], width * 1000)
        validate_value(65.17, rule, 'opening_mm')
        with self.assertRaises(ValueError):
            validate_value(70, rule, 'opening_mm')


if __name__ == '__main__':
    unittest.main()
