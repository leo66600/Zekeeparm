"""Chinese display cleanup must never alter detection data or motion review."""

from contextlib import redirect_stdout
import copy
import io
import json
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from zekeep_llm.planner import Plan, Step
from zekeep_llm.terminal import _describe_detections, _run_task


def reported_detections():
    return {'frame_id': 'base_link', 'observation_window_s': 2, 'observations': [
        {'class_name': 'blue block', 'confidence': .8469, 'bbox': [773, 499, 853, 582], 'hits': 8},
        {'class_name': 'purple block', 'confidence': .7229, 'bbox': [454, 647, 537, 747], 'hits': 8},
        {'class_name': 'blue block', 'confidence': .4819, 'bbox': [600, 523, 656, 595], 'hits': 8},
        {'class_name': 'red block', 'confidence': .3775, 'bbox': [713, 589, 795, 662], 'hits': 4},
        {'class_name': 'cardboard box', 'confidence': .6674, 'bbox': [1000, 0, 1279, 177], 'hits': 5},
        {'class_name': 'purple block', 'confidence': .3658, 'bbox': [713, 589, 795, 663], 'hits': 2},
    ]}


class TerminalDetectionTest(unittest.TestCase):
    def test_reported_scene_has_five_objects_without_duplicate_color_or_raw_json(self):
        data = reported_detections()
        before = copy.deepcopy(data)
        robot = SimpleNamespace(execute=Mock(return_value=json.dumps(data)), stop_best_effort=Mock())
        planner = SimpleNamespace(plan=lambda _: Plan('查询桌面', (Step('detect_blocks'),)))
        output = io.StringIO()
        with redirect_stdout(output), patch('builtins.input', side_effect=AssertionError('query must not require confirmation')):
            self.assertTrue(_run_task(planner, robot, '桌面上有什么', False))
        text = output.getvalue()
        for expected in ('识别到 5 个物体', '蓝色物块：2 个', '紫色物块：1 个', '纸箱：1 个',
                         '红色物块：1 个（识别待确认）'):
            self.assertIn(expected, text)
        for technical in ('summary', 'steps', 'frame_id', 'bbox', 'base_link', 'blue block', 'confidence'):
            self.assertNotIn(technical, text)
        self.assertEqual(data, before)
        robot.stop_best_effort.assert_not_called()

    def test_separate_same_color_targets_remain_separate_and_aliases_translate(self):
        text = _describe_detections(json.dumps({'detections': [
            {'color': 'blue', 'confidence': .8, 'bbox': [0, 0, 20, 20]},
            {'class_name': 'blue block', 'confidence': .9, 'bbox': [30, 0, 50, 20]},
            {'class_name': 'paper cup', 'confidence': .7},
        ]}))
        self.assertIn('识别到 3 个物体', text)
        self.assertIn('蓝色物块：2 个', text)
        self.assertIn('纸杯：1 个', text)

    def test_invalid_and_empty_results_do_not_invent_a_scene(self):
        for raw in ('not-json', '[]', '{}', '{"observations": null}'):
            self.assertIn('格式异常', _describe_detections(raw))
        self.assertIn('不代表桌面上没有东西', _describe_detections('{"detections": []}'))
        raw = json.dumps({'detections': [None, {'class_name': 'red block', 'confidence': float('nan')}]})
        self.assertIn('没有有效识别记录', _describe_detections(raw))
        raw = json.dumps({'detections': [{'class_name': 'unmapped class', 'confidence': .9, 'bbox': [1, 2, 1, 4]}]})
        self.assertIn('其他物体：1 个', _describe_detections(raw))

    def test_motion_parameters_are_visible_before_confirmation_in_a_mixed_plan(self):
        arguments = {'positions_rad': [.1, .2, .3, 0, 0, 0]}
        planner = SimpleNamespace(plan=lambda _: Plan('查询后移动', (Step('detect_blocks'), Step('move_joints', arguments))))
        robot = Mock()
        output = io.StringIO()
        def cancel(prompt):
            self.assertIn('positions_rad', output.getvalue())
            self.assertIn('0.3', output.getvalue())
            return 'no'
        with redirect_stdout(output), patch('builtins.input', side_effect=cancel):
            self.assertFalse(_run_task(planner, robot, '查询后移动', False))
        robot.execute.assert_not_called()

    def test_dry_run_keeps_the_plan_and_does_not_query_the_robot(self):
        planner = SimpleNamespace(plan=lambda _: Plan('查询桌面', (Step('detect_blocks'),)))
        robot = Mock()
        output = io.StringIO()
        with redirect_stdout(output):
            self.assertTrue(_run_task(planner, robot, '桌面上有什么', True))
        self.assertIn('"steps"', output.getvalue())
        self.assertIn('dry-run', output.getvalue())
        robot.execute.assert_not_called()


if __name__ == '__main__':
    unittest.main()
