"""Color configuration and grasp rejection diagnostics, without hardware access."""
from contextlib import ExitStack
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import cv2
import yaml

PROJECT = Path(__file__).resolve().parents[2] / 'zekeep_grasp'
sys.path.insert(0, str(PROJECT))

from utils.yolo_utils import YoloDetection, detect_objects, load_yolo


class WebColorDetectionTest(unittest.TestCase):
    def test_pixel_color_corrects_labels_and_rejects_uncertain_masks(self):
        names = ('red block', 'blue block', 'green block', 'purple block', 'gray block', 'yellow block')
        colors = ((0, 200, 150), (110, 200, 150), (55, 200, 150), (150, 200, 150), (0, 0, 100), (25, 200, 150))
        for expected, hsv_color in zip(names, colors):
            with self.subTest(color=expected):
                hsv = np.empty((30, 30, 3), dtype=np.uint8)
                hsv[:] = hsv_color
                frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
                candidates = [YoloDetection(0, index, name, .99, (0, 0, 29, 29), np.ones((30, 30)))
                              for index, name in enumerate(names)]
                model = Mock()
                model.predict.return_value = ['raw network output']
                with patch('utils.yolo_utils.collect_detections', return_value=candidates):
                    results, verified = detect_objects(model, frame, {})
                self.assertEqual(results, ['raw network output'])
                self.assertEqual([item.class_name for item in verified], [expected])
                self.assertEqual(verified[0].conf, .99)
                self.assertEqual([item.class_name for item in candidates], list(names))
        frame = np.zeros((30, 30, 3), dtype=np.uint8)
        frame[:, :15] = [0, 0, 200]
        frame[:, 15:] = [200, 0, 0]
        with patch('utils.yolo_utils.collect_detections', return_value=candidates):
            self.assertEqual(detect_objects(model, frame, {})[1], [])
        tiny = YoloDetection(0, 0, 'red block', .99, (0, 0, 1, 1), np.zeros((30, 30)))
        ordinary = YoloDetection(0, 1, 'paper cup', .7, (0, 0, 29, 29), np.ones((30, 30)))
        with patch('utils.yolo_utils.collect_detections', return_value=[tiny, ordinary]):
            self.assertEqual(detect_objects(model, frame, {})[1], [ordinary])
        frame[:] = [0, 0, 200]
        wrong = YoloDetection(0, 0, 'purple block', .579, (0, 0, 29, 29), np.ones((30, 30)))
        with patch('utils.yolo_utils.collect_detections', return_value=[wrong]):
            fixed = detect_objects(model, frame, {})[1]
        self.assertEqual(fixed[0].class_name, 'red block')
        self.assertEqual(fixed[0].conf, .579)
        self.assertIs(fixed[0].mask, wrong.mask)
        self.assertEqual(wrong.class_name, 'purple block')
        for invalid in (.5, 0, float('nan'), 1.1):
            with self.subTest(minimum=invalid), self.assertRaises(ValueError):
                detect_objects(model, frame, {'block_color_min_fraction': invalid})

    def test_default_detector_passes_purple_to_open_vocabulary_model(self):
        cfg = yaml.safe_load((PROJECT / 'config/default.yaml').read_text())
        model = Mock()
        with patch.dict(sys.modules, {'ultralytics': SimpleNamespace(YOLO=lambda path: model)}):
            loaded, opts = load_yolo(cfg, project_root=PROJECT)
        self.assertIs(loaded, model)
        model.set_classes.assert_called_once_with(cfg['yolo']['custom_classes'])
        self.assertIn('purple block', opts['custom_classes'])
        self.assertIn('blue block', opts['custom_classes'])
        self.assertIn('green block', opts['custom_classes'])
        self.assertAlmostEqual(cfg['robot']['gripper_max_width_m'], .070 * 1.35 / 1.45)
        self.assertEqual(opts['imgsz'], 1280)
        model.predict.return_value = []
        frame = np.zeros((800, 1280, 3), dtype=np.uint8)
        self.assertEqual(detect_objects(model, frame, opts), ([], []))
        model.predict.assert_called_once_with(frame, verbose=False, device='0',
            imgsz=1280, conf=.3, iou=.45)
        del cfg['yolo']['imgsz']
        with patch.dict(sys.modules, {'ultralytics': SimpleNamespace(YOLO=lambda path: model)}):
            _, legacy_opts = load_yolo(cfg, project_root=PROJECT)
        self.assertEqual(legacy_opts['imgsz'], 640)

    def test_width_rejection_is_specific_and_keeps_physical_limit(self):
        from utils import graspnet_utils as g
        frame = np.zeros((10, 10, 3), dtype=np.uint8)
        target = YoloDetection(0, 0, 'green block', .36, (0, 0, 10, 10),
                               np.ones((10, 10), dtype=bool))
        for widths, rejected in (([.072, .082], True), ([.064, .082], False), ([], False)):
            with self.subTest(widths=widths), ExitStack() as stack:
                data = np.zeros((len(widths), 17))
                data[:, 0] = .8
                data[:, 1] = widths
                candidates = g.GraspGroup(data)
                stack.enter_context(patch.object(g, 'build_end_points', return_value=(None, None, None)))
                stack.enter_context(patch.object(g, 'infer_grasps', return_value=(candidates,
                    {'decoded': 1024, 'collision_removed': 810, 'pre_collision': 1024})))
                for name in ('filter_grasps_by_bbox', 'filter_grasps_by_target_mask',
                             'bias_grasp_scores_by_mask_center'):
                    stack.enter_context(patch.object(g, name, side_effect=lambda grasps, *a, **kw: grasps))
                stack.enter_context(patch.object(g, 'estimate_target_depth', return_value=None))
                result = g.infer_frame(None, frame, np.zeros((10, 10)), np.eye(3),
                    num_point=10, min_depth=.1, max_depth=2, collision_thresh=.01,
                    selected_target_override=target, max_grasp_width_m=.070 * 1.35 / 1.45)
                self.assertEqual('rejected: width filter removed all candidates' in result.status, rejected)
                self.assertEqual(len(result.grasps), sum(width <= .070 * 1.35 / 1.45 for width in widths))
                if rejected:
                    self.assertIsNone(result.best)
                    self.assertIn('width_limit=0.065m', result.status)
                    self.assertIn('width=[0.072,0.082]m', result.status)


if __name__ == '__main__':
    unittest.main()
