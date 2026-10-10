"""Check missed frames and distinct same-color objects without hardware."""
import unittest
from types import SimpleNamespace
import json

import rclpy
from std_msgs.msg import String

from zekeep_llm.detection_history import recent_observations
from zekeep_llm.robot import RobotTools


def block(name, x, confidence=.4):
    return dict(class_name=name, color=name, bbox=[x, 10, x+30, 40], confidence=confidence,
                id='snapshot', position=[.3, 0, .02])


def frame(stamp, detections, **values):
    return dict(stamp={'sec': int(stamp), 'nanosec': round((stamp % 1)*1e9)},
                calibration_status='CALIBRATED_STATIONARY', calibration_identity='installation',
                detections=detections, **values)


class DetectionHistoryTest(unittest.TestCase):
    def test_query_spins_for_new_frames_instead_of_reading_one_cached_message(self):
        rclpy.init()
        robot = RobotTools('vision_history_test')
        publisher = robot.create_publisher(String, '/vision_history_test/web/vision/detections', 1)
        patterns = [[block('blue block', 10), block('red block', 110)],
                    [block('blue block', 60)],
                    [block('blue block', 10), block('blue block', 60), block('red block', 110)], []]
        sent = []
        def publish():
            stamp = robot.get_clock().now().to_msg()
            body = frame(0, patterns[len(sent) % len(patterns)])
            body['stamp'] = {'sec': stamp.sec, 'nanosec': stamp.nanosec}
            publisher.publish(String(data=json.dumps(body)))
            sent.append(body)
        robot.create_timer(.1, publish)
        try:
            result = robot.detect_blocks()
            self.assertGreaterEqual(len(sent), 5)
            self.assertEqual(sorted(item['class_name'] for item in result['observations']), ['blue block', 'blue block', 'red block'])
            self.assertEqual(result['detections'], robot._detections['detections'])
            self.assertEqual(result['observation_window_s'], 2)
        finally:
            robot.destroy_node()
            rclpy.shutdown()

    def test_two_blue_one_red_survive_intermittent_misses_without_motion_coordinates(self):
        blue1, blue2, red = block('blue block', 10), block('blue block', 60), block('red block', 110)
        frames = [frame(10, [blue1, red]), frame(10.2, [blue2]), frame(10.4, [blue1, blue2, red]), frame(10.6, [])]
        result = recent_observations(frames, 10.7, 'installation')
        self.assertEqual(sorted(item['class_name'] for item in result), ['blue block', 'blue block', 'red block'])
        self.assertTrue(all(item['hits'] == 2 for item in result))
        self.assertTrue(all('position' not in item and 'id' not in item for item in result))
        self.assertEqual(recent_observations(frames, 12.5, 'installation'), [])

    def test_jitter_deduplicates_and_single_hit_duplicate_frames_and_old_identity_do_not_count(self):
        frames = [frame(10, [block('blue block', 10)]), frame(10.2, [block('blue block', 11)])]
        self.assertEqual(len(recent_observations(frames, 10.3, 'installation')), 1)
        self.assertEqual(recent_observations([frames[0], frames[0]], 10.3, 'installation'), [])
        self.assertEqual(recent_observations(frames, 10.3, 'different installation'), [])
        frames[1]['calibration_status'] = 'PREVIEW_ONLY'
        self.assertEqual(recent_observations(frames, 10.3, 'installation'), [])

    def test_text_observations_include_2d_detections_without_grasp_targets(self):
        raw = dict(class_name='cardboard box', confidence=.8, bbox=[10, 10, 50, 50])
        frames = [frame(10, [], visual_detections=[raw]),
                  frame(10.2, [], visual_detections=[raw])]
        observations = recent_observations(frames, 10.3, 'installation')
        self.assertEqual(len(observations), 1)
        self.assertEqual(observations[0]['class_name'], 'cardboard box')
        self.assertNotIn('id', observations[0])
        self.assertNotIn('position', observations[0])
        self.assertTrue(all(body['detections'] == [] for body in frames))
        frames[1]['visual_detections'] = []
        self.assertEqual(recent_observations(frames, 10.3, 'installation'), [])

    def test_grasp_does_not_use_history_and_known_same_color_ambiguity_rejects(self):
        calls = []
        body = {'detections': [], 'observations': [block('blue block', 10)]}
        robot = SimpleNamespace(detect_blocks=lambda: body, _pick=None, _delegated_busy=False,
            run_action=lambda client, goal, label, timeout: calls.append(goal.detection_id) or SimpleNamespace(message='done'))
        robot.run_web_task = lambda *args: RobotTools.run_web_task(robot, *args)
        with self.assertRaisesRegex(RuntimeError, '没有可执行'):
            RobotTools.pick_color(robot, 'blue')
        body['detections'] = [block('blue block', 10)]
        body['observations'].append(block('blue block', 60))
        with self.assertRaisesRegex(RuntimeError, '多个同色'):
            RobotTools.pick_color(robot, 'blue')
        self.assertEqual(calls, [])
        body['observations'] = [block('blue block', 10)]
        self.assertEqual(RobotTools.pick_color(robot, 'blue'), 'done')
        self.assertEqual(calls, ['snapshot'])


if __name__ == '__main__':
    unittest.main()
