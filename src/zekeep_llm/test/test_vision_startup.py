"""Exercise automatic perception startup without a camera or motor commands."""

import json
import os
from pathlib import Path
import signal
import tempfile
import unittest
from unittest.mock import patch

import rclpy
from ament_index_python.packages import PackageNotFoundError
from std_msgs.msg import String

from zekeep_llm.robot import RobotTools


class VisionStartupTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.workspace = Path(self.directory.name)
        self.prefix = self.workspace/'install/zekeep_shadow'
        entry = self.prefix/'lib/zekeep_shadow/web_vision'
        entry.parent.mkdir(parents=True)
        entry.touch()
        self.python = self.workspace/'vision-python'
        self.python.touch()
        self.python.chmod(0o700)
        self.patches = [
            patch('zekeep_llm.robot.Path.home', return_value=self.workspace),
            patch('zekeep_llm.robot.get_package_prefix', return_value=str(self.prefix)),
            patch.dict(os.environ, {'ZKEEP_WS': str(self.workspace),
                                    'ZKEEP_VISION_PYTHON': str(self.python),
                                    'ZKEEP_LLM_API_KEY': 'test-secret'}),
            patch('zekeep_llm.robot.subprocess.Popen'),
        ]
        started = [item.start() for item in self.patches]
        self.start = started[-1]
        self.process = self.start.return_value
        self.process.poll.return_value = None
        rclpy.init()
        self.robot = RobotTools('vision_startup_test')

    def tearDown(self):
        self.robot.destroy_node()
        rclpy.shutdown()
        for item in reversed(self.patches):
            item.stop()
        self.directory.cleanup()

    def publish_frame(self):
        stamp = self.robot.get_clock().now().to_msg()
        self.robot._on_detections(String(data=json.dumps({
            'stamp': {'sec': stamp.sec, 'nanosec': stamp.nanosec},
            'calibration_status': 'CALIBRATED_STATIONARY',
            'calibration_identity': 'test-installation',
            'detections': [{'color': 'blue', 'confidence': .8}],
        })))

    def test_miniforge_default_custom_root_and_explicit_python(self):
        for mode in ('default', 'empty', 'custom', 'explicit'):
            with self.subTest(mode=mode):
                root = self.workspace / ('custom forge' if mode == 'custom' else 'miniforge3')
                python = root / 'envs/rebotarm/bin/python'
                if mode == 'explicit':
                    python = self.python
                python.parent.mkdir(parents=True, exist_ok=True)
                python.touch()
                python.chmod(0o700)
                env = {'ZKEEP_WS': str(self.workspace)}
                if mode == 'empty':
                    env['ZKEEP_MINIFORGE_DIR'] = ''
                elif mode == 'custom':
                    env['ZKEEP_MINIFORGE_DIR'] = str(root)
                elif mode == 'explicit':
                    env.update(ZKEEP_MINIFORGE_DIR=str(self.workspace / 'missing'),
                               ZKEEP_VISION_PYTHON=str(python))
                with patch.dict(os.environ, env, clear=True), \
                        patch.object(self.robot, 'count_publishers', return_value=0):
                    self.robot._start_vision()
                self.assertEqual(self.start.call_args.args[0][-3], str(python))
                self.robot._stop_vision()

    def test_query_starts_fixed_backend_once_waits_for_frames_and_cleans_up(self):
        def spin(predicate, timeout, message):
            if timeout == 45:
                self.publish_frame()
                self.assertTrue(predicate())

        with patch.object(self.robot, 'count_publishers', return_value=0), \
                patch.object(self.robot, '_spin_until', side_effect=spin):
            self.assertEqual(self.robot.detect_blocks()['detections'][0]['color'], 'blue')
            self.assertEqual(self.robot.detect_blocks()['detections'][0]['color'], 'blue')
        self.start.assert_called_once()
        argv = self.start.call_args.args[0]
        self.assertEqual(argv[-1], 'vision_startup_test')
        self.assertIn('grasp_backend:=yolo-graspnet', argv[2])
        self.assertEqual(Path(argv[-2]).name, 'web_vision')
        self.assertNotIn('ZKEEP_LLM_API_KEY', self.start.call_args.kwargs['env'])
        self.robot._stop_vision()
        self.process.send_signal.assert_called_once_with(signal.SIGINT)
        self.process.wait.assert_called_once_with(timeout=5)
        self.assertIsNone(self.robot._vision_file_lock)

    def test_existing_publisher_and_another_assistant_are_reused(self):
        with patch.object(self.robot, 'count_publishers', return_value=1):
            self.robot._start_vision()
        self.start.assert_not_called()
        peer = RobotTools('vision_startup_peer')
        try:
            with patch.object(self.robot, 'count_publishers', return_value=0), \
                    patch.object(peer, 'count_publishers', return_value=0):
                self.robot._start_vision()
                peer._start_vision()
            self.start.assert_called_once()
            self.assertIsNone(peer._vision_process)
        finally:
            peer.destroy_node()
        self.process.send_signal.assert_not_called()

    def test_startup_failure_reports_camera_error_and_releases_owned_resources(self):
        with patch.object(self.robot, 'count_publishers', return_value=0):
            self.robot._start_vision()
        self.robot._vision_log.write(b'camera is already in use')
        self.robot._vision_log.flush()
        self.process.poll.return_value = 1
        with self.assertRaisesRegex(RuntimeError, 'camera is already in use'):
            self.robot._vision_ready()
        self.assertIsNone(self.robot._vision_process)
        self.assertIsNone(self.robot._vision_log)
        self.assertIsNone(self.robot._vision_file_lock)
        self.process.send_signal.assert_not_called()

    def test_unusable_visual_environment_does_not_launch_any_process(self):
        with patch.object(self.robot, 'count_publishers', return_value=0), \
                patch.dict(os.environ, {'ZKEEP_VISION_PYTHON': str(self.workspace/'missing-python')}):
            with self.assertRaisesRegex(RuntimeError, '视觉 Python'):
                self.robot._start_vision()
        self.start.assert_not_called()

    def test_auto_start_does_not_bypass_calibration_checks(self):
        self.publish_frame()
        self.robot._detections['calibration_status'] = 'PREVIEW_ONLY: calibration mismatch'
        with patch.object(self.robot, '_spin_until'):
            with self.assertRaisesRegex(RuntimeError, 'calibration mismatch'):
                self.robot.detect_blocks()
        self.start.assert_not_called()

    def test_missing_optional_vision_package_reports_setup_error(self):
        with patch.object(self.robot, 'count_publishers', return_value=0), \
                patch('zekeep_llm.robot.get_package_prefix', side_effect=PackageNotFoundError('zekeep_shadow')):
            with self.assertRaisesRegex(RuntimeError, 'zekeep_shadow 包'):
                self.robot._start_vision()
        self.start.assert_not_called()


if __name__ == '__main__':
    unittest.main()
