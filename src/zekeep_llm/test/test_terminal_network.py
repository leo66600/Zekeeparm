"""The terminal must start its configured exit before creating the planner."""

import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from zekeep_llm import terminal
from zekeep_llm.planner import parse_plan


class TerminalNetworkTest(unittest.TestCase):
    def test_dry_run_uses_saved_exit_and_cleans_up_on_system_exit(self):
        planner = Mock()
        planner.plan.return_value = parse_plan(json.dumps({
            'summary': '查询状态',
            'steps': [{'tool': 'get_robot_status', 'arguments': {}}],
        }))
        process = Mock()
        process.poll.return_value = None

        def create_planner(**kwargs):
            self.assertEqual(os.environ['ZKEEP_LLM_PROXY_URL'], 'http://127.0.0.1:17897')
            return planner

        with tempfile.TemporaryDirectory() as directory:
            profile = Path(directory) / 'llm_network.json'
            profile.write_text(json.dumps({'direct_interface': 'enp2s0', 'proxy_port': 17897}))
            with patch.dict(os.environ, {
                'ZKEEP_LLM_NETWORK_FILE': str(profile), 'ZKEEP_LLM_PROXY_URL': '',
            }), patch('sys.argv', ['terminal', '--dry-run', '查询状态']), \
                    patch('zekeep_llm.web_network.listening', side_effect=[False, True]), \
                    patch('zekeep_llm.web_network.subprocess.Popen', return_value=process) as start, \
                    patch.object(terminal, 'DeepSeekPlanner', side_effect=create_planner), \
                    patch.object(terminal.rclpy, 'init') as ros_init:
                with self.assertRaises(SystemExit) as result:
                    terminal.main()
                self.assertEqual(result.exception.code, 0)
                planner.plan.assert_called_once_with('查询状态')
                start.assert_called_once()
                process.terminate.assert_called_once()
                process.wait.assert_called_once_with(timeout=3)
                self.assertEqual(os.environ['ZKEEP_LLM_PROXY_URL'], '')
                ros_init.assert_not_called()


if __name__ == '__main__':
    unittest.main()
