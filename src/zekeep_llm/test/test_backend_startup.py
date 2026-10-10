"""Check backend startup and confirmation boundaries without motor access."""

import os
import fcntl
from pathlib import Path
from subprocess import Popen as real_popen
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from zekeep_llm.planner import Plan, Step
from zekeep_llm.robot import RobotTools
from zekeep_llm.startup import RobotStartup
from zekeep_llm.terminal import _run_task
from zekeep_llm.web_agent import AgentSession


class BackendStartupTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.workspace = Path(self.directory.name)
        (self.workspace/'tools').mkdir()
        (self.workspace/'tools/activate_ros.sh').touch()
        python = self.workspace/'.venv-ros/bin/python'
        python.parent.mkdir(parents=True)
        python.touch()
        python.chmod(0o700)
        self.ready = False
        self.node = Mock()
        self.node.context.ok.return_value = True
        def spin(predicate, timeout, message):
            if timeout == 30:
                self.ready = True
                self.assertTrue(predicate())
        self.node._spin_until.side_effect = spin
        self.runtime = RobotStartup(self.node, 'zekeep')
        self.patches = [
            patch('zekeep_llm.startup.Path.home', return_value=self.workspace),
            patch('zekeep_llm.startup.Path.glob', return_value=[]),
            patch('zekeep_llm.startup.get_package_prefix', return_value=str(self.workspace/'install/zekeep_llm')),
            patch.dict(os.environ, {'ZKEEP_WS': str(self.workspace),
                                    'ZKEEP_ROS_VENV': str(self.workspace/'.venv-ros'),
                                    'ZKEEP_LLM_API_KEY': 'test-secret'}),
            patch('zekeep_llm.startup.subprocess.Popen'),
        ]
        self.start = [item.start() for item in self.patches][-1]
        self.process = self.start.return_value
        self.process.poll.return_value = None

    def tearDown(self):
        for item in reversed(self.patches):
            item.stop()
        self.directory.cleanup()

    def test_missing_backends_use_fixed_launches_without_enabling_and_remain_running(self):
        for backend in ('moveit', 'driver'):
            self.ready = False
            self.runtime.ensure(backend, lambda: self.ready)
        self.assertEqual(self.start.call_count, 2)
        moveit, driver = self.start.call_args_list
        self.assertIn('hardware.launch.py', moveit.args[0])
        self.assertIn('publish_worktable:=true', moveit.args[0])
        self.assertIn('driver.launch.py', driver.args[0])
        self.assertIn('auto_enable:=false', driver.args[0])
        self.assertIn('python_executable:='+str(self.workspace/'.venv-ros/bin/python'), driver.args[0])
        for call in (moveit, driver):
            self.assertTrue(call.kwargs['start_new_session'])
            self.assertEqual(len(call.kwargs['pass_fds']), 1)
            self.assertNotIn('ZKEEP_LLM_API_KEY', call.kwargs['env'])
        self.process.send_signal.assert_not_called()
        self.process.kill.assert_not_called()

    def test_existing_services_do_not_start_any_process(self):
        self.runtime.ensure('moveit', lambda: True)
        self.runtime.ensure('driver', lambda: True)
        self.start.assert_not_called()

    def test_existing_unreachable_controller_blocks_duplicate_startup(self):
        entry = self.workspace/'controller-cmdline'
        entry.write_bytes(b'/usr/bin/python3\0/workspace/install/zekeepcontroller/lib/zekeepcontroller/ZekeepController\0')
        with patch('zekeep_llm.startup.Path.glob', return_value=[entry]):
            with self.assertRaisesRegex(RuntimeError, '不重复启动'):
                self.runtime.ensure('driver', lambda: False)
        self.start.assert_not_called()

    def test_startup_failure_preserves_logs_and_never_kills_driver(self):
        self.process.poll.return_value = 1
        def spin(predicate, timeout, message):
            if timeout == 30:
                (self.workspace/'.local/share/zekeep/llm_driver.log').write_text('serial port busy')
                predicate()
        self.node._spin_until.side_effect = spin
        with self.assertRaisesRegex(RuntimeError, 'serial port busy'):
            self.runtime.ensure('driver', lambda: False)
        self.process.kill.assert_not_called()

    def test_timeout_does_not_kill_or_restart_a_driver_still_initializing(self):
        def spin(predicate, timeout, message):
            if timeout == 30:
                self.assertFalse(predicate())
                raise TimeoutError(message)
        self.node._spin_until.side_effect = spin
        for _ in range(2):
            with self.assertRaises(TimeoutError):
                self.runtime.ensure('driver', lambda: False)
        self.start.assert_called_once()
        self.process.send_signal.assert_not_called()
        self.process.kill.assert_not_called()

    def test_unknown_backend_is_rejected_before_any_launch(self):
        with self.assertRaises(ValueError):
            self.runtime.ensure('shell', lambda: False)
        self.start.assert_not_called()

    def test_closed_ros_context_prevents_hardware_startup(self):
        self.node.context.ok.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'ROS 已关闭'):
            self.runtime.ensure('driver', lambda: False)
        self.start.assert_not_called()

    def test_detached_child_keeps_startup_lock_after_parent_closes_it(self):
        argv = ['/bin/bash', '-c', 'shift; exec "$@"', 'test-startup', str(self.workspace),
                sys.executable, '-c', 'import signal; signal.pause()']
        process = None
        try:
            with patch.object(self.runtime, 'command', return_value=(argv, self.workspace)), \
                    patch('zekeep_llm.startup.subprocess.Popen', side_effect=real_popen):
                self.runtime.ensure('driver', lambda: self.ready)
            process = self.runtime.processes['driver']
            with (self.workspace/'.local/share/zekeep/llm_driver.lock').open('a+b') as lock:
                with self.assertRaises(BlockingIOError):
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        finally:
            if process is not None:
                process.terminate()
                process.wait(timeout=3)
        with (self.workspace/'.local/share/zekeep/llm_driver.lock').open('a+b') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)


class StartupBoundaryTest(unittest.TestCase):
    def test_prepare_starts_moveit_before_driver_and_does_not_start_for_stop_or_diagnosis(self):
        calls = []
        robot = SimpleNamespace(_autostart=True,
            _startup=SimpleNamespace(ensure=lambda name, ready: calls.append(name)),
            _validity=SimpleNamespace(service_is_ready=lambda: False),
            _triggers={'enable_robot': SimpleNamespace(service_is_ready=lambda: False)},
            _telemetry_is_fresh=lambda: True, _spin_until=lambda *args: calls.append('feedback'))
        RobotTools.prepare(robot, 'move_joints')
        self.assertEqual(calls, ['moveit', 'driver', 'feedback'])
        calls.clear()
        RobotTools.prepare(robot, 'get_robot_status')
        self.assertEqual(calls, ['driver', 'feedback'])
        calls.clear()
        for tool in ('stop', 'disable_robot', 'diagnose_ros', 'gravity_compensation_stop', 'record_clear', 'record_stop'):
            RobotTools.prepare(robot, tool)
        self.assertEqual(calls, [])
        robot._autostart = False
        RobotTools.prepare(robot, 'move_joints')
        self.assertEqual(calls, [])

    def test_cancelled_terminal_confirmation_does_not_start_hardware(self):
        planner = SimpleNamespace(plan=lambda _: Plan('使能', (Step('enable_robot'),)))
        robot = Mock()
        with patch('builtins.input', return_value='no'):
            self.assertFalse(_run_task(planner, robot, '使能', False))
        robot.execute.assert_not_called()
        robot.prepare.assert_not_called()

    def test_web_prepares_dependencies_before_lease_only_after_authorization(self):
        calls = []
        planner = SimpleNamespace(plan=lambda _: Plan('回预备位', (Step('return_ready'),)))
        robot = SimpleNamespace(prepare=lambda tool: calls.append(('prepare', tool)),
                                execute=lambda tool, arguments: calls.append(tool) or 'done')
        session = AgentSession(planner, robot, motion_authorized=True,
                               lease=lambda acquire: calls.append(acquire))
        plan = session.plan('回预备位')
        session.execute(plan['plan_id'], True)
        self.assertEqual(calls, [('prepare', 'return_ready'), True, 'return_ready', False])
        calls.clear()
        session.motion_authorized = False
        plan = session.plan('回预备位')
        with self.assertRaises(PermissionError):
            session.execute(plan['plan_id'], True)
        self.assertEqual(calls, [])

    def test_cancellation_during_startup_does_not_acquire_a_motion_lease(self):
        calls = []
        planner = SimpleNamespace(plan=lambda _: Plan('回预备位', (Step('return_ready'),)))
        robot = SimpleNamespace(execute=lambda *args: calls.append('execute'))
        session = AgentSession(planner, robot, motion_authorized=True,
                               lease=lambda acquire: calls.append(acquire))
        robot.prepare = lambda tool: session.cancelled.set()
        plan = session.plan('回预备位')
        with self.assertRaisesRegex(RuntimeError, 'cancelled during backend startup'):
            session.execute(plan['plan_id'], True)
        self.assertEqual(calls, [])


if __name__ == '__main__':
    unittest.main()
