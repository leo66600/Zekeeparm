"""Exercise real process groups and signals without ROS, cameras, or motors."""
import importlib.util
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('launcher', Path(__file__).with_name('web_grasp_launcher.py'))
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)

CHILD = '''
import os, signal, sys, time
from pathlib import Path
log, role, mode = sys.argv[1:]
def record(text):
    with open(log, 'a') as output:
        output.write(text + '\\n')
count = 0
def interrupt(*_):
    global count
    count += 1
    record(role + ':interrupt')
    if role == 'driver' and mode == 'retry' and count == 1:
        record('driver:retained')
        return
    if role == 'driver':
        time.sleep(0.15)
    record(role + ':exit')
    raise SystemExit(0)
signal.signal(signal.SIGINT, interrupt)
record(role + ':ready')
if role == 'driver' and mode == 'fail':
    record('driver:failed')
    raise SystemExit(1)
while True:
    time.sleep(0.05)
'''


class LauncherTest(unittest.TestCase):
    def test_miniforge_default_custom_root_and_explicit_python(self):
        for mode in ('default', 'empty', 'custom', 'explicit'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                workspace = Path(directory)
                (workspace / 'install').mkdir()
                root = workspace / ('custom forge' if mode == 'custom' else 'miniforge3')
                python = root / 'envs/rebotarm/bin/python'
                if mode == 'explicit':
                    python = workspace / 'explicit python'
                python.parent.mkdir(parents=True, exist_ok=True)
                python.touch()
                python.chmod(0o700)
                env = {}
                if mode == 'empty':
                    env['ZKEEP_MINIFORGE_DIR'] = ''
                elif mode == 'custom':
                    env['ZKEEP_MINIFORGE_DIR'] = str(root)
                elif mode == 'explicit':
                    env.update(ZKEEP_MINIFORGE_DIR=str(workspace / 'missing'),
                               ZKEEP_VISION_PYTHON=str(python))
                with patch.object(launcher, '__file__', str(workspace / 'tools/web_grasp_launcher.py')), \
                        patch.object(launcher.Path, 'home', return_value=workspace), \
                        patch.object(launcher, 'preflight'), \
                        patch.object(launcher, 'environment', return_value={}), \
                        patch.object(launcher, 'supervise', return_value=0) as run, \
                        patch.dict(os.environ, env, clear=True), \
                        patch.object(sys, 'argv', ['launcher', '--preview']):
                    self.assertEqual(launcher.main(), 0)
                    self.assertEqual(launcher.shlex.split(run.call_args.args[1]['ZKEEP_VISION_PYTHON']),
                                     [str(python)])

    def test_occupied_web_port_is_rejected_before_launch(self):
        with tempfile.TemporaryDirectory() as directory, socket.socket() as server:
            workspace = Path(directory)
            (workspace / 'install').mkdir()
            (workspace / 'install/setup.bash').touch()
            server.bind(('127.0.0.1', 0))
            server.listen()
            with patch.dict(os.environ, {'PORT': str(server.getsockname()[1])}):
                with self.assertRaisesRegex(RuntimeError, '已占用'):
                    launcher.preflight(workspace)

    def test_missing_sdk_environment_keeps_web_launch(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            (workspace / 'tools').mkdir()
            (workspace / 'install').mkdir()
            with patch.object(launcher, '__file__', str(workspace / 'tools/web_grasp_launcher.py')), \
                    patch.object(launcher, 'preflight'), \
                    patch.object(launcher, 'environment', side_effect=[{}, RuntimeError('SDK missing')]), \
                    patch.object(launcher, 'supervise', return_value=0) as run, \
                    patch.dict(os.environ, {'ZKEEP_VISION_PYTHON': sys.executable}), \
                    patch.object(sys, 'argv', ['launcher']):
                self.assertEqual(launcher.main(), 0)
                self.assertIsNone(run.call_args.args[2])

    def test_defaults_authorize_supervised_control_but_never_auto_enable(self):
        for preview, ai_motion in ((False, False), (True, False), (False, True)):
            with self.subTest(preview=preview, ai_motion=ai_motion), tempfile.TemporaryDirectory() as directory:
                workspace = Path(directory)
                (workspace / 'tools').mkdir()
                (workspace / 'install').mkdir()
                with patch.object(launcher, '__file__', str(workspace / 'tools/web_grasp_launcher.py')), \
                        patch.object(launcher, 'preflight'), \
                        patch.object(launcher, 'environment', return_value={'VIRTUAL_ENV': '/sdk'}), \
                        patch.object(launcher, 'supervise', return_value=0) as run, \
                        patch.dict(os.environ, {'ZKEEP_VISION_PYTHON': sys.executable}), \
                        patch.object(sys, 'argv', ['launcher'] + (['--preview'] if preview else [])
                                     + (['--ai-motion'] if ai_motion else [])):
                    self.assertEqual(launcher.main(), 0)
                    web, env, driver, driver_env = run.call_args.args
                    self.assertIn(f'motion_authorized:={str(not preview).lower()}', web)
                    self.assertIn(f'ai_motion_authorized:={str(ai_motion).lower()}', web)
                    self.assertIn('ZKEEP_VISION_PYTHON', env)
                    if preview:
                        self.assertIsNone(driver)
                    else:
                        self.assertIn('auto_enable:=false', driver)
                        self.assertIn('python_executable:=/sdk/bin/python', driver)

    def test_preview_rejects_ai_motion_before_launch(self):
        with patch.object(sys, 'argv', ['launcher', '--preview', '--ai-motion']), \
                patch.object(launcher, 'supervise') as run, \
                patch('sys.stderr'):
            with self.assertRaises(SystemExit) as error:
                launcher.main()
            self.assertEqual(error.exception.code, 2)
            run.assert_not_called()

    def run_processes(self, mode):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            child, log = root / 'child.py', root / 'events.txt'
            child.write_text(CHILD)
            web = [sys.executable, str(child), str(log), 'web', mode]
            driver = [sys.executable, str(child), str(log), 'driver', mode]
            supervisor = root / 'supervisor.py'
            supervisor.write_text(
                'import os, runpy\n'
                f'module = runpy.run_path({launcher.__file__!r})\n'
                f'module["supervise"]({web!r}, os.environ.copy(), {driver!r}, os.environ.copy(), cwd={str(root)!r})\n')
            process = subprocess.Popen([sys.executable, str(supervisor)], start_new_session=True,
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

            def events():
                return log.read_text().splitlines() if log.exists() else []

            def wait_for(event):
                deadline = time.monotonic() + 8
                while event not in events():
                    if process.poll() is not None or time.monotonic() > deadline:
                        self.fail(f'{event} missing: {events()}')
                    time.sleep(0.02)

            try:
                wait_for('web:ready')
                wait_for('driver:ready')
                if mode == 'fail':
                    wait_for('driver:failed')
                    time.sleep(0.3)
                    self.assertIsNone(process.poll())
                    self.assertNotIn('web:interrupt', events())
                process.send_signal(signal.SIGINT)
                if mode == 'retry':
                    wait_for('driver:retained')
                    time.sleep(0.3)
                    self.assertIsNone(process.poll())
                    self.assertNotIn('web:interrupt', events())
                    process.send_signal(signal.SIGINT)
                output, _ = process.communicate(timeout=8)
                self.assertEqual(process.returncode, 0, output)
                records = events()
                if mode != 'fail':
                    self.assertLess(records.index('driver:exit'), records.index('web:interrupt'))
                self.assertIn('web:exit', records)
            finally:
                # This cleanup only applies to the disposable fake children.
                for entry in (str(log),):
                    for path in Path('/proc').glob('[0-9]*/cmdline'):
                        try:
                            arguments = path.read_bytes().decode(errors='replace').split('\0')
                            if entry in arguments and str(child) in arguments:
                                os.kill(int(path.parent.name), signal.SIGKILL)
                        except (OSError, ProcessLookupError):
                            pass
                if process.poll() is None:
                    process.kill()
                    process.wait()

    def test_driver_exits_before_web(self):
        self.run_processes('normal')

    def test_failed_homing_retains_web_and_allows_retry(self):
        self.run_processes('retry')

    def test_failed_driver_start_keeps_preview_alive(self):
        self.run_processes('fail')


if __name__ == '__main__':
    unittest.main()
