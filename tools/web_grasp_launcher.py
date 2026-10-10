"""Run the existing web and driver launches with isolated, ordered lifetimes."""
import argparse
import fcntl
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import threading


def environment(workspace, *, driver=False):
    setup = ('source tools/activate_ros.sh' if driver else
             'source /opt/ros/humble/setup.bash && source install/setup.bash')
    # Each launch gets its own Python/native-library paths, independent of the
    # caller's active Conda or SDK venv. Other settings (ROS domain, etc.) survive.
    result = subprocess.run([
        '/bin/bash', '-c',
        'unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH PYTHONNOUSERSITE; '
        + setup + ' && env -0',
    ], cwd=workspace, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=30)
    if result.returncode:
        raise RuntimeError(result.stderr.decode(errors='replace').strip() or '环境加载失败')
    env = dict(item.split('=', 1) for item in result.stdout.decode().split('\0') if '=' in item)
    env['ZKEEP_WS'] = str(workspace)
    env.pop('PYTHONHOME', None)
    if not driver:
        env.pop('PYTHONNOUSERSITE', None)
    return env


def send_interrupt(process):
    if process is not None and process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass


def supervise(web_command, web_env, driver_command=None, driver_env=None, *, cwd):
    requested = threading.Event()
    shutting_down = False
    web = driver = None

    def interrupt(_sig, _frame):
        requested.set()
        if shutting_down:
            # A second Ctrl+C retries the driver's graceful exit, never SIGKILL.
            send_interrupt(driver)

    previous = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        web = subprocess.Popen(web_command, cwd=cwd, env=web_env, start_new_session=True)
        if driver_command is not None and not requested.is_set():
            try:
                driver = subprocess.Popen(driver_command, cwd=cwd, env=driver_env, start_new_session=True)
            except OSError as exc:
                print(f'驱动启动失败，网页与相机预览继续：{exc}', flush=True)
        print('网页：http://127.0.0.1:3001；启动不自动使能或抓取。Ctrl+C 按顺序退出。', flush=True)
        reported = False
        while not requested.wait(0.2):
            if driver is not None and driver.poll() is not None and not reported:
                print('驱动已退出，网页与相机预览继续；实机抓取不可用。', flush=True)
                reported = True
            if web.poll() is not None:
                break
    finally:
        shutting_down = True
        if driver is not None and driver.poll() is None:
            print('正在安全退出驱动，保留 MoveIt。回零失败时处理故障后再次按 Ctrl+C 重试。', flush=True)
            send_interrupt(driver)
            # The existing controller retains holding torque on failed homing.
            # Its launch has infinite escalation timeouts; do not impose one here.
            driver.wait()
        send_interrupt(web)
        if web is not None:
            web.wait()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return web.returncode or 0


def preflight(workspace):
    if not (workspace / 'install/setup.bash').is_file():
        raise RuntimeError('工作区尚未构建，请先按网页 README 构建。')
    for port in (int(os.environ.get('PORT', '3001')), 9090, 8082):
        with socket.socket() as connection:
            connection.settimeout(0.2)
            if connection.connect_ex(('127.0.0.1', port)) == 0:
                raise RuntimeError(f'端口 {port} 已占用，请先关闭旧网页或 AI 后端进程。')
    for entry in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            arguments = entry.read_bytes().decode(errors='replace').split('\0')
        except (OSError, PermissionError):
            continue
        if any(argument.startswith(str(workspace / 'install') + '/')
               and Path(argument).name in ('ZekeepController', 'web_tasks', 'web_vision')
               for argument in arguments):
            raise RuntimeError('已有旧驱动或视觉任务进程，请先按原退出顺序关闭旧流程。')


def main():
    parser = argparse.ArgumentParser(description='一个终端启动网页、原视觉、MoveIt 和真机驱动。')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--preview', action='store_true', help='只预览，不启动驱动、不授权运动')
    mode.add_argument('--ai-motion', action='store_true', help='允许 AI 计划经人工确认后执行实机动作')
    parser.add_argument('--channel', default='', help='串口；默认沿用硬件配置')
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[1]
    try:
        preflight(workspace)
        # Keep the lock in the already-ignored install directory for this workspace.
        with (workspace / 'install/.web_grasp.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            env = environment(workspace)
            miniforge_dir = Path(os.environ.get('ZKEEP_MINIFORGE_DIR') or str(Path.home() / 'miniforge3')).expanduser()
            vision_python = Path(os.environ.get('ZKEEP_VISION_PYTHON',
                str(miniforge_dir / 'envs/rebotarm/bin/python'))).expanduser().resolve()
            if not vision_python.is_file() or not os.access(vision_python, os.X_OK):
                raise RuntimeError(f'视觉 Python 不可用：{vision_python}')
            # ROS launch interprets prefix as shell tokens; quote paths with spaces.
            env['ZKEEP_VISION_PYTHON'] = shlex.quote(str(vision_python))
            ros2 = ['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch', 'zekeep_bringup']
            web_command = ros2 + ['web_grasp.launch.py',
                f'motion_authorized:={str(not args.preview).lower()}',
                f'ai_motion_authorized:={str(args.ai_motion).lower()}']
            driver_command = driver_env = None
            if not args.preview:
                try:
                    driver_env = environment(workspace, driver=True)
                    driver_python = Path(driver_env['VIRTUAL_ENV']) / 'bin/python'
                    driver_command = ros2 + ['driver.launch.py', 'auto_enable:=false',
                        'python_executable:=' + shlex.quote(str(driver_python))]
                    if args.channel:
                        driver_command.append('channel:=' + args.channel)
                except (RuntimeError, subprocess.TimeoutExpired) as exc:
                    print(f'驱动环境不可用，继续启动网页与预览：{exc}', flush=True)
            return supervise(web_command, env, driver_command, driver_env, cwd=workspace)
    except (OSError, RuntimeError, subprocess.TimeoutExpired, ValueError) as exc:
        print(f'启动失败：{exc}', flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
