"""Start fixed ROS backends; keep hardware and collision checks alive on exit."""

import fcntl
import os
from pathlib import Path
import shlex
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix, PackageNotFoundError


class RobotStartup:
    def __init__(self, node, namespace):
        self.node, self.namespace = node, namespace
        self.processes = {}
        self.lock = threading.Lock()

    def ensure(self, backend, ready):
        if backend not in ('driver', 'moveit'):
            raise ValueError('unsupported robot backend')
        with self.lock:
            if not self.node.context.ok():
                raise RuntimeError('ROS 已关闭；不启动后台进程')
            discovery = time.monotonic() + 1
            self.node._spin_until(lambda: ready() or time.monotonic() >= discovery, 2, 'ROS 服务发现超时')
            if not self.node.context.ok():
                raise RuntimeError('ROS 已关闭；不启动后台进程')
            if ready():
                return
            directory = Path.home()/'.local/share/zekeep'
            directory.mkdir(parents=True, exist_ok=True)
            log_path = directory/f'llm_{backend}.log'
            process = self.processes.get(backend)
            if process is None or process.poll() is not None:
                with (directory/f'llm_{backend}.lock').open('a+b') as lock:
                    try:
                        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    except BlockingIOError:
                        pass  # Another assistant owns this backend's startup.
                    else:
                        executable = 'ZekeepController' if backend == 'driver' else 'move_group'
                        for entry in Path('/proc').glob('[0-9]*/cmdline'):
                            try:
                                arguments = entry.read_bytes().decode(errors='replace').split('\0')
                            except OSError:
                                continue
                            launches = (('zekeep_bringup', ('driver.launch.py', 'bringup.launch.py'))
                                        if backend == 'driver' else ('zekeep_moveit_config', ('hardware.launch.py',)))
                            if (any(Path(value).name == executable for value in arguments if value)
                                    or (launches[0] in arguments and any(name in arguments for name in launches[1]))):
                                raise RuntimeError(f'已有 {executable} 进程但服务不可用；检查 ROS_DOMAIN_ID/命名空间，不重复启动')
                        argv, workspace = self.command(backend)
                        env = os.environ.copy()
                        for name in ('ZKEEP_LLM_API_KEY', 'ZKEEP_VLM_API_KEY'):
                            env.pop(name, None)
                        with log_path.open('ab', buffering=0) as log:
                            process = subprocess.Popen(argv, cwd=workspace, env=env, stdin=subprocess.DEVNULL,
                                stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                                pass_fds=(lock.fileno(),))
                        self.processes[backend] = process
                        self.node.get_logger().info(f'自动启动 {backend}；日志 {log_path}；退出助手后保留此进程')

            def available():
                if process is not None and process.poll() is not None:
                    with log_path.open('rb') as log:
                        log.seek(0, os.SEEK_END)
                        log.seek(max(0, log.tell()-4096))
                        detail = log.read().decode(errors='replace').strip()
                    raise RuntimeError(f'{backend} 启动失败: {detail}')
                return ready()

            # Never kill a driver on failure: its exit may move an enabled arm.
            self.node._spin_until(available, 30, f'{backend} 服务未就绪；保留进程，检查 {log_path}')
            if not ready():
                raise RuntimeError(f'{backend} 初始化被中断；未执行后续动作')

    def command(self, backend):
        try:
            prefix = Path(get_package_prefix('zekeep_llm'))
        except PackageNotFoundError as exc:
            raise RuntimeError('请构建并 source zekeep_llm 工作区') from exc
        workspace = os.environ.get('ZKEEP_WS')
        if not workspace:
            workspace = next((p for p in prefix.parents if (p/'tools/activate_ros.sh').is_file()), None)
        if workspace is None:
            raise RuntimeError('无法定位机械臂工作区；请设置 ZKEEP_WS')
        workspace = Path(workspace).expanduser().resolve()
        reset = 'set -e; unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH PYTHONNOUSERSITE; '
        ros2 = ['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch']
        if backend == 'driver':
            python = Path(os.environ.get('ZKEEP_ROS_VENV', str(workspace/'.venv-ros'))).expanduser()/'bin/python'
            if not python.is_file() or not os.access(python, os.X_OK):
                raise RuntimeError('驱动 Python 不可用；检查 ZKEEP_ROS_VENV 和 tools/activate_ros.sh')
            setup = 'source "$1/tools/activate_ros.sh"; '
            launch = ros2 + ['zekeep_bringup', 'driver.launch.py', 'auto_enable:=false',
                f'arm_namespace:={self.namespace}', 'python_executable:='+shlex.quote(str(python))]
        else:
            setup = 'source /opt/ros/humble/setup.bash; source "$1/install/setup.bash"; '
            launch = ros2 + ['zekeep_moveit_config', 'hardware.launch.py',
                f'arm_namespace:={self.namespace}', 'use_rviz:=false', 'publish_worktable:=true']
        return ['/bin/bash', '-c', reset+setup+'shift; exec "$@"', 'zekeep-startup', str(workspace), *launch], workspace
