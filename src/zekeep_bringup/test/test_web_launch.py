"""Run with: source /opt/ros/humble/setup.bash && python3 test/test_web_launch.py."""
import fnmatch
from pathlib import Path
import runpy

import yaml


launch_file = Path(__file__).resolve().parents[1] / 'launch' / 'web.launch.py'
description = runpy.run_path(str(launch_file))['generate_launch_description']()
from launch.actions import IncludeLaunchDescription
bridge = next(action for action in description.entities if isinstance(action, IncludeLaunchDescription))
arguments = dict(bridge.launch_arguments)

# Match the XML launch frontend's YAML conversion, before rclpy type checking.
for name in ('topics_glob', 'services_glob', 'params_glob'):
    value = yaml.safe_load(arguments[name])
    assert isinstance(value, str), f'{name} must remain a ROS string parameter'
    patterns = value.strip('[]').split(',')
    allowed = '/zekeep/joint_states' if name == 'topics_glob' else '/zekeep/enable'
    assert any(fnmatch.fnmatch(allowed, pattern) for pattern in patterns) == (name != 'params_glob')
    assert not any(fnmatch.fnmatch('/unrelated/resource', pattern) for pattern in patterns)

assert yaml.safe_load(arguments['call_services_in_new_thread']) is True
assert yaml.safe_load(arguments['send_action_goals_in_new_thread']) is True
print('PASS: web launch parameter types, allowlists, and nonblocking ROS calls')
