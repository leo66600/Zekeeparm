"""Check XML parameter conversion and the installed rosbridge glob parser."""

import runpy
import unittest
from fnmatch import fnmatch
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.substitutions import TextSubstitution
from launch_ros.utilities import evaluate_parameters, normalize_parameters


class WebLaunchGlobsTest(unittest.TestCase):
    def test_globs_remain_strings_and_enforce_the_allowlist(self):
        path = Path(__file__).resolve().parents[1] / "launch" / "web.launch.py"
        description = runpy.run_path(str(path))["generate_launch_description"]()
        bridge = next(
            action for action in description.entities
            if isinstance(action, IncludeLaunchDescription)
        )
        context = LaunchContext()
        arguments = {
            name: value
            for name, value in bridge.launch_arguments
            if name.endswith("_glob")
        }
        parameters = evaluate_parameters(context, normalize_parameters([{
            name: TextSubstitution(text=value) for name, value in arguments.items()
        }]))[0]
        parser_path = (
            Path(get_package_prefix("rosbridge_server"))
            / "lib" / "rosbridge_server" / "rosbridge_websocket.py"
        )
        parse = runpy.run_path(str(parser_path))["parse_glob_string"]
        for value in parameters.values():
            self.assertIsInstance(value, str)
        topics = parse(parameters["topics_glob"])
        services = parse(parameters["services_glob"])
        for topic in (
            "/zekeep/joint_states", "/zekeep/mujoco/object_states",
            "/camera/color/image_raw", "/gemini305g/color/image_raw", "/tf_static",
        ):
            self.assertTrue(any(fnmatch(topic, glob) for glob in topics), topic)
        for service in (
            "/zekeep/stop", "/zekeep/gravity_compensation/status",
            "/rosapi/topics", "/ZekeepController/get_parameters",
            "/compute_ik",
        ):
            self.assertTrue(any(fnmatch(service, glob) for glob in services), service)
        for patterns in (topics, services, parse(parameters["params_glob"])):
            self.assertFalse(any(fnmatch("/other/private", glob) for glob in patterns))
        self.assertEqual(parse(parameters["params_glob"]), ["/__web_parameters_disabled__"])


if __name__ == "__main__":
    unittest.main()
