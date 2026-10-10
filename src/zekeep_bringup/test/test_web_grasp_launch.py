"""Expand the real launch stack without starting processes or touching hardware."""
import runpy
import os
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml
from launch import LaunchContext
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.utilities import perform_substitutions


def expand_stack(launch_file="web_grasp.launch.py", **arguments):
    context = LaunchContext()
    context.launch_configurations.update(arguments)
    processes = {}

    def capture(process, context):
        command = [perform_substitutions(context, part) for part in process.cmd]
        parameters = {}
        for index, part in enumerate(command[:-1]):
            if part == "--params-file":
                # launch writes Python tuple tags for ROS parameter arrays.
                for node in yaml.full_load(Path(command[index + 1]).read_text()).values():
                    parameters.update(node.get("ros__parameters", {}))
        processes[Path(command[0]).name] = (
            command, parameters,
            perform_substitutions(context, process.process_description.prefix),
        )

    def visit(entity):
        if isinstance(entity, RegisterEventHandler):
            processes['moveit_exit_shutdown'] = entity.condition.evaluate(context) if entity.condition else True
            return
        for child in entity.visit(context) or []:
            visit(child)

    path = Path(__file__).resolve().parents[1] / "launch" / launch_file
    with patch.object(ExecuteProcess, "execute", capture):
        visit(runpy.run_path(str(path))["generate_launch_description"]())
    return processes


class WebGraspLaunchTest(unittest.TestCase):
    def test_default_perception_root_uses_renamed_project(self):
        workspace = Path(__file__).resolve().parents[3]
        root = workspace / 'src/zekeep_grasp'
        self.assertTrue((root / 'scripts/main.py').is_file())
        self.assertFalse((workspace / 'src/rebot_grasp').exists())
        with patch.dict(os.environ, {'ZKEEP_WS': str(workspace)}):
            processes = expand_stack()
        for name in ('web_tasks', 'web_vision'):
            self.assertEqual(processes[name][1]['perception_root'], str(root))

    def test_driver_uses_explicit_sdk_python_and_retains_disabled_default(self):
        processes = expand_stack("driver.launch.py", python_executable="/sdk/bin/python")
        controller = processes['ZekeepController']
        self.assertEqual(controller[2], '/sdk/bin/python')
        self.assertFalse(controller[1]['auto_enable'])
        processes = expand_stack("driver.launch.py")
        self.assertEqual(processes['ZekeepController'][2], '')

    def test_preview_defaults_and_grasp_authorization_remain_separate_from_ai(self):
        for authorized in ("false", "true"):
            with self.subTest(authorized=authorized):
                processes = expand_stack(
                    motion_authorized=authorized, grasp_geometry_confirmed=authorized,
                    perception_root="/tmp/perception", python_executable="/camera/python",
                    grasp_rpy_rad="[1.0, 2.0, 3.0]", object_size_m="[0.02, 0.03, 0.04]",
                )
                self.assertNotIn("ZekeepController", processes)
                self.assertIn("move_group", processes)
                self.assertIn("robot_state_publisher", processes)
                self.assertIn('worktable_collision', processes)
                self.assertFalse(processes['moveit_exit_shutdown'])
                self.assertIn("node", processes)
                self.assertNotIn("rviz2", processes)
                task = processes["web_tasks"]
                self.assertEqual(task[2], "/camera/python")
                self.assertEqual(task[1]["motion_authorized"], authorized == "true")
                self.assertEqual(task[1]["grasp_geometry_confirmed"], authorized == "true")
                self.assertEqual(task[1]["perception_root"], "/tmp/perception")
                self.assertEqual(task[1]['grasp_backend'], 'yolo-graspnet')
                self.assertEqual(list(task[1]["grasp_rpy_rad"]), [1.0, 2.0, 3.0])
                self.assertEqual(list(task[1]["object_size_m"]), [0.02, 0.03, 0.04])
                self.assertEqual(processes["web_vision"][2], "/camera/python")
                self.assertEqual(processes["web_vision"][1]["perception_root"], "/tmp/perception")
                self.assertEqual(processes["web_vision"][1]['grasp_backend'], 'yolo-graspnet')
                ai = processes["python3"][0]
                self.assertEqual(ai[0], "/usr/bin/python3")
                self.assertIn("zekeep_llm.web_agent", ai)
                self.assertNotIn("--motion-authorized", ai)

    def test_optional_services_can_be_disabled(self):
        processes = expand_stack(start_ai="false", start_moveit="false", start_vision="false")
        self.assertNotIn("move_group", processes)
        self.assertNotIn('worktable_collision', processes)
        self.assertNotIn("web_vision", processes)
        self.assertNotIn("python3", processes)
        self.assertIn("web_tasks", processes)
        self.assertFalse(processes["web_tasks"][1]["motion_authorized"])

    def test_explicit_ai_grant_does_not_authorize_visual_tasks(self):
        processes = expand_stack(ai_motion_authorized='true')
        self.assertIn('--motion-authorized', processes['python3'][0])
        self.assertEqual(processes['python3'][0][0], '/usr/bin/python3')
        self.assertFalse(processes['web_tasks'][1]['motion_authorized'])


if __name__ == "__main__":
    unittest.main()
