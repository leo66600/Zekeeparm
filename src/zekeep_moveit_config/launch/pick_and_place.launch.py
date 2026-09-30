import os
from importlib.machinery import SourceFileLoader

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """启动固定点 Pick and Place 教学任务。

    sim:=true  启动 MoveIt 仿真栈、RViz、ros2_control fake/demo 控制器。
    sim:=false 启动连接真实 ZekeepController 的 MoveIt 栈，要求硬件控制节点已运行。
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim",
                default_value="true",
                description="true: demo simulation stack, false: real hardware MoveIt stack",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="true",
                description="Run task automatically after node startup",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    sim_value = LaunchConfiguration("sim").perform(context).strip().lower()
    sim_enabled = sim_value in ("true", "1", "yes", "on")

    package_share = get_package_share_directory("zekeep_moveit_config")
    moveit_launch = "demo.launch.py" if sim_enabled else "hardware.launch.py"

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", moveit_launch)
        ),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    params = os.path.join(package_share, "config", "pick_place_params.yaml")
    task_node = Node(
        package="zekeep_moveit_config",
        executable="pick_and_place_node",
        name="pick_and_place_node",
        output="screen",
        parameters=[
            params,
            {
                "sim": sim_enabled,
                "auto_start": LaunchConfiguration("auto_start"),
            },
        ],
    )

    return [bringup, task_node]
