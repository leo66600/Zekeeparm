import os
from importlib.machinery import SourceFileLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

moveit_launch_helpers = SourceFileLoader(
    "moveit_launch_helpers",
    os.path.join(os.path.dirname(__file__), "moveit_launch_helpers.py"),
).load_module()
build_moveit_parameters = moveit_launch_helpers.build_moveit_parameters
create_worktable_collision_node = (
    moveit_launch_helpers.create_worktable_collision_node
)


def generate_launch_description():
    """Launch MoveIt against an already-running Zekeep controller."""
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file in zekeep_moveit_config/launch",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz with the MoveIt motion planning plugin",
    )
    arm_namespace_arg = DeclareLaunchArgument(
        "arm_namespace",
        default_value="zekeep",
        description="Namespace used by an already-running ZekeepController",
    )
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="sixaxis",
        description="Robot model to load: sixaxis",
    )
    publish_worktable_arg = DeclareLaunchArgument(
        "publish_worktable",
        default_value="false",
        description="Publish the fixed worktable collision object",
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            use_rviz_arg,
            arm_namespace_arg,
            model_arg,
            publish_worktable_arg,
            DeclareLaunchArgument("shutdown_on_moveit_exit", default_value="true",
                                  description="Shut down this launch if move_group exits"),
            OpaqueFunction(function=_launch_setup),
        ]
    )


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    model = LaunchConfiguration("model").perform(context).strip().lower()
    if model != "sixaxis":
        raise ValueError(f"unsupported robot model: {model}")
    arm_namespace = (LaunchConfiguration("arm_namespace").perform(context) or "zekeep").strip("/")

    moveit_config = (
        MoveItConfigsBuilder("zekeep", package_name="zekeep_moveit_config")
        .robot_description(file_path="config/zekeep.urdf.xacro")
        .robot_description_semantic(file_path="config/zekeep.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_hardware_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    moveit_params = build_moveit_parameters(moveit_config)
    controllers = moveit_params["moveit_simple_controller_manager"]
    controllers[arm_namespace] = controllers.pop("zekeep")
    controllers["controller_names"] = [arm_namespace]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params],
        remappings=[("/joint_states", f"/{arm_namespace}/joint_states")],
    )

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("zekeep_moveit_config"),
            "launch",
            LaunchConfiguration("rviz_config"),
        ]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        parameters=[moveit_params],
        remappings=[("/joint_states", f"/{arm_namespace}/joint_states")],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
        remappings=[("/joint_states", f"/{arm_namespace}/joint_states")],
    )

    return [
        robot_state_publisher_node,
        move_group_node,
        create_worktable_collision_node(
            moveit_config.robot_description_semantic,
            LaunchConfiguration("use_rviz"),
            condition=IfCondition(LaunchConfiguration("publish_worktable")),
        ),
        rviz_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=move_group_node,
                on_exit=[
                    EmitEvent(event=Shutdown(reason="move_group exited"))
                ],
            ),
            condition=IfCondition(LaunchConfiguration("shutdown_on_moveit_exit")),
        ),
    ]
