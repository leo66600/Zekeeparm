import os
import signal

from ament_index_python.packages import get_package_share_directory
from importlib.machinery import SourceFileLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown, matches_action
from launch.events.process import SignalProcess
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
    # ==============================================================================
    # 1. 动态启动参数声明 (Launch Arguments)
    # 将配置与代码解耦，允许在终端通过命令行动态修改运行状态
    # ==============================================================================
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
            model_arg,
            publish_worktable_arg,
            # 使用 OpaqueFunction 延迟执行节点装载，以便在运行时准确捕获上方声明的动态参数
            OpaqueFunction(function=_launch_setup),
        ]
    )


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    model = LaunchConfiguration("model").perform(context).strip().lower()
    if model != "sixaxis":
        raise ValueError(f"unsupported robot model: {model}")

    # ==============================================================================
    # 2. 核心参数统一构建 (MoveItConfigsBuilder)
    # 建立“唯一真理源”，确保大脑(MoveIt)和四肢(ros2_control)读取统一的物理与约束边界
    # ==============================================================================
    moveit_config = (
        MoveItConfigsBuilder("zekeep", package_name="zekeep_moveit_config")
        .robot_description(file_path="config/zekeep.urdf.xacro")        # 物理尺寸与连杆模型
        .robot_description_semantic(file_path="config/zekeep.srdf")     # 关节组划分与碰撞免检矩阵 (ACM)
        .robot_description_kinematics(file_path="config/kinematics.yaml") # 逆向运动学求解器配置
        .joint_limits(file_path="config/joint_limits.yaml")             # 电机速度/加速度物理限制
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"                  # 轨迹执行时的硬件控制器映射
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])                         # 加载 OMPL 等规划算法管道
        .to_moveit_configs()
    )
    moveit_params = build_moveit_parameters(moveit_config)

    # ==============================================================================
    # 3. 分布式核心节点组网 (Node Instantiation) 
    # ==============================================================================
    
    #  运动规划核心 (大脑)：负责碰撞检测、路径搜索和轨迹时间参数化
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params],
        # ==============================================================================
        # 4. 话题隔离与重映射 (Topic Remapping)
        # 仿真与真机使用不同的话题，避免误同时启动时混用关节反馈。
        # ==============================================================================
        remappings=[("/joint_states", "/zekeep_sim/joint_states")],
    )

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("zekeep_moveit_config"),
            "launch",
            LaunchConfiguration("rviz_config"),
        ]
    )
    
    #  可视化终端 (交互界面)：加载 MoveIt 插件，渲染 3D 运动状态
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")), # 根据动态参数决定是否启动
        parameters=[moveit_params],
        remappings=[("/joint_states", "/zekeep_sim/joint_states")],
    )

    #  状态解析层：基于 URDF 和实时关节角度，计算全尺寸 TF 坐标树
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
        remappings=[("/joint_states", "/zekeep_sim/joint_states")],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("zekeep_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    
    #  硬件驱动总线 (枢纽)：管理硬件接口通信
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
        remappings=[("/joint_states", "/zekeep_sim/joint_states")],
    )

    #  驱动层 Spawners：向 controller_manager 注册并激活具体的硬件控制器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", # 负责广播各电机当前读数
            "--controller-manager",
            "/controller_manager",
        ],
    )

    zekeep_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "zekeep_controller", # 负责接收轨迹命令并驱动机械臂本体
            "--controller-manager",
            "/controller_manager",
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller", # 负责驱动末端执行器(夹爪)
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # ==============================================================================
    # 统筹启动队列与进程安全级联保护
    # ==============================================================================
    nodes = [
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        zekeep_controller_spawner,
    ]
    nodes.append(gripper_controller_spawner)
    nodes.extend(
        [
            move_group_node,
            create_worktable_collision_node(  # 可选发布工作台防撞边界
                moveit_config.robot_description_semantic,
                LaunchConfiguration("use_rviz"),
                condition=IfCondition(LaunchConfiguration("publish_worktable")),
            ),
            rviz_node,
            
            # [安全机制] Fail-Safe 规则 1：
            # 若规划大脑(move_group)崩溃退出，强制向控制枢纽(ros2_control)发送中断信号(SIGINT)停机，防止暴走
            RegisterEventHandler(
                OnProcessExit(
                    target_action=move_group_node,
                    on_exit=[
                        EmitEvent(
                            event=SignalProcess(
                                signal_number=signal.SIGINT,
                                process_matcher=matches_action(ros2_control_node),
                            )
                        )
                    ],
                )
            ),
            
            # [安全机制] Fail-Safe 规则 2：
            # 若控制枢纽(ros2_control)停止工作，触发全局 Shutdown 关闭所有配套节点，安全退出
            RegisterEventHandler(
                OnProcessExit(
                    target_action=ros2_control_node,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason="ros2_control_node exited"))
                    ],
                )
            ),
        ]
    )
    return nodes
