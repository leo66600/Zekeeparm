import os

# 从 ament 索引库中导入获取功能包共享路径的工具函数
from ament_index_python.packages import get_package_share_directory

# 导入 ROS 2 Launch 核心描述符与动作组件
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

# 导入 Launch 条件判断工具（用于实现类似 if-else 的节点动态加载）
from launch.conditions import IfCondition, UnlessCondition

# 导入 Python 格式 Launch 文件的源解析器
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 导入 Launch 参数占位符与 ROS 2 节点/参数处理类
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    ROS 2 Launch 文件入口函数
    返回值: LaunchDescription 包含所有需启动的参数、子 Launch 文件及节点
    """

    # 第一部分：获取相关功能包的安装共享路径 (Share Directory)
    # 获取 MoveIt 配置包路径 (包含 demo.launch.py 与 hardware.launch.py)
    moveit_pkg_dir = get_package_share_directory('zekeep_moveit_config')
    # 获取机械臂 Bringup 配置包路径 (包含 carry_params.yaml 参数文件)
    bringup_pkg_dir = get_package_share_directory('zekeep_bringup')

    # 第二部分：声明对外暴露的启动参数 (支持终端命令行动态覆盖，如 sim:=true)
    # 1. 运行模式切换开关：true 为仿真环境，false 为真实硬件
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='模式切换: false 代表连接真机硬件栈，true 代表启动 MoveIt 虚拟仿真栈'
    )

    # 2. RViz 可视化开关：是否同步打开三维可视化界面
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否随同启动 RViz 2 界面'
    )

    # 3. 自动运行开关：启动后是否立刻执行九步搬运时序
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='true: 启动后自动开始搬运; false: 需手动调用 /carry/start 服务触发（默认）'
    )

    # 4. 轨迹仅预览开关：用于安全检查规划路径，不让机械臂实际动作
    preview_only_arg = DeclareLaunchArgument(
        'preview_only',
        default_value='false',
        description='true: 仅在 RViz 中规划并预览轨迹，不发送给电机执行'
    )

    # 5. 机械臂命名空间：统一管理真机话题与服务前缀 (如 /zekeep/joint_states)
    arm_ns_arg = DeclareLaunchArgument(
        'arm_namespace',
        default_value='zekeep',
        description='机械臂 ROS 命名空间'
    )

    # 第三部分：动态引入底层 MoveIt 运动规划栈 (根据 sim 参数二选一)
    # 【仿真栈】：当 sim 为 true 时条件触发，启动 demo.launch.py (虚拟控制器)
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg_dir, 'launch', 'demo.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz')  # 将当前 Launch 的 use_rviz 透传给子 Launch
        }.items(),
        condition=IfCondition(LaunchConfiguration('sim'))  # 条件判断：仅当 sim==true 时执行
    )

    # 【真机栈】：当 sim 为 false 时条件触发，启动 hardware.launch.py (连接实际驱动)
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg_dir, 'launch', 'hardware.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'arm_namespace': LaunchConfiguration('arm_namespace')  # 真机模式下需绑定命名空间
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('sim'))  # 条件判断：仅当 sim!=true 时执行
    )

    # 第四部分：配置搬运状态机控制节点 (加载 YAML 参数与动态运行时配置)
    # 拼装外部 YAML 配置文件路径 (包含抓取点 pick_pose、放置点 place_pose 等)
    params_file = os.path.join(bringup_pkg_dir, 'config', 'carry_params.yaml')


    carry_node = Node(
        package='zekeep_bringup',       # 节点所在的功能包名
        executable='carry_node',         # 编译出的可执行文件名称
        name='carry_node',               # 运行时的 ROS 2 节点名称
        output='screen',                 # 将状态机日志输出到终端控制台屏幕
        parameters=[
            params_file,                 # 1. 静态注入：载入 YAML 文件中的坐标、速度和约束参数
            {                            # 2. 动态覆盖：将命令行传入的配置强转为布尔类型后注入节点
                'sim': ParameterValue(LaunchConfiguration('sim'), value_type=bool),
                'auto_start': ParameterValue(LaunchConfiguration('auto_start'), value_type=bool),
                'preview_only': ParameterValue(LaunchConfiguration('preview_only'), value_type=bool),
            }
        ]
    )

    # 第五部分：组装并返回 LaunchDescription 容器
    return LaunchDescription([
        # 1. 注册暴露的所有命令行参数
        sim_arg,
        use_rviz_arg,
        auto_start_arg,
        preview_only_arg,
        arm_ns_arg,
        
        # 2. 启动条件选中的 MoveIt 底层栈 (仿真 或 真机)
        demo_launch,
        hardware_launch,
        
        # 3. 启动上层搬运业务状态机节点
        carry_node
    ])
