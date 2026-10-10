"""Start the local AI planner; hardware startup requires motion_authorized."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression


def generate_launch_description():
    authorized = LaunchConfiguration('motion_authorized')
    return LaunchDescription([
        DeclareLaunchArgument('motion_authorized', default_value='false'),
        DeclareLaunchArgument('llm_proxy_url', default_value=EnvironmentVariable('ZKEEP_LLM_PROXY_URL', default_value='')),
        DeclareLaunchArgument('python_executable', default_value='/usr/bin/python3'),
        ExecuteProcess(cmd=[LaunchConfiguration('python_executable'), '-m', 'zekeep_llm.web_agent',
            PythonExpression(["'--motion-authorized' if '", authorized, "'.lower() == 'true' else '--port=8082'"])], additional_env={'ZKEEP_LLM_PROXY_URL': LaunchConfiguration('llm_proxy_url')}, output='screen'),
    ])
