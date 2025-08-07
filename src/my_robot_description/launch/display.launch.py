# rviz2 显示代码.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_description')
    default_model_path = os.path.join(pkg_path, 'urdf', 'robot.xacro')
    rviz_config_path = os.path.join(pkg_path, 'config', 'display_config.rviz')

    declare_model_arg_path = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='Absolute path to robot urdf or xacro file'
    )

    robot_description_result = Command(['xacro ', LaunchConfiguration('model')])
    robot_description_val = ParameterValue(robot_description_result, value_type=str)

    return LaunchDescription([
        declare_model_arg_path,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_val}],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
        )
    ])
