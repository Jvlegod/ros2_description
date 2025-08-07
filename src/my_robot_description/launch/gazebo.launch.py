# gazebo 显示代码.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_description')
    default_model_path = os.path.join(pkg_path, 'urdf', 'robot.xacro')
    default_world_path = os.path.join(pkg_path, 'worlds', 'nav_world.sdf')

    declare_model = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Path to URDF/XACRO model'
    )

    robot_description_content = Command(['xacro ', LaunchConfiguration('model')])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', default_world_path],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'my_robot',
                    '-topic', 'robot_description',
                    '-x', '0', '-y', '0', '-z', '0.3'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        declare_model,
        robot_state_publisher,
        gz_sim,
        spawn_robot,
    ])
