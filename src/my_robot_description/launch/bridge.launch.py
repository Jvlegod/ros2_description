# 桥接代码.
from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'config',
        'bridge_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                "--ros-args",
                "-p",
                f"config_file:={config_path}",
            ],
            output='screen'
        )
    ])