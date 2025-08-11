from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_path = get_package_share_directory('robot_pkg')
    ydlidar_ros2_dir = get_package_share_directory('ydlidar')

    urdf2tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_path, '/launch', '/robot.launch.py']),
    )

    odom2tf = Node(
        package='robot_bringup',
        executable='odom2tf',
        output='screen'
    )

    microros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['udp4','--port','8888'],
        output='screen'
    )

    ros_serial2wifi =  Node(
        package='ros_serial2wifi',
        executable='tcp_server',
        parameters=[{'serial_port': '/tmp/tty_laser'}],
        output='screen'
    )

    ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ydlidar_ros2_dir, '/launch', '/ydlidar_launch.py']),
    )

    ydlidar_delay = TimerAction(period=5.0, actions=[ydlidar])

    return LaunchDescription([
        microros_agent,
        ros_serial2wifi,
        odom2tf,
        ydlidar_delay,
        urdf2tf
    ])
