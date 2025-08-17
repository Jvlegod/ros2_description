from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_path = get_package_share_directory('robot_pkg')
    # ydlidar_ros2_dir = get_package_share_directory('ydlidar')
    ydlidar_ros2_dir = get_package_share_directory('hls_lfcd_lds_driver')

    declare_use_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Use simulation time (/clock) if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    set_global_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)

    urdf2tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_path, '/launch', '/robot.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    odom2tf = Node(
        package='robot_bringup',
        executable='odom2tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}] 
    )

    # microros_agent = Node(
    #     package='micro_ros_agent',
    #     executable='micro_ros_agent',
    #     arguments=['udp4','--port','8888'],
    #     output='screen'
    # )

    microros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['serial','-b','115200', '--dev', '/dev/ttyUSB1'],
        output='screen'
    )

    # ros_serial2wifi =  Node(
    #     package='ros_serial2wifi',
    #     executable='tcp_server',
    #     parameters=[{'serial_port': '/tmp/tty_laser'}],
    #     output='screen'
    # )

    # laser A
    # ydlidar = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [ydlidar_ros2_dir, '/launch', '/ydlidar_launch.py']),
    # )

    # ydlidar_delay = TimerAction(period=5.0, actions=[ydlidar])

    # laser B
    ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ydlidar_ros2_dir, '/launch', '/hlds_laser.launch.py']),
    )

    ydlidar_delay = TimerAction(period=5.0, actions=[ydlidar])

    return LaunchDescription([
        declare_use_sim,
        microros_agent,
        odom2tf,
        # ros_serial2wifi,
        ydlidar_delay,
        urdf2tf
    ])
