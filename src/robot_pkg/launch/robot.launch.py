import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('robot_pkg')
    default_model_path = os.path.join(pkg_path, 'urdf', 'robot.xacro')

    default_world_path = os.path.join(pkg_path, 'worlds', 'nav_world.world')
    # default_world_path = os.path.join(pkg_path, 'worlds', 'test.world')
    
    rviz_config = os.path.join(pkg_path, 'config', 'display_config.rviz')
    bridge_launch_path = os.path.join(pkg_path, 'launch', 'bridge.launch.py')

    # Launch arguments
    declare_use_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time (/clock) if true'
    )

    declare_model = DeclareLaunchArgument(
        'model', default_value=default_model_path,
        description='Path to XACRO model'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='True',
        description='Whether to launch RViz2'
    )

    # Substitutions
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    rviz = LaunchConfiguration('rviz')

    # robot_description argument
    robot_description_content = Command(['xacro ', model])
    robot_description = ParameterValue(
        robot_description_content, value_type=str
    )

    # 1. start Gazebo Harmonic sim
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', default_world_path],
        output='screen'
    )

    # 2. publish robot_description -> TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # 3. start ros_gz_sim create spawn robot
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

    # 4. start ros_gz_bridge
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bridge_launch_path)
    )

    # 5. start rviz2
    rviz_node = GroupAction(
        condition=IfCondition(rviz),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim,
        declare_model,
        declare_rviz,
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        bridge_launch,
        rviz_node,
    ])
