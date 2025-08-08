import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, GroupAction, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessExit
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
    declare_model = DeclareLaunchArgument(
        'model', default_value=default_model_path,
        description='Path to XACRO model'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='True',
        description='Whether to launch RViz2'
    )

    # Substitutions
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
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # 3. start ros_gz_sim create spawn robot
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.3'
        ],
        output='screen'
    )

    spawn_robot = TimerAction(
        period=2.0,
        actions=[spawn_robot_node]
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
                output='screen'
            )
        ]
    )

    load_joint_state_controller_spawner = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', 'joint_state_broadcaster',
            '--set-state', 'active',
        ],
        output='screen'
    )

    load_effort_controller_spawner = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', 'effort_controller',
            '--set-state', 'active',
        ],
        output='screen'
    )

    # 6. Register event: after spawn, run controller_spawner
    register_load_joint_state_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[load_joint_state_controller_spawner]
        )
    )

    register_load_effort_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller_spawner,
            on_exit=[load_effort_controller_spawner]
        )
    )

    return LaunchDescription([
        declare_model,
        declare_rviz,
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        bridge_launch,
        rviz_node,
        register_load_joint_state_spawner,
        register_load_effort_spawner
    ])
