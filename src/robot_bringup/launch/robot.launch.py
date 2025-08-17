import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('robot_pkg')
    default_model_path = os.path.join(pkg_path, 'urdf', 'robot.urdf')
    
    rviz_config = os.path.join(pkg_path, 'config', 'display_config.rviz')

    # Launch arguments
    declare_use_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Use simulation time (/clock) if true'
    )

    declare_model = DeclareLaunchArgument(
        'model', default_value=default_model_path,
        description='Path to XACRO model'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='False',
        description='Whether to launch RViz2'
    )

    # Substitutions
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    rviz = LaunchConfiguration('rviz')

    robot_description_content = Command(['xacro ', model])
    robot_description = ParameterValue(
        robot_description_content, value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     parameters=[{'use_sim_time': use_sim_time}],
    # )

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
        # joint_state_publisher,
        rviz_node,
    ])
