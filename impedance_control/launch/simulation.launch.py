
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments values
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='If true, use simulated clock'
    )
    
    # Gazebo related
    world_path = os.path.join(get_package_share_directory('impedance_control'), 'worlds', 'empty_world.world')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'),
                'launch','gazebo.launch.py'])])
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("impedance_control"),
                '/launch',
                '/view_manipulator.launch.py'
            ]
        )
    )

    robot_spawn_node = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'robot'],
        output='screen'
    )

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load'
    )

    return LaunchDescription([
        robot_spawn_node,
        robot_description,
        use_sim_time_arg,
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        # Node(
        #     package="controller_manager",
        #     executable="spawner",
        #     arguments=["position_controller", "-c", "/controller_manager"],
        #     output="screen",
        # ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
            output="screen",
        ),
        declare_world_cmd,
        gazebo_launch,
    ])
