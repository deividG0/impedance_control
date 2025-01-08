import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'impedance_control'

    xacro_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        '3dof_manipulator.xacro'
    )

    robot_description = {'robot_description': Command(
        ['xacro ', xacro_file])}
    
    rviz_config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'impedance_control.rviz'
    )
    
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        )
    ])