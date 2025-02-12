import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def controllers_node_launch(context, *args, **kwargs):
    controllers_group = LaunchConfiguration(
        'controllers_group').perform(context)
    nodes = []
    if controllers_group == 'trajectory':
        nodes.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_trajectory_controller",
                "-c",
                "/controller_manager"
                ],
            output="screen",
        ))
    elif controllers_group == 'position':
        nodes.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=["position_controller", "-c", "/controller_manager"],
            output="screen",
        ))
    elif controllers_group == 'effort':
        nodes.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=["effort_controller", "-c", "/controller_manager"],
            output="screen",
        ))
    return nodes


def generate_launch_description():
    use_sim_time = LaunchConfiguration(
                                'use_sim_time',
                                default=True
                                )

    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='If true, use simulated clock'
    )

    controllers_group_arg = DeclareLaunchArgument(
        'controllers_group',
        default_value='effort',
        description='Determine the set of controllers to be launched',
        choices=['trajectory', 'position', 'effort']
    )

    robot_name = "gripper"
    package_name = robot_name + "_description"

    xacro_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'gripper.urdf'
    )

    robot_description = {'robot_description': Command(
        ['xacro ', xacro_file])}

    controller_config = os.path.join(
        get_package_share_directory(package_name), "config", "controllers.yaml"
    )

    return LaunchDescription([
        use_sim_time_arg,
        controllers_group_arg,
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                robot_description, controller_config],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager"
                ],
            output="screen",
        ),
        # Node(
        #     package="controller_manager",
        #     executable="spawner",
        #     arguments=["velocity_controller", "-c", "/controller_manager"],
        #     output="screen",
        # ),
        OpaqueFunction(function=controllers_node_launch),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                robot_description],
            output="screen",
        )
    ])
