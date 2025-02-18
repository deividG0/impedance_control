import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_name = "gripper"
    package_name = robot_name + "_description"

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    # xacro_file = os.path.join(
    #     get_package_share_directory(package_name),
    #     'urdf',
    #     'gripper.urdf'
    # )

    # robot_description = {'robot_description': Command(
    #     ['xacro ', xacro_file])}

    controller_config = os.path.join(
        get_package_share_directory(package_name), "config", "controllers.yaml"
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_config]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["effort_controller"],
    )

    delayed_effort_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[effort_controller_spawner],
        )
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    return LaunchDescription([
        rsp,
        delayed_controller_manager,
        delayed_effort_controller_spawner,
        delayed_joint_broad_spawner
    ])
