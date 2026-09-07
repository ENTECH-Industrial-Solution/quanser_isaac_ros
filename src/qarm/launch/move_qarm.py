# This is the launch file that starts up the basic QArm nodes for move arm action

from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    # Declare launch arguments with default values
    declare_args = [
        DeclareLaunchArgument('goal_pose', default_value='[0.0,0.0,0.5,0.0]'),
        DeclareLaunchArgument(
            'use_sim', default_value='false',
            description='Run the simulated arm (qarm_sim) instead of the '
                        'physical QArm driver. No Quanser hardware needed.'),
    ]

    use_sim = LaunchConfiguration('use_sim')

    hardware = Node(
            package='qarm',
            executable='qarm_hardware',
            name='Hardware',
            condition=UnlessCondition(use_sim)
        )

    simulator = Node(
            package='qarm',
            executable='qarm_sim',
            name='Hardware',
            condition=IfCondition(use_sim)
        )

    move_server = Node(
            package='qarm',
            executable='move_qarm_server',
            name='Move_Server'
        )

    move_client = Node(
            package='qarm',
            executable='move_qarm_client',
            name='Move_Client',
            parameters=[{
                'goal_pose':LaunchConfiguration("goal_pose")
            }]
        )

    return LaunchDescription(
        declare_args+[
        hardware,
        simulator,
        # realsense_camera_node,
        move_server,
        move_client,
    ])
