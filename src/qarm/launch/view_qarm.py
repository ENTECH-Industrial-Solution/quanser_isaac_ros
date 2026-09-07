"""Show the QArm in RViz.

robot_state_publisher turns /qarm/joint_states into TF, so the model follows
whichever arm node is publishing - the simulator or the real driver.

  ros2 launch qarm view_qarm.py                 # simulated arm
  ros2 launch qarm view_qarm.py use_sim:=false  # real QArm
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('qarm')
    urdf = os.path.join(share, 'urdf', 'qarm.urdf')
    rviz_config = os.path.join(share, 'rviz', 'qarm.rviz')

    with open(urdf, 'r') as f:
        robot_description = f.read()

    args = [
        DeclareLaunchArgument(
            'use_sim', default_value='true',
            description='Run the simulated arm instead of the real driver.'),
        DeclareLaunchArgument(
            'run_server', default_value='true',
            description='Also start the MoveQArm action server.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Open RViz.'),
    ]

    use_sim = LaunchConfiguration('use_sim')

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='qarm_state_publisher',
        parameters=[{'robot_description': robot_description}],
        # the arm nodes publish on /qarm/joint_states, not /joint_states
        remappings=[('/joint_states', '/qarm/joint_states')],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    simulator = Node(
        package='qarm',
        executable='qarm_sim',
        name='Hardware',
        condition=IfCondition(use_sim),
    )

    hardware = Node(
        package='qarm',
        executable='qarm_hardware',
        name='Hardware',
        condition=UnlessCondition(use_sim),
    )

    move_server = Node(
        package='qarm',
        executable='move_qarm_server',
        name='Move_Server',
        condition=IfCondition(LaunchConfiguration('run_server')),
    )

    return LaunchDescription(
        args + [state_publisher, simulator, hardware, move_server, rviz])
