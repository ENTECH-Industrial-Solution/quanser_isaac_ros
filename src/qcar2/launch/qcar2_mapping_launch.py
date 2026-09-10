"""Phase 1 - build a map.

Drive the QCar2 around in Isaac Sim with teleop while Cartographer builds the map,
then run scripts/save_map.sh to write maps/<name>.yaml + .pgm.

    # 1. Press PLAY in Isaac Sim first, then:
    ros2 launch qcar2 qcar2_mapping_launch.py

    # 2. In a second terminal, drive the car:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard /cmd_vel:=/cmd_vel_twist

    # 3. When the map looks complete, in a third terminal:
    ros2 run qcar2 save_map.sh qcar2_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('qcar2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    configuration_basename = LaunchConfiguration('configuration_basename')

    cartographer_config_dir = PathJoinSubstitution(
        [FindPackageShare('qcar2'), 'config']
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use the Isaac Sim /clock. Isaac Sim must be PLAYING before launch.')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Open RViz to watch the map being built')

    declare_configuration_basename = DeclareLaunchArgument(
        'configuration_basename', default_value='qcar2_mapping.lua',
        description='Cartographer .lua file in this package config/ directory')

    declare_resolution = DeclareLaunchArgument(
        'resolution', default_value='0.05',
        description='Occupancy grid cell size in meters')

    declare_publish_period_sec = DeclareLaunchArgument(
        'publish_period_sec', default_value='1.0',
        description='How often the occupancy grid is republished')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename,
        ],
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', resolution,
            '-publish_period_sec', publish_period_sec,
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'qcar2_mapping.rviz')],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_configuration_basename,
        declare_resolution,
        declare_publish_period_sec,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node,
    ])