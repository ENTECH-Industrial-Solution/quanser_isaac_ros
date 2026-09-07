"""Phase 2 - navigate on a map that was already saved.

No SLAM here: map_server serves the saved .yaml and AMCL provides map -> odom.
Isaac Sim still provides odom -> base_footprint -> base_link, /scan, /imu and
/clock.

    # 1. Press PLAY in Isaac Sim first, then:
    ros2 launch qbot_platform_issac_nav2 qbot_platform_navigation_launch.py

    # or with an explicit map:
    ros2 launch qbot_platform_issac_nav2 qbot_platform_navigation_launch.py \
        map:=/home/earth157/entech_quanser_ros2_ws/src/qbot_platform_issac_nav2/maps/qbot_map.yaml

    # 2. In RViz, click "2D Pose Estimate" on the robot's real position (only
    #    needed if it did not start at the mapping origin), then "2D Goal Pose".

Unlike qbot_platform_slam_and_nav_bringup_launch.py, which runs Cartographer
live and never reads a saved map, this one is the phase-2 half: the map is
fixed and AMCL only has to localize within it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

# Navigation must not start configuring until AMCL is actually broadcasting
# map -> odom.  nav2_costmap_2d gives up on that transform after a short
# timeout and reports it as a hard activation failure:
#
#   global_costmap: Invalid frame ID "base_link" ... frame does not exist
#   Failed to activate global_costmap because transform from base_link to map
#   did not become available before timeout
#   lifecycle_manager_navigation: Failed to bring up all requested nodes
#
# which leaves localization up, the map served, TF complete moments later, and
# every goal rejected - a failure that looks like a frame or a map problem
# rather than a race.  Isaac Sim runs this scene at roughly half real time and
# publishes /scan at ~7 Hz, so AMCL needs several wall seconds to see its first
# scan and start broadcasting.
NAVIGATION_START_DELAY_S = 10.0


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('qbot_platform_issac_nav2')
    nav2_share = get_package_share_directory('nav2_bringup')

    # Resolved HERE, in the parent context, on purpose.
    #
    # nav2's localization_launch.py also declares a launch argument named `map`.
    # IncludeLaunchDescription applies launch_arguments inside the CHILD scope,
    # so a LaunchConfiguration('map') evaluated lazily during the include would
    # read the child's value ('') instead of ours and the map path would
    # silently vanish. OpaqueFunction forces the resolution to happen first.
    map_yaml = LaunchConfiguration('map').perform(context)
    params_yaml = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')

    if not os.path.isfile(map_yaml):
        raise RuntimeError(
            f'Map file not found: {map_yaml}\n'
            'Build the map first:\n'
            '  ros2 launch qbot_platform_issac_nav2 qbot_platform_cartographer_launch.py\n'
            '  ros2 run nav2_map_server map_saver_cli -f '
            '<pkg>/maps/qbot_map --ros-args -p use_sim_time:=true\n'
            '  colcon build --packages-select qbot_platform_issac_nav2')

    # The map is substituted INTO the params file rather than passed as
    # localization_launch.py's `map:=` argument, because that argument arrives
    # as a second params file scoped to `/**:`, and ROS 2 gives an explicit
    # `map_server:` key priority over a `/**:` wildcard regardless of file
    # order - so our own empty yaml_filename would win and map_server would
    # start blank.
    configured_params = RewrittenYaml(
        source_file=params_yaml,
        root_key='',
        param_rewrites={'yaml_filename': map_yaml},
        convert_types=False,
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': '',                 # already baked into configured_params
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': autostart,
        }.items(),
    )

    navigation_launch = TimerAction(
        period=NAVIGATION_START_DELAY_S,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': configured_params,
                'autostart': autostart,
            }.items(),
        )],
    )

    # Nav2 publishes TwistStamped on /cmd_vel_nav (enable_stamped_cmd_vel:
    # true); the Isaac Sim QBot drive graph subscribes to plain Twist on
    # /cmd_vel_twist. For this differential-drive robot the bridge is a pure
    # message retype - no unit conversion, unlike the QCar2's.
    twist_bridge_node = Node(
        package='qbot_platform_issac_nav2',
        executable='twist_stamped_to_twist.py',
        name='twist_stamped_to_twist',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', os.path.join(pkg_share, 'rviz',
                                      '/home/earth157/entech_quanser_ros2_ws/src/qbot_platform_issac_nav2/rviz/qbot_platform.rviz')],
    )

    return [localization_launch, navigation_launch, twist_bridge_node, rviz_node]


def generate_launch_description():
    pkg_share = get_package_share_directory('qbot_platform_issac_nav2')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_share, 'maps', 'qbot_map.yaml'),
            description='Full path to the saved map .yaml from map_saver_cli'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config',
                                       'qbot_platform_nav2_amcl.yaml'),
            description='Nav2 + AMCL parameters'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Isaac Sim /clock. Isaac Sim must be PLAYING '
                        'before launch.'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically bring the Nav2 lifecycle nodes up'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open RViz preconfigured for 2D Pose Estimate / '
                        '2D Goal Pose'),
        OpaqueFunction(function=launch_setup),
    ])
