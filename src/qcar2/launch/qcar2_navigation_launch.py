"""Phase 2 - navigate on a map that was already saved.

No SLAM here: map_server serves the saved .yaml and AMCL provides map -> odom.
Isaac Sim still provides odom -> base_link, /scan, /imu and /clock.

    # 1. Press PLAY in Isaac Sim first, then:
    ros2 launch qcar2 qcar2_navigation_launch.py

    # or with an explicit map:
    ros2 launch qcar2 qcar2_navigation_launch.py \
        map:=/home/earth157/entech_quanser_ros2_ws/src/qcar2/maps/qcar2_map.yaml

    # 2. In RViz, click "2D Pose Estimate" on the car's real position (only needed
    #    if it did not start at the mapping origin), then "2D Goal Pose".
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('qcar2')
    nav2_share = get_package_share_directory('nav2_bringup')

    bt_dir = os.path.join(pkg_share, 'behavior_trees')
    ackermann_to_pose_bt = os.path.join(bt_dir, 'navigate_to_pose_ackermann.xml')
    ackermann_through_poses_bt = os.path.join(
        bt_dir, 'navigate_through_poses_ackermann.xml')

    # Resolved HERE, in the parent context, on purpose.
    #
    # nav2's localization_launch.py also declares a launch argument named `map`.
    # IncludeLaunchDescription applies launch_arguments inside the child scope,
    # so a LaunchConfiguration('map') evaluated lazily during the include would
    # read the child's value ('') instead of ours and the map path would silently
    # vanish. OpaqueFunction forces the resolution to happen before that.
    map_yaml = LaunchConfiguration('map').perform(context)
    params_yaml = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    use_depth_scan = LaunchConfiguration('use_depth_scan')

    if not os.path.isfile(map_yaml):
        raise RuntimeError(
            f'Map file not found: {map_yaml}\n'
            'Build the map first:\n'
            '  ros2 launch qcar2 qcar2_mapping_launch.py\n'
            '  ros2 run qcar2 save_map.sh qcar2_map\n'
            '  colcon build --packages-select qcar2')

    # Inject the two paths that are only known at runtime:
    #   - yaml_filename                   -> map_server
    #   - default_nav_to_pose_bt_xml      -> bt_navigator
    #   - default_nav_through_poses_bt_xml -> bt_navigator
    #
    # BOTH trees must be overridden. bt_navigator loads them both at activation,
    # so leaving navigate_through_poses on the stock tree makes activation fail
    # with "Action server spin not available".
    #
    # The map is substituted into the params file rather than passed as
    # localization_launch.py's `map:=` argument, because that argument arrives as
    # a second params file scoped to `/**:` and ROS 2 gives an explicit
    # `map_server:` key priority over a `/**:` wildcard regardless of file order -
    # so our own empty yaml_filename would win and map_server would start blank.
    configured_params = RewrittenYaml(
        source_file=params_yaml,
        root_key='',
        param_rewrites={
            'yaml_filename': map_yaml,
            'default_nav_to_pose_bt_xml': ackermann_to_pose_bt,
            'default_nav_through_poses_bt_xml': ackermann_through_poses_bt,
        },
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

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': autostart,
        }.items(),
    )

    # Nav2 publishes TwistStamped on /cmd_vel_nav (enable_stamped_cmd_vel: true);
    # the Isaac Sim QCar2 drive graph subscribes to plain Twist on /cmd_vel_twist.
    twist_bridge_node = Node(
        package='qcar2',
        executable='twist_stamped_to_twist.py',
        name='twist_stamped_to_twist',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # RealSense depth -> /scan_depth, as a live obstacle source for the local
    # costmap. See launch/depth_scan_launch.py for the frame and scan_height
    # traps; 3 m is enough for a costmap that only needs what is
    # about to be driven over.
    depth_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'depth_scan_launch.py')),
        condition=IfCondition(use_depth_scan),
        launch_arguments={'use_sim_time': use_sim_time,
                          'range_max': '3.0'}.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'qcar2_nav2.rviz')],
    )

    return [localization_launch, navigation_launch, twist_bridge_node,
            depth_scan, rviz_node]


def generate_launch_description():
    pkg_share = get_package_share_directory('qcar2')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_share, 'maps', 'qcar2_map.yaml'),
            description='Full path to the saved map .yaml produced by save_map.sh'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'qcar2_nav2_amcl.yaml'),
            description='Nav2 + AMCL parameters'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Isaac Sim /clock. Isaac Sim must be PLAYING before launch.'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically bring the Nav2 lifecycle nodes up'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open RViz preconfigured for 2D Pose Estimate / 2D Goal Pose'),
        DeclareLaunchArgument(
            'use_depth_scan', default_value='true',
            description='Flatten the Isaac Sim RealSense depth image into /scan_depth '
                        'and feed it to the local costmap. Requires the depth publisher '
                        'graph: run scripts/isaac_add_depth_camera.py in Isaac Sim first. '
                        'Harmless if that graph is missing - the source just stays empty.'),
        OpaqueFunction(function=launch_setup),
    ])
