"""Phase 2 (camera) - navigate on the map RTAB-Map built, localising visually.

Same shape as qcar2_navigation_launch.py, with the localization half swapped:

    qcar2_navigation_launch.py        map_server + AMCL   (map from .yaml, lidar)
    qcar2_vslam_navigation_launch.py  RTAB-Map            (map from .db, camera)

RTAB-Map is started in localization mode (Mem/IncrementalMemory false): it loads
the database written during qcar2_vslam_mapping_launch.py, republishes that
map's occupancy grid on /map for the global costmap, matches the live RGB-D
frames against the stored ones, and publishes map -> odom. Nothing else changes -
Isaac Sim still owns odom -> base_link and everything below it, and the Nav2 half
is the same navigation_launch.py with the same params file as the AMCL route.

    # 1. Press PLAY in Isaac Sim, then:
    ros2 launch qcar2_isaac_nav2 qcar2_vslam_navigation_launch.py

    # 2. Drive a goal:
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}, \
        orientation: {w: 1.0}}}}"

Run AMCL or map_server alongside this and the stack breaks in the usual quiet
way: two publishers of map -> odom fight, and two /map publishers of differing
size give the endless "Received map message is malformed. Rejecting."

Relocalisation is not instant. RTAB-Map has to recognise a stored view before it
can place the car, so start it near where mapping started, or drive a metre or
two - until then map -> odom is identity and goals in `map` are off by whatever
the initial offset was.
"""

import os
import sys

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

# The grid and 2D-SLAM parameters live in the mapping launch file so the two
# cannot drift. ROS 2 launch does not put a launch file's own directory on
# sys.path, and both files are installed side by side in share/*/launch/.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from qcar2_vslam_mapping_launch import GRID_PARAMS, SLAM_2D_PARAMS  # noqa: E402


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('qcar2_isaac_nav2')
    nav2_share = get_package_share_directory('nav2_bringup')

    bt_dir = os.path.join(pkg_share, 'behavior_trees')
    ackermann_to_pose_bt = os.path.join(bt_dir, 'navigate_to_pose_ackermann.xml')
    ackermann_through_poses_bt = os.path.join(
        bt_dir, 'navigate_through_poses_ackermann.xml')

    params_yaml = LaunchConfiguration('params_file').perform(context)
    database_path = LaunchConfiguration('database_path').perform(context)
    map_yaml = LaunchConfiguration('map').perform(context)
    start_at_origin = LaunchConfiguration('start_at_origin').perform(context)
    # "not start_at_origin" - RTAB-Map has to relocalise visually.
    visual_reloc_str = 'false' if start_at_origin.lower() == 'true' else 'true'
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_depth_scan = LaunchConfiguration('use_depth_scan')

    if not os.path.isfile(map_yaml):
        raise RuntimeError(
            f'Map file not found: {map_yaml}\n'
            'Save it while qcar2_vslam_mapping_launch.py is running:\n'
            '  ros2 run qcar2_isaac_nav2 save_vslam_map.sh qcar2_vslam_map\n'
            '  colcon build --packages-select qcar2_isaac_nav2')

    if not os.path.isfile(database_path):
        raise RuntimeError(
            f'RTAB-Map database not found: {database_path}\n'
            'Build the map first:\n'
            '  ros2 launch qcar2_isaac_nav2 qcar2_vslam_mapping_launch.py\n'
            '  (drive the car around, then Ctrl-C to close the database)')

    # Both behavior trees must be overridden, same as the AMCL route: `spin` is
    # absent from behavior_server.behavior_plugins because an Ackermann car
    # cannot rotate in place, and bt_navigator loads navigate_to_pose AND
    # navigate_through_poses at activation.
    #
    # The static map comes from map_server reading the saved .pgm, NOT from
    # RTAB-Map. Serving it from RTAB-Map means keeping every node's occupancy
    # grid in working memory, which on this map (253 nodes, 12 m range, 3D) grew
    # the process past 6 GB and got Isaac Sim killed by the OOM killer. A .pgm
    # is a few hundred kB and is exactly the 2D projection Nav2 consumes anyway.
    #
    # Same substitution rule as qcar2_navigation_launch.py: the path goes INTO
    # the params file, never through a `map:=` launch argument.
    configured_params = RewrittenYaml(
        source_file=params_yaml,
        root_key='',
        param_rewrites={
            'yaml_filename': map_yaml,
            'default_nav_to_pose_bt_xml': ackermann_to_pose_bt,
            'default_nav_through_poses_bt_xml': ackermann_through_poses_bt,
            'map_subscribe_transient_local': LaunchConfiguration(
                'map_subscribe_transient_local'),
        },
        convert_types=False,
    )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',
            'odom_tf_linear_variance': 0.001,
            'odom_tf_angular_variance': 0.001,
            'publish_tf': True,
            'subscribe_depth': True,
            'subscribe_scan': False,
            'subscribe_odom_info': False,
            'approx_sync': False,
            'topic_queue_size': 30,
            'sync_queue_size': 30,
            'qos_image': 1,
            'qos_camera_info': 1,
            'database_path': database_path,
            # Localization mode: do not grow the map, and load the whole stored
            # graph into working memory so any part of it can be matched
            # immediately instead of being paged in after a few recognitions.
            'Mem/IncrementalMemory': 'false',
            # Tied to start_at_origin, because the two answer the same question:
            # does RTAB-Map have to FIND the car in the stored map?
            #
            #   start_at_origin=true  -> no. The pose is given, working memory
            #       can stay empty, and the process sits at ~0.7 GB.
            #   start_at_origin=false -> yes. The stored graph has to be in
            #       working memory for the live frame to be matched against it,
            #       which costs ~2.7 GB on this map and ~0.6 s of CPU per frame.
            #
            # Getting this pair wrong is quiet: with an empty working memory and
            # no given pose, RTAB-Map simply never publishes map -> odom, and
            # every goal fails with "map does not exist" while all the logs look
            # healthy.
            'Mem/InitWMWithAllNodes': visual_reloc_str,
            # Anchor the session to the stored map's origin instead of waiting
            # for a visual relocalisation.
            #
            # Until RTAB-Map recognises a stored view it opens a NEW session
            # whose graph is unconnected to the old one, and the published /map
            # and clouds then contain only the node under the car - everything
            # reports healthy while the map looks lost. Anchoring at the origin
            # skips that window, and Isaac Sim always respawns the car at the
            # spot mapping started from, so it is simply true here.
            #
            # It is not required any more: with a correctly exposed camera this
            # map relocalises visually within a second or two
            # ("Localization was good" in the log). Pass start_at_origin:=false
            # to make it do that - which is also the only honest test that
            # visual localization works at all.
            'RGBD/StartAtOrigin': start_at_origin,
            # Publish the correction as soon as one localization is good.
            #
            # The default (10) makes RTAB-Map hold a cache of odometry poses and
            # wait for several consistent localizations before it commits, which
            # gives smoother corrections on a robot that is moving. A car parked
            # at the start of a run produces no new odometry poses at all, so the
            # cache never fills: the log repeats "Localization was good, but
            # waiting for another one to be more accurate", map -> odom is never
            # published, and every goal fails with "map does not exist" - with
            # nothing in the logs marked as an error.
            'RGBD/MaxOdomCacheSize': '1',
            **GRID_PARAMS,
            **SLAM_2D_PARAMS,
        }],
        remappings=[
            ('rgb/image', '/realsense/color/image_raw'),
            ('rgb/camera_info', '/realsense/camera_info'),
            ('depth/image', '/realsense/depth/image_raw'),
            # Keep RTAB-Map's map outputs off the global names. Two publishers
            # on /map of differing size give the endless "Received map message
            # is malformed. Rejecting.", and anything that subscribes to
            # /cloud_map makes RTAB-Map assemble the full 3D cloud on the spot -
            # which is what exhausted memory here. Under /rtabmap/ they are
            # available on request and silent otherwise.
            ('map', '/rtabmap/map'),
            ('grid_map', '/rtabmap/grid_map'),
            ('cloud_map', '/rtabmap/cloud_map'),
            ('cloud_obstacles', '/rtabmap/cloud_obstacles'),
            ('cloud_ground', '/rtabmap/cloud_ground'),
        ],
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
    )

    map_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server'],
        }],
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

    # The depth image as a PointCloud2, which is what the costmaps' VoxelLayer
    # consumes. depth_image_proc is not installed on this machine; rtabmap_util
    # ships the same conversion and is already a dependency here.
    #
    # decimation 4 keeps 1 point per 4x4 px (~19 k points per frame at 640x480)
    # and voxel_size 0.10 collapses them before they are ever published. The
    # VoxelLayer's z_resolution is 0.15 m, so finer points buy nothing: they all
    # land in the same voxel, they just cost the insertion.
    depth_cloud_node = Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        name='depth_to_cloud',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'decimation': 4,
            'max_depth': 12.0,
            'min_depth': 0.25,
            'voxel_size': 0.10,
            'approx_sync': False,
            'qos': 1,
            'qos_camera_info': 1,
        }],
        remappings=[
            ('depth/image', '/realsense/depth/image_raw'),
            ('depth/camera_info', '/realsense/camera_info'),
            ('cloud', '/camera_cloud'),
        ],
    )

    # Nav2 publishes TwistStamped on /cmd_vel_nav; the Isaac Sim QCar2 drive
    # graph subscribes to plain Twist on /cmd_vel_twist, and reads angular.z as
    # a steering angle. See src/twist_stamped_to_twist.py.
    twist_bridge_node = Node(
        package='qcar2_isaac_nav2',
        executable='twist_stamped_to_twist.py',
        name='twist_stamped_to_twist',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Unchanged from qcar2_navigation_launch.py - the local costmap still wants
    # a live obstacle source, and /realsense_depth (the unregistered depth
    # camera) is the right one for that: depthimage_to_laserscan only reads a
    # band of rows, so registration to the colour image is irrelevant there.
    depth_scan_frame_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_scan_frame',
        output='screen',
        condition=IfCondition(use_depth_scan),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '--x', '0.095', '--y', '-0.003', '--z', '0.176',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'depth_scan_link',
        ],
    )

    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_scan',
        output='screen',
        condition=IfCondition(use_depth_scan),
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_frame': 'depth_scan_link',
            'range_min': 0.25,
            'range_max': 12.0,
            'scan_height': 10,
            'scan_time': 0.1,
        }],
        remappings=[
            ('depth', '/realsense_depth'),
            ('depth_camera_info', '/realsense_depth_camera_info'),
            ('scan', '/scan_depth'),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'qcar2_vslam_nav.rviz')],
    )

    return [map_server_node, map_lifecycle_node, rtabmap_node, depth_cloud_node,
            navigation_launch, twist_bridge_node, depth_scan_frame_node,
            depth_to_scan_node, rviz_node]


def generate_launch_description():
    pkg_share = get_package_share_directory('qcar2_isaac_nav2')

    return LaunchDescription([
        DeclareLaunchArgument(
            'database_path',
            default_value=os.path.join(
                os.path.expanduser('~'), '.ros', 'qcar2_vslam.db'),
            description='RTAB-Map database written by qcar2_vslam_mapping_launch.py'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_share, 'maps', 'qcar2_vslam_map.yaml'),
            description='Saved 2D projection of the RTAB-Map map, served by map_server'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'qcar2_nav2_vslam.yaml'),
            description='Camera-only Nav2 parameters: no lidar source anywhere, a '
                        'VoxelLayer fed by /camera_cloud, and 12 m ranges.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Isaac Sim /clock. Isaac Sim must be PLAYING before launch.'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically bring the Nav2 lifecycle nodes up'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open RViz preconfigured for 2D Goal Pose'),
        DeclareLaunchArgument(
            'use_depth_scan', default_value='true',
            description='Feed /scan_depth to the local costmap (see '
                        'qcar2_navigation_launch.py for why this frame exists)'),
        DeclareLaunchArgument(
            'start_at_origin', default_value='true',
            description='Assume the car begins at the map origin instead of waiting for '
                        'a visual relocalisation. Correct in Isaac Sim, which always '
                        'respawns it there, and much cheaper: ~0.7 GB instead of ~2.7 GB '
                        'and no per-frame search. Set false to make RTAB-Map actually '
                        'find itself from the camera - the honest V-SLAM test, but it '
                        'needs loop closures to succeed, so the map must have been built '
                        'with a correctly exposed camera.'),
        DeclareLaunchArgument(
            'map_subscribe_transient_local', default_value='true',
            description="Must match the QoS RTAB-Map's /map publisher uses, or the "
                        'global costmap silently never receives a map. Check with '
                        'ros2 topic info -v /map'),
        OpaqueFunction(function=launch_setup),
    ])
