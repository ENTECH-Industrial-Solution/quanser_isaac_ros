"""Phase 1 (camera) - build a map with RTAB-Map RGB-D SLAM instead of Cartographer.

The lidar is not used at all here. RTAB-Map takes the registered RGB-D pair from
the Isaac Sim RealSense, extracts visual features, closes loops on appearance,
and accumulates the depth images into a 3D voxel map, published as coloured
point clouds on /cloud_map, /cloud_obstacles and /cloud_ground. Nav2 is a 2D
planner, so RTAB-Map also projects that voxel map down to a plain occupancy grid
on /map, which the costmaps consume unchanged.

    # 1. Press PLAY in Isaac Sim first, then:
    ros2 launch qcar2_isaac_nav2 qcar2_vslam_mapping_launch.py

    # 2. Drive the car slowly around the whole area:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard /cmd_vel:=/cmd_vel_twist

    # 3. With this still running, save the result:
    ros2 run qcar2_isaac_nav2 save_vslam_map.sh qcar2_vslam_map

Requires the RGB-D publisher graph on the Isaac side - run
scripts/isaac_add_rgbd_camera.py once and save the stage. It publishes
/realsense/color/image_raw, /realsense/depth/image_raw and
/realsense/camera_info, all rendered from ONE render product on a camera
parented under `realsenseRGB`.

Two consequences of that arrangement matter here:

- The depth image is registered to the colour image. RTAB-Map looks up a depth
  value at the pixel where it found a visual feature, so the 37 mm offset
  between the asset's `realsenseRGB` and `realsenseDepth` prims would put every
  feature's depth ~18 px off at 1 m. The separate /realsense_depth stream used
  by depthimage_to_laserscan is NOT usable for this.
- Colour, depth and camera_info carry identical timestamps (verified: 34/34
  messages), so `approx_sync` is false. Exact sync is strictly better when it is
  available - approximate sync would happily pair frames a few tens of ms apart
  and blur the map while looking perfectly healthy.

If every loop closure is rejected with "Not enough inliers 0/15", look at the
colour image before touching any SLAM parameter. That is what a blown-out camera
looks like from here, not a texture-poor scene: a map recorded through an
overexposed camera builds and navigates fine on odometry alone, it just has no
colour and no features. See the auto-exposure note in
scripts/isaac_add_rgbd_camera.py. With the camera exposed correctly this scene
closes loops normally (31 global + 6 proximity over a 35 m drive).

Odometry comes from Isaac Sim's odom -> base_link TF, not from rgbd_odometry.
Isaac's wheel odometry is essentially ground truth here, and rgbd_odometry would
have to publish a second odom -> base_link to be useful, which would fight the
one Isaac already owns. RTAB-Map still does the visual work that matters:
feature extraction, loop closure and the grid. If you ever want true visual
odometry, the Isaac drive graph's TF publisher has to be disabled first.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Occupancy-grid settings shared by mapping and localization. Kept here so the
# two launch files cannot drift apart: a grid built with one set of heights and
# then re-served with another produces a map that does not match what Nav2 sees.
GRID_PARAMS = {
    'Grid/Sensor': '1',                # 1 = depth image (0 would be a laser scan)
    # 3D voxel map, not a flat grid. The map RTAB-Map keeps is then a real
    # reconstruction of the space - published as a point cloud on
    # /cloud_map, /cloud_obstacles and /cloud_ground (global namespace: there is
    # no /rtabmap/ prefix, the node just runs without one).
    # /map keeps working: RTAB-Map projects the 3D grid down to 2D for it, which
    # is what Nav2's costmaps consume (Nav2 plans in 2D no matter what the map
    # behind it is).
    'Grid/3D': 'true',
    'Grid/CellSize': '0.05',           # voxel edge; also the /map projection's
                                       # resolution, matching the Cartographer map
    'Grid/RangeMax': '12.0',           # how far each frame is allowed to write
                                       # into the map. Isaac renders exact depth
                                       # out to ~24 m, so this is a choice about
                                       # geometry, not sensor noise: past ~12 m a
                                       # 67 deg FOV covers 14 m of width with 640
                                       # px, and grazing angles start smearing
                                       # thin obstacles across voxels.
    'Grid/DepthDecimation': '2',       # 1 point per 2x2 px instead of the default
                                       # 4x4 - four times denser cloud, still well
                                       # inside what this GPU handles at 10 Hz
    'Grid/RayTracing': 'true',         # carve free space between sensor and hit,
                                       # otherwise everything unseen stays unknown
                                       # and the global planner refuses to plan
    'Grid/NormalsSegmentation': 'false',  # flat sim floor - the cheap passthrough
                                          # below is enough and far more stable
    'Grid/MaxGroundHeight': '0.10',    # base_link sits at floor level on this asset
    'Grid/MaxObstacleHeight': '2.00',  # keep the full height of walls and racking
                                       # in the 3D cloud. The 2D projection Nav2
                                       # sees is filtered separately by the
                                       # costmap's own z limits.
}

SLAM_2D_PARAMS = {
    'Reg/Force3DoF': 'true',           # car on a plane: solve x, y, yaw only
    'Optimizer/Slam2D': 'true',
    'RGBD/ProximityBySpace': 'true',   # close loops on revisits, not only on
                                       # appearance - matters in a warehouse where
                                       # every aisle looks alike
    'RGBD/AngularUpdate': '0.05',      # add a node every 3 deg or 5 cm. Defaults
    'RGBD/LinearUpdate': '0.05',       # (0.1) are tuned for a walking robot; this
                                       # car crawls at 0.3 m/s
    'Rtabmap/DetectionRate': '2.0',    # images arrive at 10 Hz; 2 Hz of loop-closure
                                       # detection is plenty and leaves GPU for Isaac
    'Vis/MinInliers': '15',            # the sim scene is texture-poor compared with
                                       # a real room (flat painted walls)
}


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path').perform(context)
    view_only = LaunchConfiguration('view_only').perform(context).lower() == 'true'
    # view_only opens the database read-only, so deleting it would be absurd.
    delete_db = 'false' if view_only else \
        LaunchConfiguration('delete_db_on_start').perform(context)
    reopening = view_only or delete_db.lower() != 'true'

    rtabmap_params = {
        'use_sim_time': use_sim_time,
        'frame_id': 'base_link',
        'map_frame_id': 'map',
        # Read odometry from TF rather than a topic. Isaac Sim owns
        # odom -> base_link; RTAB-Map adds map -> odom on top, exactly where
        # Cartographer and AMCL sit in the other two launch files.
        'odom_frame_id': 'odom',
        'odom_tf_linear_variance': 0.001,
        'odom_tf_angular_variance': 0.001,
        'publish_tf': True,
        'subscribe_depth': True,       # implies rgb + depth + camera_info
        'subscribe_scan': False,
        'subscribe_odom_info': False,
        'approx_sync': False,          # identical stamps - see the module docstring
        'topic_queue_size': 30,
        'sync_queue_size': 30,
        'qos_image': 1,                # Isaac publishes RELIABLE
        'qos_camera_info': 1,
        'database_path': database_path,
        # SLAM mode adds nodes; view_only does not. Reopening an existing map in
        # SLAM mode is only safe when the car really is back at the origin (see
        # RGBD/StartAtOrigin below) - otherwise the old map gets anchored to the
        # wrong place and new nodes are written on top of it, which shows up as
        # a smeared cloud and a /map smaller than the one you recorded.
        'Mem/IncrementalMemory': 'false' if view_only else 'true',
        # When continuing an existing database, pull the whole stored graph into
        # working memory. The published cloud and grid are assembled from WM
        # only, so without this a reopened map publishes just the handful of
        # nodes near the car and looks like the map was lost. It also lets
        # RTAB-Map regenerate every node's occupancy grid when the Grid/*
        # parameters above have changed - which is how a 2D map recorded earlier
        # becomes a 3D one without driving the route again.
        'Mem/InitWMWithAllNodes': 'true' if reopening else 'false',
        # It does NOT regenerate the per-node grids when the Grid/* parameters
        # above change - a database recorded with Grid/3D=false reopens with the
        # flat grid it was recorded with ("2D occupancy grid map loaded" in the
        # log) no matter what is set here. Converting an existing map to 3D
        # means reprocessing the database offline:
        #
        #   rtabmap-reprocess --Grid/3D true --Grid/DepthDecimation 2 \
        #       --Grid/MaxObstacleHeight 2.0 old.db new.db
        #
        # Continuing an existing map: assume the car is back at the origin it
        # was mapped from (true in Isaac Sim, which always respawns it there)
        # rather than waiting for a visual loop closure to connect the new
        # session to the old graph. See qcar2_vslam_navigation_launch.py.
        'RGBD/StartAtOrigin': 'true' if reopening else 'false',
        # Isaac publishes 32FC1 depth. RTAB-Map's default .rvl compression is a
        # 16-bit format, so it would either warn and silently fall back to .png
        # every run, or - if forced - truncate everything past 65 m. Say .png
        # explicitly.
        'Mem/DepthCompressionFormat': '.png',
        **GRID_PARAMS,
        **SLAM_2D_PARAMS,
    }

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_params],
        remappings=[
            ('rgb/image', '/realsense/color/image_raw'),
            ('rgb/camera_info', '/realsense/camera_info'),
            ('depth/image', '/realsense/depth/image_raw'),
        ],
        # Without this, a second mapping run silently continues the first one:
        # the old graph is reloaded, the car is "relocalised" somewhere in it,
        # and the new map comes out doubled.
        arguments=['--delete_db_on_start'] if delete_db.lower() == 'true' else [],
    )

    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rtabmap_viz')),
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'subscribe_depth': True,
            'approx_sync': False,
            'qos_image': 1,
            'qos_camera_info': 1,
        }],
        remappings=[
            ('rgb/image', '/realsense/color/image_raw'),
            ('rgb/camera_info', '/realsense/camera_info'),
            ('depth/image', '/realsense/depth/image_raw'),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', os.path.join(
            get_package_share_directory('qcar2_isaac_nav2'),
            'rviz', 'qcar2_vslam.rviz')],
    )

    # The 3D clouds (/cloud_map, /cloud_obstacles, /cloud_ground) are assembled
    # lazily: RTAB-Map only builds them when the map changes AND something is
    # subscribed. Reopening an existing map therefore shows nothing in RViz
    # until the car moves. Nudge it once, after RViz has had time to subscribe.
    #
    # Note the topics have no /rtabmap/ prefix - the node runs in the global
    # namespace, so it is /cloud_map, not /rtabmap/cloud_map. Subscribing to the
    # wrong one is invisible: the topic appears in `ros2 topic list` simply
    # because you subscribed to it, and stays silent forever.
    publish_map_once = TimerAction(
        period=20.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/rtabmap/publish_map',
                 'rtabmap_msgs/srv/PublishMap',
                 '{global_map: true, optimized: true, graph_only: false}'],
            output='log')],
    )

    return [rtabmap_node, rtabmap_viz_node, rviz_node, publish_map_once]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Isaac Sim /clock. Isaac Sim must be PLAYING before launch.'),
        DeclareLaunchArgument(
            'database_path',
            default_value=os.path.join(
                os.path.expanduser('~'), '.ros', 'qcar2_vslam.db'),
            description='RTAB-Map database. Holds the pose graph, the visual words '
                        'and the grid; needed again for localization.'),
        DeclareLaunchArgument(
            'delete_db_on_start', default_value='true',
            description='Start a fresh map. Set false only to extend an existing one, '
                        'and only with the car back at the origin it was mapped from '
                        '(restart Isaac Sim first). To just LOOK at a saved map, use '
                        'view_only:=true instead - it cannot damage the database.'),
        DeclareLaunchArgument(
            'view_only', default_value='false',
            description='Open the saved map read-only: publish /map and the 3D clouds, '
                        'add nothing. The safe way to inspect a map in RViz - safe for '
                        'the database, not for memory: assembling the full 3D cloud of '
                        'a 12 m map took 6 GB here and the OOM killer took Isaac Sim '
                        'with it. Close Isaac Sim first, or read the exported .ply.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open RViz showing /map, the RGB-D pair and the pose graph'),
        DeclareLaunchArgument(
            'use_rtabmap_viz', default_value='false',
            description="RTAB-Map's own GUI (loop closures, feature matches). "
                        'Heavy - it renders a second 3D view while Isaac Sim runs.'),
        OpaqueFunction(function=launch_setup),
    ])
