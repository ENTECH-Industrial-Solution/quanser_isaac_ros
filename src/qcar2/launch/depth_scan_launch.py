"""RealSense depth image -> /scan_depth, in a frame that is not upside down.

Included by every route that wants the depth camera as an obstacle source:
qcar2_navigation_launch.py and qcar2_vslam_navigation_launch.py feed it to the
local costmap, qcar2_lane_follow_launch.py feeds it to obstacle_avoider.py.
Only `range_max` ever differed between them, so that is the one argument; it
lived as three copies of these two nodes before, and the copies had already
drifted apart under a comment claiming they had not.

Needs scripts/isaac_add_depth_camera.py to have been run on the stage and the
`ros_qcar2_realsense_depth` render product to be enabled by
scripts/isaac_camera_streams.py. Without either, these nodes start and publish
nothing, which is a silence rather than an error.

WHY THE EXTRA FRAME

depthimage_to_laserscan collapses the centre rows of the depth image into a
LaserScan. It does NOT stamp the camera's own frame on the result: the ranges it
emits follow the LaserScan convention (angles about +z, zero at +x), while
Isaac's `realsenseDepth` frame is an OPTICAL frame (+z forward, +y down).
Publishing the scan in `realsenseDepth` would rotate every obstacle 90 degrees
into the floor. So declare a second, non-optical child of base_link at the same
physical spot - translation copied from the QCar2 USD (base_link ->
realsenseDepth = 0.095, -0.003, 0.176 m) with no rotation - and hand that to
output_frame.

scan_height AND WHAT IT CAN SEE

scan_height is the number of image rows collapsed around the optical axis, and
it decides the SHORTEST obstacle this can detect. The camera sits h = 0.176 m up
looking straight ahead, fy ~= 484, so a row `n` pixels below centre:

    hits the floor at   h * fy / n          metres  (n=5 -> 17 m, n=20 -> 4.3 m)
    passes at height    h - d * n / fy      metres at distance d

Both halves matter, in opposite directions:

    too narrow  the band is a horizontal plane at 0.176 m and flies straight
                over anything shorter than that. Measured on this scene: the
                barriers are 0.11-0.19 m tall, the lidar plane at 0.194 m reads
                a clean wall 4.8 m away while the car's nose is against one, and
                a 10-row band would agree with the lidar and miss them too.

    too wide    the floor comes inside range_max and registers as a wall a few
                metres ahead. The floor row is safe while h * fy / n > range_max,
                i.e. n < 85.2 / range_max: at range_max 3.0 that allows n up to
                28, so scan_height up to ~56.

40 rows (+/- 20) sits between the two: the floor lands at 4.3 m and is discarded,
while the lowest ray passes 0.093 m up at 2 m and 0.114 m up at 1.5 m, so
obstacles about a tenth of a metre tall are caught in time to steer round them.
The Nav2 routes keep 10 because their costmaps want the far, clean reading and
already have the lidar for what is close.
"""

from ament_index_python import get_package_share_directory
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Isaac Sim publishes /clock.'),
        DeclareLaunchArgument(
            'scan_height', default_value='10',
            description='Image rows collapsed into the scan. 10 is a horizontal '
                        'plane at the camera height and misses anything shorter '
                        'than 0.176 m; 40 reaches down to ~0.1 m obstacles at '
                        '2 m. Keep it under 85.2/range_max or the floor becomes '
                        'a wall - see the header.'),
        DeclareLaunchArgument(
            'range_max', default_value='3.0',
            description='Longest depth reading kept [m]. Short for obstacle '
                        'work (fewer far-wall false positives), long for a '
                        'costmap that wants to see down the road.'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='depth_scan_frame',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '--x', '0.095', '--y', '-0.003', '--z', '0.176',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'depth_scan_link',
            ],
        ),
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depth_to_scan',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'output_frame': 'depth_scan_link',
                'range_min': 0.25,
                'range_max': ParameterValue(LaunchConfiguration('range_max'),
                                            value_type=float),
                'scan_height': ParameterValue(LaunchConfiguration('scan_height'),
                                              value_type=int),
                'scan_time': 0.1,
            }],
            remappings=[
                ('depth', '/realsense_depth'),
                ('depth_camera_info', '/realsense_depth_camera_info'),
                ('scan', '/scan_depth'),
            ],
        ),
    ])
