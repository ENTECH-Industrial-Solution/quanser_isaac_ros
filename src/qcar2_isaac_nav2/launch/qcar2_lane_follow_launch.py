"""Camera lane following for the QCar2 - tune the colours, then drive.

    # 0. Once, in Isaac Sim with the sim STOPPED, switch on the front CSI
    #    render product (four cameras would starve the GPU for no gain here):
    #      QCAR2_CAMERA_PRESET=csi_front  scripts/isaac_camera_streams.py
    #    If the CSI graphs have never been activated on this stage, run
    #    scripts/isaac_add_csi_cameras.py first.

    # 1. Press PLAY, then tune the lane colours against the live camera:
    ros2 launch qcar2_isaac_nav2 qcar2_lane_follow_launch.py tune:=true
    #    Sliders: colour 0=white / 1=blue, then H/S/V min-max, plus the ROI top
    #    and the look-ahead band.  's' saves ~/.ros/qcar2_lane_colors.yaml,
    #    'q' quits.  Tune white on a 2-white-line road and blue on a road with
    #    the centre line - both sets live in the same file.

    # 2. Drive:
    ros2 launch qcar2_isaac_nav2 qcar2_lane_follow_launch.py
    ros2 run rqt_image_view rqt_image_view /lane/debug_image   # what it sees

NAV2 MUST NOT BE RUNNING.  The follower is a controller in its own right and
publishes straight to /cmd_vel_twist, the same topic twist_stamped_to_twist.py
feeds from Nav2 - two publishers there fight over the steering.  There is no
map, costmap or goal pose involved in this route at all.

Only `tune:=true` swaps the executable's mode; everything else is shared, so
the thresholds you approve in the tuner are the ones the follower uses.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Written by tune mode.  Kept out of the install tree on purpose: config/ is
# installed by CMake, so a value tuned there would do nothing until the next
# colcon build - the classic silent no-op in this workspace.
TUNED_COLORS = os.path.expanduser('~/.ros/qcar2_lane_colors.yaml')


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('qcar2_isaac_nav2')

    colors = LaunchConfiguration('colors').perform(context)
    if not colors:
        colors = (TUNED_COLORS if os.path.exists(TUNED_COLORS)
                  else os.path.join(pkg_share, 'config', 'lane_colors.yaml'))
    if not os.path.exists(colors):
        raise RuntimeError(f'colours file not found: {colors}')

    tune = LaunchConfiguration('tune').perform(context).lower() in ('true', '1')

    # Launch arguments arrive as node-specific overrides and therefore beat the
    # `/**:` wildcard in the colours file regardless of order, which is what we
    # want: the file owns the colours, the command line owns the driving.
    overrides = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'image_topic': LaunchConfiguration('image_topic'),
    }
    if not tune:
        overrides.update({
            'cmd_topic': LaunchConfiguration('cmd_topic'),
            'speed': LaunchConfiguration('speed'),
            'kp': LaunchConfiguration('kp'),
            'kd': LaunchConfiguration('kd'),
            'max_steering_angle': LaunchConfiguration('max_steering_angle'),
            'publish_debug': LaunchConfiguration('publish_debug'),
        })

    return [Node(
        package='qcar2_isaac_nav2',
        executable='lane_follower.py',
        name='lane_tuner' if tune else 'lane_follower',
        output='screen',
        arguments=['--tune'] if tune else [],
        parameters=[colors, overrides],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tune', default_value='false',
            description='Open the HSV tuning window instead of driving. '
                        'Publishes no cmd_vel.'),
        DeclareLaunchArgument(
            'colors', default_value='',
            description='Lane colours parameter file. Empty means '
                        '~/.ros/qcar2_lane_colors.yaml if it exists, else the '
                        'packaged config/lane_colors.yaml.'),
        DeclareLaunchArgument(
            'image_topic', default_value='/csi_front/image_raw',
            description='Camera to detect the lane in. The front CSI camera '
                        'looks down the road and costs one render product.'),
        DeclareLaunchArgument(
            'cmd_topic', default_value='/cmd_vel_twist',
            description='Where the steering command goes. This is the Isaac '
                        'QCar2 drive graph topic, so angular.z is a front-wheel '
                        'STEERING ANGLE in radians, not a yaw rate.'),
        DeclareLaunchArgument(
            'speed', default_value='0.30',
            description='Straight-line speed [m/s]. The camera runs at ~10 Hz, '
                        'so faster than this steers on stale frames. Reduced '
                        'automatically in corners.'),
        DeclareLaunchArgument(
            'kp', default_value='0.70',
            description='Steering gain on the normalised lateral error '
                        '(-1..+1 across the image). 0.70 asks for 0.35 rad of '
                        'steer at half-image error.'),
        DeclareLaunchArgument(
            'kd', default_value='0.10',
            description='Damping on the rate of change of that error. Raise it '
                        'if the car weaves down a straight lane.'),
        DeclareLaunchArgument(
            'max_steering_angle', default_value='0.50',
            description='Steering clamp [rad]. Isaac Sim clamps around 0.5; '
                        'keep this equal to the bridge and the Nav2 turning '
                        'radius so all three agree about the car.'),
        DeclareLaunchArgument(
            'publish_debug', default_value='true',
            description='Publish /lane/debug_image with masks, boundaries and '
                        'the aim point drawn on. This is how you verify the '
                        'follower without the tuner window.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Isaac Sim publishes /clock.'),
        OpaqueFunction(function=launch_setup),
    ])
