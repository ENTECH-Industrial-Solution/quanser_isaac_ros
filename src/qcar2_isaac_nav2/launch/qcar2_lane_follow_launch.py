"""Camera lane following for the QCar2 - tune the colours, then drive.

    # 0. Once, in Isaac Sim with the sim STOPPED, switch on the render products
    #    this route needs - two of them, the front CSI camera and the depth
    #    camera (four cameras would starve the GPU for no gain here):
    #      QCAR2_CAMERA_PRESET=lane_avoid  scripts/isaac_camera_streams.py
    #    If the CSI graphs have never been activated on this stage, run
    #    scripts/isaac_add_csi_cameras.py first, and if /realsense_depth has
    #    never existed, scripts/isaac_add_depth_camera.py.  Without the depth
    #    camera the route still runs on the lidar alone - use avoid:=true
    #    use_depth_scan:=false and skip the preset's second render product.

    # 1. Press PLAY, then tune the lane colours against the live camera:
    ros2 launch qcar2_isaac_nav2 qcar2_lane_follow_launch.py tune:=true
    #    Sliders: colour 0=white / 1=blue, then H/S/V min-max, plus the ROI top
    #    and the look-ahead band.  's' saves ~/.ros/qcar2_lane_colors.yaml,
    #    'q' quits.  Tune white on a 2-white-line road and blue on a road with
    #    the centre line - both sets live in the same file.

    # 2. Drive, avoiding whatever is parked in the lane:
    ros2 launch qcar2_isaac_nav2 qcar2_lane_follow_launch.py
    ros2 run rqt_image_view rqt_image_view /lane/debug_image   # what it sees
    ros2 topic echo /lane/avoid                                # follow/avoid/stop

    # 3. Lane following on its own, with nothing watching for obstacles:
    ros2 launch qcar2_isaac_nav2 qcar2_lane_follow_launch.py avoid:=false

OBSTACLE AVOIDANCE lives in a second node, obstacle_avoider.py, which publishes
one word on /lane/avoid and steers nothing.  The follower stays the only
publisher on /cmd_vel_twist.  `avoid` makes it hold the lane to the LEFT of the
blue centre line; dropping back to `follow` makes it cross back.  Both halves
are closed loop against that line, so `trigger_distance` and the return
geometry are the only things to tune - there is no manoeuvre duration and no
fixed steering angle anywhere.  See src/obstacle_avoider.py for the two windows
it measures and src/lane_follower.py for what the modes do to the aim point.

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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Written by tune mode.  Kept out of the install tree on purpose: config/ is
# installed by CMake, so a value tuned there would do nothing until the next
# colcon build - the classic silent no-op in this workspace.
TUNED_COLORS = os.path.expanduser('~/.ros/qcar2_lane_colors.yaml')

# Arguments the PROFILE owns. Each declares an empty default so that "the user
# typed it" can be told from "the user did not" - a launch argument always
# arrives with a value, and a node-specific override beats a params file however
# the two are ordered, so a normal default would silently win over the profile
# every time and the file would do nothing.
FOLLOWER_TUNED = ('speed', 'kp', 'kd', 'k_head', 'max_error_rate',
                  'max_steering_angle', 'avoid_offset_frac',
                  'blue_is_centre_line')
AVOIDER_TUNED = ('scan_topics', 'trigger_distance', 'corridor_half_width',
                 'stop_distance', 'pass_distance', 'max_avoid_distance',
                 'side_clear_time', 'lane_half_width', 'min_clearance')


def typed(context, names):
    """Only the tunables actually given on the command line.

    The value goes in as the LaunchConfiguration itself, not as the string it
    performs to. launch_ros only runs a parameter value through YAML when it is
    a Substitution; hand it a plain str and it stays a str, so `speed:=0.15`
    reaches the node as STRING and rclpy rejects it against a DOUBLE
    declaration before the node has started. The string is performed here only
    to ask whether the argument was given at all.
    """
    out = {}
    for n in names:
        if LaunchConfiguration(n).perform(context).strip():
            out[n] = LaunchConfiguration(n)
    return out


def find_profile(pkg_share, name):
    """~/.ros first, then the packaged copy.

    Same rule as the colours file, and for the same reason: config/ is
    installed by CMake, so a value edited there does nothing until the next
    colcon build - the classic silent no-op in this workspace. A profile kept
    in ~/.ros is edit-and-run.
    """
    # `ros2 launch` rejects an empty value on the command line ('profile:=' is
    # a malformed argument), so `none` is the way to ask for no profile at all.
    if not name or name.lower() == 'none':
        return None
    if os.path.isabs(name) or name.endswith('.yaml'):
        path = os.path.expanduser(name)
    else:
        home = os.path.expanduser(f'~/.ros/qcar2_{name}.yaml')
        path = home if os.path.exists(home) else os.path.join(
            pkg_share, 'config', f'{name}.yaml')
    if not os.path.exists(path):
        raise RuntimeError(f'profile not found: {path}')
    return path


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('qcar2_isaac_nav2')

    colors = LaunchConfiguration('colors').perform(context)
    if not colors:
        colors = (TUNED_COLORS if os.path.exists(TUNED_COLORS)
                  else os.path.join(pkg_share, 'config', 'lane_colors.yaml'))
    if not os.path.exists(colors):
        raise RuntimeError(f'colours file not found: {colors}')

    tune = LaunchConfiguration('tune').perform(context).lower() in ('true', '1')
    state_topic = LaunchConfiguration('state_topic').perform(context)
    profile = find_profile(pkg_share,
                           LaunchConfiguration('profile').perform(context).strip())

    # Order is the precedence: profile, then anything typed on the command line.
    # Both are node-specific, so the later one wins. The colours file is a `/**:`
    # wildcard and loses to either, which is what we want - it owns the colours
    # and nothing else.
    structural = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'image_topic': LaunchConfiguration('image_topic'),
    }
    if not tune:
        structural.update({
            'cmd_topic': LaunchConfiguration('cmd_topic'),
            'publish_debug': LaunchConfiguration('publish_debug'),
            # Set before the Node is built: launch_ros normalises `parameters`
            # at construction, so mutating this dict afterwards is silently lost.
            'avoid_topic': state_topic,
        })

    follower = Node(
        package='qcar2_isaac_nav2',
        executable='lane_follower.py',
        name='lane_tuner' if tune else 'lane_follower',
        output='screen',
        arguments=['--tune'] if tune else [],
        parameters=([profile] if profile else [])
                   + [colors, structural, typed(context, FOLLOWER_TUNED)],
    )
    if tune:
        # The tuner publishes no cmd_vel, so there is nothing for an avoider to
        # influence and no reason to spend a render product on the depth camera.
        return [follower]

    avoid = LaunchConfiguration('avoid').perform(context).lower() in ('true', '1')
    if not avoid:
        return [follower]

    # Everything the profile does not set keeps obstacle_avoider.py's own
    # default - including the ones with no launch argument at all, such as the
    # side window geometry and min_hits.
    avoider = Node(
        package='qcar2_isaac_nav2',
        executable='obstacle_avoider.py',
        name='obstacle_avoider',
        output='screen',
        parameters=([profile] if profile else [])
                   + [{'use_sim_time': LaunchConfiguration('use_sim_time'),
                       'state_topic': state_topic},
                      typed(context, AVOIDER_TUNED)],
    )

    # 3 m of depth is plenty when the pull-out happens around 2 m, and a short
    # range keeps far walls on a bend out of the forward window - it also buys
    # the room to widen the band.
    #
    # scan_height 40, not the Nav2 routes' 10: the obstacles on this map are
    # 0.11-0.19 m tall, BELOW both the lidar plane (0.194 m) and the depth
    # camera's optical axis (0.176 m). A 10-row band is a horizontal plane at
    # that axis and flies over them exactly as the lidar does, so the depth
    # camera would add a second sensor that agrees about seeing nothing. The
    # rows below the axis are the ones that look down onto a low obstacle; 40
    # reaches ~0.09 m at 2 m while the floor stays past range_max.
    depth_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'depth_scan_launch.py')),
        condition=IfCondition(LaunchConfiguration('use_depth_scan')),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'range_max': '3.0',
                          'scan_height': LaunchConfiguration('scan_height')}.items(),
    )
    return [follower, avoider, depth_scan]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tune', default_value='false',
            description='Open the HSV tuning window instead of driving. '
                        'Publishes no cmd_vel.'),
        DeclareLaunchArgument(
            'profile', default_value='lane_avoid',
            description='Tuning profile: a ROS 2 parameter file holding the '
                        'driving and avoidance values so they do not have to be '
                        'retyped. Resolved as ~/.ros/qcar2_<name>.yaml if that '
                        'exists, else config/<name>.yaml in the package - the '
                        'first is edit-and-run, the second needs a colcon '
                        'build. A path ending in .yaml is used as given. Empty '
                        'loads no profile and every node keeps its own '
                        'defaults. Anything typed on the command line still '
                        'wins over the file.'),
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
            'speed', default_value='',
            description='Empty = take it from the profile. Straight-line speed [m/s]. The camera runs at ~10 Hz, '
                        'so faster than this steers on stale frames. Reduced '
                        'automatically in corners.'),
        DeclareLaunchArgument(
            'kp', default_value='',
            description='Empty = take it from the profile. Steering gain on the normalised lateral error '
                        '(-1..+1 across the image). 0.70 asks for 0.35 rad of '
                        'steer at half-image error.'),
        DeclareLaunchArgument(
            'kd', default_value='',
            description='Empty = take it from the profile. Damping on the rate of change of that error. Raise it '
                        'if the car weaves down a straight lane.'),
        DeclareLaunchArgument(
            'max_steering_angle', default_value='',
            description='Empty = take it from the profile. Steering clamp [rad]. Isaac Sim clamps around 0.5; '
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

        # -- obstacle avoidance ------------------------------------------
        DeclareLaunchArgument(
            'avoid', default_value='true',
            description='Run obstacle_avoider.py alongside the follower. False '
                        'leaves plain lane following: the follower assumes '
                        '`follow` whenever nobody publishes the state topic.'),
        DeclareLaunchArgument(
            'k_head', default_value='',
            description='Empty = take it from the profile. Gain on the lane DIRECTION - where the boundaries seen '
                        'in the two look-ahead bands converge, which is off the '
                        'image centre exactly when the car is not aligned with '
                        'the lane. It is what stops a lane change carrying past '
                        'the lane it aimed for: closed loop on a 0.66 m lane, '
                        '0 vs 0.5 gives 0.051 m of overshoot against 0.000 and '
                        'peak heading 43 deg against 34. Above ~1 the car stops '
                        'short of the new lane instead.'),
        DeclareLaunchArgument(
            'max_error_rate', default_value='',
            description='Empty = take it from the profile. How fast the aim point may travel sideways, in units '
                        'of half-image per second. Switching to `avoid` moves '
                        'it a whole lane at once, and steering on that step is '
                        'full lock, which swings the camera off the road and '
                        'ends the overtake in `no lane`. Lower is a gentler '
                        'lane change that keeps the lines in view; too low and '
                        'the car reacts to a real obstacle too slowly.'),
        DeclareLaunchArgument(
            'avoid_offset_frac', default_value='',
            description='Empty = take it from the profile. Where `avoid` puts the car on a road with NO centre '
                        'line, as a fraction of the half corridor width from '
                        'the left edge line: 1.0 is the middle, 0 is on the '
                        'line. Only used on single carriageways - with a blue '
                        'centre line the car crosses into the left lane '
                        'instead and this does nothing. Watch '
                        '/lane/debug_image while tuning: the aim point must '
                        'stay clear of the left line.'),
        DeclareLaunchArgument(
            'blue_is_centre_line', default_value='',
            description='Empty = take it from the profile. True only where a blue line really divides two '
                        'directions of traffic. It switches on both two-lane '
                        'rules: `avoid` crosses the line into the left lane, '
                        'and `follow` treats blue-on-the-right as "wrong side, '
                        'cross back". Off, blue is just another boundary colour '
                        'and `avoid` hugs the left edge instead. Leave it off '
                        'unless the blue paint is a centre line - on the shipped '
                        'Isaac map the edge lines read as blue (S 37 against a '
                        'tuned floor of 15) and the rule then asks for full lock '
                        'towards a lane that is not there.'),
        DeclareLaunchArgument(
            'state_topic', default_value='/lane/avoid',
            description='follow / avoid / stop, from the avoider to the '
                        'follower. Echo it to see the state machine.'),
        DeclareLaunchArgument(
            'use_depth_scan', default_value='true',
            description='Also feed the avoider /scan_depth, made from the '
                        'RealSense depth image. Needs the depth camera to exist '
                        'on the stage; without it the nodes start and publish '
                        'nothing, and the avoider runs on the lidar alone.'),
        DeclareLaunchArgument(
            'scan_height', default_value='40',
            description='Depth image rows collapsed into /scan_depth. 40 sees '
                        'obstacles down to ~0.1 m tall, which the lidar plane '
                        'at 0.194 m and the Nav2 routes\' 10-row band both fly '
                        'straight over. See launch/depth_scan_launch.py.'),
        DeclareLaunchArgument(
            'scan_topics', default_value='',
            description='Empty = take it from the profile. LaserScan sources, all measured in base_link through '
                        'TF. Drop /scan_depth here if the depth camera is not '
                        'set up, to silence its staleness warning.'),
        DeclareLaunchArgument(
            'trigger_distance', default_value='',
            description='Empty = take it from the profile. Pull out when something is this far ahead [m]. The '
                        'main knob, and the one that decides whether the car '
                        'clears the obstacle or clips it. Bicycle model: at '
                        'full lock the turn radius is 0.258/tan(0.42) = 0.58 m, '
                        'so moving 0.35 m sideways costs 0.67 m of forward '
                        'travel, plus ~0.2 m of nose ahead of base_link, plus '
                        'the time the PD takes to reach full lock at 10 Hz. '
                        'A lane change of y metres costs about 2*sqrt(2*R*y) of '
                        'forward travel: 0.4 m sideways needs ~1.4 m, a full '
                        '0.66 m lane ~1.75 m, plus ~0.2 m of nose ahead of '
                        'base_link. Lower it to start the manoeuvre closer in - '
                        'the car then passes nearer the obstacle, and below the '
                        'figure above it cannot get clear at all. Watch '
                        '/lane/front_distance while it happens.'),
        DeclareLaunchArgument(
            'corridor_half_width', default_value='',
            description='Empty = take it from the profile. Half width of the forward window [m]. The car body is '
                        '0.19 m wide, so this is a body plus a little margin - '
                        'widen it and roadside furniture starts triggering.'),
        DeclareLaunchArgument(
            'stop_distance', default_value='',
            description='Empty = take it from the profile. Brake instead of steering when something is this close '
                        '[m] - the obstacle appeared too late to drive round, '
                        'or the left lane is blocked too. Measured from '
                        'base_link, and the nose is ~0.2 m ahead of that, so '
                        '0.45 is about 0.25 m of actual clearance. 0 disables '
                        'braking.'),
        DeclareLaunchArgument(
            'pass_distance', default_value='',
            description='Empty = take it from the profile. Fallback for when the lidar never sees the '
                        'obstacle from the side at all - anything under its '
                        '0.194 m plane. How far to keep going [m] after the '
                        'obstacle leaves the forward window. So it means the obstacle\'s '
                        'length plus the car\'s, and it does not change when '
                        'you change trigger_distance. It is also the ONLY thing '
                        'holding the car out past an obstacle the lidar cannot '
                        'see, since the side window is lidar-only (the depth '
                        'camera sees +/-33 deg and never abeam).'),
        DeclareLaunchArgument(
            'max_avoid_distance', default_value='',
            description='Empty = take it from the profile. Give up and merge back after this far [m], for an '
                        'obstacle the side window never manages to see.'),
        DeclareLaunchArgument(
            'side_clear_time', default_value='',
            description='Empty = take it from the profile. The lane we came from must read clear for this long [s] '
                        'before we merge back. At 4 Hz of lidar this is two or '
                        'three scans, so one dropout cannot cut us back early.'),
        DeclareLaunchArgument(
            'lane_half_width', default_value='',
            description='Empty = take it from the profile. Half the width [m] of the window held over the lane the '
                        'car pulled out of. Wide enough to hold anything in the '
                        'way of merging back, narrow enough to leave the kerb '
                        'and the far wall out of the decision.'),
        DeclareLaunchArgument(
            'min_clearance', default_value='',
            description='Empty = take it from the profile. How far the car must get off the lane it left [m] before '
                        'it may look for a way back into it. Stops the car '
                        'merging back while it is still in its own lane with '
                        'the obstacle dead ahead.'),
        OpaqueFunction(function=launch_setup),
    ])
