#!/usr/bin/env python3
"""Decide when the lane follower swings into the left lane, and when to come back.

This node steers nothing.  It is pure sensing plus one state machine, exactly
like yolo_detector.py, and it publishes a single word on /lane/avoid:

    follow   drive your own lane normally
    avoid    hold the LEFT lane - there is something in ours
    stop     something is too close to drive round; brake

lane_follower.py is still the only publisher on /cmd_vel_twist.  That is not a
style preference: two publishers on that topic fight over the steering angle and
the car drives a blend of both, which is the failure mode CLAUDE.md warns about
for Nav2 + lane following.  Keeping the decision and the control in separate
nodes on separate topics also means this node can be started and stopped under a
running follower - `follow` is what the follower assumes when nobody is talking.

Two sensors, one code path
--------------------------
`/scan` (Isaac's lidar, ~4 Hz, 360 deg) and `/scan_depth`
(depthimage_to_laserscan over /realsense_depth, ~10 Hz, forward only) are both
sensor_msgs/LaserScan, so `scan_topics` is a list and every source goes through
the same projection.  They complement each other rather than duplicate: the
lidar is the one that can see abeam and behind, which is what tells us the
obstacle has been passed, while the depth camera refreshes 2-3x faster and sees
things at car-body height that a single lidar plane can miss.

Everything is measured in base_link, not in the sensor frames.  The lidar and
the depth camera sit ~0.1 m apart along x, which is 8% of a 1.2 m trigger
distance, so "how far ahead" is only meaningful after a TF transform.  The scans
arrive in `lidar` and `depth_scan_link`; both are static children of base_link,
and looking them up at time zero (latest available) keeps this immune to the
/clock jumps that follow a sim replay.

The two windows
---------------
Everything this node decides comes from counting points in two boxes.  They
live in different frames, and that is the whole design:

    FRONT     in base_link (x forward, y LEFT), so it points where the car
              points.  |y| <= corridor_half_width, x <= trigger_distance.
              Something here means our lane is blocked.  A rectangle, not a
              range circle: a wall 1.2 m away at 60 deg is not in our way.

    BEHIND    in the frame of the LANE the pull-out left, recorded in odom the
              moment it starts.  |y| <= lane_half_width across that lane, and
              side_x_min .. side_x_max along it, measured from the car - so it
              travels with the car but does not swing out with it.  Used only
              while avoiding.  Empty for side_clear_time means the lane we came
              from is free to go back into.

              Pinning it to the lane and not to the car is what makes the
              answer honest.  A box at a fixed offset to the right reaches the
              obstacle only if the berth happens to be narrow enough: measured,
              a 0.20 m block given a 0.80 m berth reads clear 1.18 m early with
              the block still dead abeam.  Sitting in the lane, the same window
              is exact.

              It is the lidar's job: the depth camera's field of view is
              +/-33 deg and cannot see abeam at all.  So for an obstacle the
              lidar cannot see - anything below its 0.194 m plane - the window
              reads clear the whole way past and `pass_distance` is the ONLY
              thing holding the car out.  Size that parameter to the longest
              obstacle you expect, not to the shortest manoeuvre.

A single stray ray sets nothing off: `min_hits` points must fall inside a window
before it counts as occupied.  An obstacle at 1.2 m subtends many rays on both
sensors; a lone hit is noise or a floor glint.

Getting out is gated, getting in is not
---------------------------------------
The moment the front window trips we go to `avoid`.  Coming back needs
`pass_distance` driven since the obstacle LEFT THE FRONT WINDOW, and then, if
the right window ever held anything during the manoeuvre, that window clear for
side_clear_time as well.

Nothing brings the car back while the forward window still shows something
inside trigger_distance either.  A row of obstacles is one manoeuvre, not one
per block: merging back into a gap only to pull out again 0.6 m later is
weaving, and it puts the car nose-first at the next block every time.

The distance is never skipped.  The window can only delay the return, never
bring it forward, because it cannot tell which object it is holding: on a course
with blocks along the verge it is busy almost continuously, and the first gap
the car turns past looks exactly like the obstacle falling behind.  Measured,
letting it decide alone returned the car at 0.28-0.44 m past, each time straight
back into the same obstacle at 0.33-0.42 m, flapping between lanes until it left
the paint.

Where the window never held anything at all, its opinion is waived rather than
trusted - an obstacle under the lidar plane is invisible from the side and reads
identical to open road, so only the distance is evidence.

`pass_distance` is measured from level rather than from the trigger because the
trigger also covers the approach, which is however far away the obstacle
happened to be spotted: 1.3 m of a 3.0 m gate went on merely driving up to it,
so the number had to be re-tuned whenever the trigger distance changed.
Measured from level it is simply how far past the thing to go.

Until the front window clears there is no merging back at all, whatever the
side window says.  `max_avoid_distance`, measured from the trigger, is the
bail-out for an obstacle that never clears.

Odometry is optional.  If /odom never arrives the distance gates are skipped
(with a warning) rather than latching us in `avoid` forever.

Never having heard a scan is NOT a dropout
------------------------------------------
The two cases look the same in `tick` and must not be treated the same.  A
source that was working and went quiet is a dropout: hold the current state,
because cutting back to `follow` in the middle of an overtake steers into the
thing being overtaken.  A source that has never said anything at all is a
CONFIGURATION failure - wrong topic, depth camera absent from the stage, no TF
to base_link - and holding `follow` there means the car drives at full
confidence with no obstacle sensing at all, with nothing in its behaviour to
show it.  That reads exactly like working obstacle avoidance right up until the
first obstacle.  So until one scan has been projected successfully we publish
`stop`, and the car does not move until the sensing does.  `avoid:=false` is
the way to drive with no obstacle sensing on purpose.
"""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)

FOLLOW, AVOID, STOP = 'follow', 'avoid', 'stop'


def quat_to_matrix(q):
    """geometry_msgs/Quaternion -> 3x3 rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def scan_to_points(msg, min_range):
    """LaserScan -> Nx3 points in the scan's own frame.

    Drops NaN and Inf (no return), anything outside the sensor's own declared
    range, and anything closer than min_range - on the QCar2 the lidar sees
    parts of the car itself at very short range.
    """
    ranges = np.asarray(msg.ranges, dtype=np.float64)
    if ranges.size == 0:
        return np.zeros((0, 3))
    lo = max(min_range, msg.range_min)
    good = np.isfinite(ranges) & (ranges >= lo) & (ranges <= msg.range_max)
    if not good.any():
        return np.zeros((0, 3))
    idx = np.flatnonzero(good)
    ang = msg.angle_min + idx * msg.angle_increment
    r = ranges[idx]
    return np.column_stack((r * np.cos(ang), r * np.sin(ang), np.zeros(r.size)))


class ObstacleAvoider(Node):

    def __init__(self):
        super().__init__('obstacle_avoider')

        # THESE NUMBERS ARE FALLBACKS, NOT THE VALUES THE CAR RUNS ON.
        #
        # declare_parameter says "this parameter exists, with this type, and
        # this is what it is worth if nobody supplies one". The launch file
        # supplies one: config/lane_avoid.yaml goes in as --params-file, and
        # anything typed on the command line after it. So the tuned values live
        # in the profile, and these only take effect under `profile:=none`.
        # They are deliberately the more cautious of the two.
        # -- what we listen to -------------------------------------------
        self.declare_parameter('scan_topics', ['/scan', '/scan_depth'])
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('state_topic', '/lane/avoid')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('scan_timeout', 1.0)
        self.declare_parameter('min_range', 0.12)
        self.declare_parameter('min_hits', 3)

        # -- when to pull out --------------------------------------------
        self.declare_parameter('trigger_distance', 2.00)
        self.declare_parameter('corridor_half_width', 0.22)
        self.declare_parameter('stop_distance', 0.45)

        # -- when to come back -------------------------------------------
        self.declare_parameter('side_x_min', -0.30)
        self.declare_parameter('side_x_max', 1.20)
        self.declare_parameter('lane_half_width', 0.35)
        self.declare_parameter('side_clear_time', 0.60)
        self.declare_parameter('min_clearance', 0.40)
        self.declare_parameter('pass_distance', 1.00)
        self.declare_parameter('max_avoid_distance', 8.00)

        g = self.get_parameter
        self.robot_frame = g('robot_frame').value
        self.scan_timeout = g('scan_timeout').value
        self.min_range = g('min_range').value
        self.min_hits = g('min_hits').value
        self.trigger_distance = g('trigger_distance').value
        self.corridor_half_width = g('corridor_half_width').value
        self.stop_distance = g('stop_distance').value
        self.side_x = (g('side_x_min').value, g('side_x_max').value)
        self.lane_half_width = g('lane_half_width').value
        self.side_clear_time = g('side_clear_time').value
        self.min_clearance = g('min_clearance').value
        self.pass_distance = g('pass_distance').value
        self.max_avoid_distance = g('max_avoid_distance').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # topic -> (stamp, Nx3 points in base_link).  Kept per source so one
        # dead sensor ages out on its own instead of poisoning the other.
        self.clouds = {}
        self.ever_seen = False
        self.odom_xy = None
        self.odom_yaw = 0.0
        self.odom_path = 0.0       # cumulative path length, metres
        # The lane the pull-out left, as (x, y, yaw) in odom, and the car's
        # pose within it.  None until a pull-out records one; see lane_frame.
        self.lane = None
        self.obstacle_s = None     # where the obstacle was, along that lane
        self.lane_pose = (0.0, 0.0, 0.0)
        self.lane_hits = None      # returns in it last tick, for the marker
        self.avoid_start_path = None
        self.level_path = None     # path length when the obstacle left the front
        self.side_seen = False     # has the right window actually held the obstacle?
        self.state = FOLLOW
        self.pre_stop = FOLLOW     # what a stop interrupted
        self.side_clear_since = None

        self.topics = list(g('scan_topics').value)
        self.subs = [self.create_subscription(
            LaserScan, t, lambda msg, t=t: self.on_scan(t, msg), SENSOR_QOS)
            for t in self.topics]
        self.create_subscription(
            Odometry, g('odom_topic').value, self.on_odom, SENSOR_QOS)
        self.state_pub = self.create_publisher(String, g('state_topic').value, 10)
        self.front_pub = self.create_publisher(Float32, '/lane/front_distance', 10)
        # The two windows, drawn where they actually are. Without this the only
        # way to tell whether the lidar is watching the side is to read the
        # log: a cloud of points on screen says nothing about which of them the
        # decision is counting.
        self.win_pub = self.create_publisher(MarkerArray, '/lane/windows', 1)
        self.create_timer(1.0 / max(1.0, g('rate').value), self.tick)

        self.get_logger().info(
            f"obstacle avoider: {' + '.join(self.topics)} -> "
            f"{g('state_topic').value}; pull out at "
            f'{self.trigger_distance:.2f} m in a '
            f'{2 * self.corridor_half_width:.2f} m corridor, '
            f'stop at {self.stop_distance:.2f} m')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # -- inputs ----------------------------------------------------------

    def on_odom(self, msg):
        p = msg.pose.pose.position
        xy = (p.x, p.y)
        if self.odom_xy is not None:
            # PATH length, not displacement from the start. An Ackermann car
            # swinging out and back covers a curved path whose endpoints can be
            # barely apart - measured on this map, 16 s of avoiding moved the
            # car 1.1 m in a straight line while it drove several times that -
            # so a displacement gate never opens and the overtake never ends.
            self.odom_path += math.hypot(xy[0] - self.odom_xy[0],
                                         xy[1] - self.odom_xy[1])
        self.odom_xy = xy
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_scan(self, topic, msg):
        pts = scan_to_points(msg, self.min_range)
        if pts.shape[0] == 0:
            self.clouds[topic] = (self.now(), pts)
            return
        try:
            # Time() means "latest available".  These are static transforms, so
            # there is nothing to interpolate, and asking for the scan's own
            # stamp would fail for a while after a sim replay rewinds /clock.
            tf = self.tf_buffer.lookup_transform(
                self.robot_frame, msg.header.frame_id, Time(),
                timeout=Duration(seconds=0.1))
        except Exception as exc:                                # noqa: BLE001
            self.get_logger().warn(
                f'no TF {self.robot_frame} <- {msg.header.frame_id}: {exc}',
                throttle_duration_sec=5.0)
            return
        t = tf.transform.translation
        pts = pts @ quat_to_matrix(tf.transform.rotation).T
        pts += np.array([t.x, t.y, t.z])
        self.clouds[topic] = (self.now(), pts)
        self.ever_seen = True

    # -- windows ---------------------------------------------------------

    def fresh_points(self):
        """(points from every source that has spoken recently, sources that have not).

        A topic listed in `scan_topics` that never produces anything is the
        quiet failure on this route: the depth camera missing from the stage, or
        its render product switched off by a camera preset, leaves the avoider
        running on the lidar alone with nothing anywhere to say so.
        """
        cutoff = self.now() - self.scan_timeout
        live, silent = [], []
        for topic in self.topics:
            entry = self.clouds.get(topic)
            if entry is None or entry[0] < cutoff:
                silent.append(topic)
            else:
                live.append(entry[1])
        if not live:
            return None, silent
        return (np.vstack(live) if len(live) > 1 else live[0]), silent

    def front_distance(self, pts):
        """Nearest obstacle straight ahead [m], or inf.

        Longitudinal x, not range: what matters for both the pull-out and the
        brake is how far we can still drive, and a point 0.3 m off to the side
        of the corridor is not in the way however short its range is.
        """
        sel = pts[(np.abs(pts[:, 1]) <= self.corridor_half_width) &
                  (pts[:, 0] > 0.0)]
        if sel.shape[0] < self.min_hits:
            return math.inf
        # The min_hits'th nearest, so one early ray cannot decide it.
        return float(np.partition(sel[:, 0], self.min_hits - 1)[self.min_hits - 1])

    # -- the lane we pulled out of --------------------------------------
    #
    # "Have I passed it yet?" is really "is the lane I want to go back into
    # free where I am about to go into it?", and a box glued to the CAR cannot
    # ask that, because the car has just moved out of that lane. Two ways it
    # lied, both measured:
    #
    #   * Fixed at 0.10..0.60 m to the right, it reaches the obstacle only if
    #     the berth happens to be narrow enough. Give a 0.20 m block a 0.80 m
    #     berth and the box reads clear 1.18 m early, with the block still
    #     dead abeam.
    #   * Anything else on the right fills it. Parked beside a block, it holds
    #     351 lidar points and the car has overtaken nothing.
    #
    # So pin the window to the lane instead. The pull-out records where the car
    # was and which way it was pointing; that is the centreline of the lane
    # being left. The window then rides along that lane beside the car, however
    # far out the car swings, and stays exactly as wide as the lane.
    #
    # Tracking the obstacle itself was tried first and is worse: an anchor that
    # re-centres on the nearest returns crawls along a long object instead of
    # holding station on it - measured here, the anchor ran 3.8 m up the side
    # of a block while the car moved 0.42 m, then fell off the end of it.

    def lane_frame(self, pts):
        """(points in lane coordinates, the car's own distance along it).

        x along the lane the pull-out started from, y across it, origin where
        the car left it. None until a pull-out has recorded one.
        """
        if self.lane is None or self.odom_xy is None:
            return None, None
        lx, ly, lyaw = self.lane
        c, sn = math.cos(-lyaw), math.sin(-lyaw)
        dx, dy = self.odom_xy[0] - lx, self.odom_xy[1] - ly
        car_s, car_t = c * dx - sn * dy, sn * dx + c * dy
        a = self.odom_yaw - lyaw                    # body -> lane rotation
        ca, sa = math.cos(a), math.sin(a)
        self.lane_pose = (car_s, car_t, a)
        return np.stack([ca * pts[:, 0] - sa * pts[:, 1] + car_s,
                         sa * pts[:, 0] + ca * pts[:, 1] + car_t], 1), car_s

    def lane_busy(self, pts):
        """Returns in the lane we came from, beside the car. None if no lane.

        Longitudinally the window is measured from the CAR, so it travels with
        it: from side_x_min behind (past the car's own tail) to side_x_max
        ahead (past its nose, so it does not merge into something it is only
        just drawing level with). Laterally it is the lane, and nothing else -
        which is what keeps the kerb and the far wall out of the decision.
        """
        lp, car_s = self.lane_frame(pts)
        if lp is None:
            return None
        return int((((lp[:, 0] >= car_s + self.side_x[0]) &
                     (lp[:, 0] <= car_s + self.side_x[1]) &
                     (np.abs(lp[:, 1]) <= self.lane_half_width))).sum())

    def record_lane(self, front=None):
        """Remember the lane being left, and where the obstacle was in it.

        The lane's origin is the car, so the obstacle's distance along the lane
        IS the forward distance that just triggered the pull-out.  Keeping it
        turns "have we gone far enough?" into a position in a fixed frame
        rather than a length of path, which is the difference between a test
        that knows where the obstacle was and one that only knows how far the
        car has driven since.
        """
        if self.odom_xy is None:
            self.lane = self.obstacle_s = None
            return
        self.lane = (self.odom_xy[0], self.odom_xy[1], self.odom_yaw)
        self.obstacle_s = None if front is None or not math.isfinite(front) else front
        self.get_logger().info(
            'watching the lane behind from here'
            + ('' if self.obstacle_s is None
               else f'; obstacle {self.obstacle_s:.2f} m along it'))

    def clearance(self):
        """How far the car is from the centreline of the lane it left [m]."""
        return None if self.lane is None else abs(self.lane_pose[1])

    def lane_to_body(self, x, y):
        """A point in lane coordinates, back in base_link - for the markers."""
        car_s, car_t, a = self.lane_pose
        dx, dy = x - car_s, y - car_t
        c, sn = math.cos(-a), math.sin(-a)
        return (c * dx - sn * dy, sn * dx + c * dy)

    def travelled(self):
        """Metres driven since entering avoid, or None without odometry."""
        if self.odom_xy is None or self.avoid_start_path is None:
            return None
        return self.odom_path - self.avoid_start_path

    # -- decision --------------------------------------------------------

    def tick(self):
        pts, silent = self.fresh_points()
        self.draw_windows(pts)
        if silent:
            self.get_logger().warn('no recent data on ' + ', '.join(silent),
                                   throttle_duration_sec=15.0)
        if pts is None:
            if not self.ever_seen:
                # Nothing has EVER arrived: this is a setup problem, and the
                # only safe thing to publish is one that stops the car.
                self.enter(STOP, 'no obstacle data on ' + ', '.join(self.topics))
                self.get_logger().error(
                    'no usable scan yet on %s - check the topics are publishing '
                    'and that TF %s <- their frames exists. Driving with no '
                    'obstacle sensing needs avoid:=false, not silence.'
                    % (', '.join(self.topics), self.robot_frame),
                    throttle_duration_sec=5.0)
            else:
                # A source that was working and went quiet: hold the current
                # state, because cutting back to `follow` in the middle of an
                # overtake steers into the thing being overtaken.  The follower
                # has its own watchdog for this node dying outright.
                self.get_logger().warn(
                    'scan dropout - holding state ' + self.state,
                    throttle_duration_sec=3.0)
            self.publish()
            return

        front = self.front_distance(pts)
        # What the trigger is actually comparing against. Echo it while driving
        # at the obstacle to pick trigger_distance from a reading instead of a
        # guess, and to tell "never saw it" apart from "saw it too late".
        self.front_pub.publish(Float32(data=float(min(front, 1e4))))
        if self.stop_distance > 0.0 and front <= self.stop_distance:
            self.enter(STOP, f'obstacle at {front:.2f} m')
        elif self.state == STOP:
            # The path opened (it drove off, it was noise, or the sensors
            # finally came up). What to resume depends on what the stop
            # interrupted, which is why `pre_stop` exists: something beside us
            # on the right is a reason not to CUT BACK mid-overtake, but it is
            # not a reason to START one - at boot the car can simply be parked
            # next to a wall, and pulling out for it would be a lane change
            # nothing ever asked for.
            if front <= self.trigger_distance:
                resumed = AVOID
            elif (self.pre_stop == AVOID
                  and (self.lane_busy(pts) or 0) >= self.min_hits):
                resumed = AVOID
            else:
                resumed = FOLLOW
            self.enter(resumed, f'path opened to {front:.2f} m')
            if resumed == AVOID and self.lane is None:
                self.record_lane(front)
        elif self.state == FOLLOW:
            if front <= self.trigger_distance:
                self.enter(AVOID, f'obstacle at {front:.2f} m')
                self.record_lane(front)
        elif self.state == AVOID:
            self.update_avoid(pts, front)

        self.publish()

    def update_avoid(self, pts, front):
        now = self.now()
        busy = self.lane_busy(pts)
        self.lane_hits = busy
        if busy is None:
            # No odometry, so there is no lane to hold the window in and
            # nothing to measure. pass_distance below is all that is left.
            self.get_logger().warn(
                'no odometry - cannot watch the lane we left',
                throttle_duration_sec=10.0)
        elif busy >= self.min_hits:
            if self.side_clear_since is not None or not self.side_seen:
                self.get_logger().info(
                    f'lane behind busy ({busy} pts, car {self.clearance():.2f} m out)')
            self.side_clear_since = None
            self.side_seen = True
        elif self.side_clear_since is None:
            self.side_clear_since = now
            self.get_logger().info(
                (f'lane behind clear (car {self.clearance():.2f} m out)'
                 if self.side_seen else
                 'lane behind empty - nothing to watch go past'))

        # The moment the obstacle stops being in front is the moment the car is
        # level with it, and that - not the moment the manoeuvre started - is
        # where "how much further" has to be measured from. Counting from the
        # trigger mixes in the approach, which is however far away the obstacle
        # happened to be seen: measured here, 1.3 m of a 3.0 m gate was spent
        # merely driving up to the thing, leaving 1.7 m of actual passing, so
        # the number had to be tuned per obstacle distance rather than per
        # obstacle. Counting from level, it means what it says - the obstacle's
        # length plus the car's.
        if self.level_path is None and front > self.trigger_distance:
            self.level_path = self.odom_path

        gone = self.travelled()
        if gone is None:
            self.get_logger().warn(
                'no odometry - returning to lane on the side check alone',
                throttle_duration_sec=10.0)
        elif gone >= self.max_avoid_distance:
            self.enter(FOLLOW, f'gave up after {gone:.2f} m')
            return

        if self.level_path is None:
            # Still something in front: whatever else is true, this is not the
            # moment to merge back.
            return
        # Nor is it, if the NEXT one is already within reach. Rejoining the lane
        # only to pull straight back out is not avoiding twice, it is weaving:
        # measured on a row of blocks, returns at 0.51-0.54 m past followed by a
        # fresh trigger 0.62-0.98 m later, over and over. A line of obstacles
        # should be passed in one move, so hold the lane we are in until the one
        # we came from is actually free to drive in.
        if front <= self.trigger_distance:
            return
        past = (self.odom_path - self.level_path) if gone is not None else None
        held = (now - self.side_clear_since) if self.side_clear_since else 0.0

        # And nor is it, until the car has actually GOT out of the way. A window
        # over the lane behind reads clear the moment the obstacle leaves it,
        # and it leaves the moment the car turns far enough - which can happen
        # while the car is still in its own lane with the obstacle dead ahead.
        # Measured on the track: pulled out at 0.99 m, merged back 1.0 m later
        # having never crossed the line, then triggered again at 0.60 m, then
        # 0.60 m again. That is not two overtakes, it is driving into the thing
        # in stages.
        #
        # So the manoeuvre has a shape, and this is the middle of it: out, hold
        # station out there, and only then look for a way back. Being this far
        # off the lane's centreline is what "out" means.
        out = self.clearance()
        if out is not None and out < self.min_clearance:
            self.get_logger().info(
                f'holding out - {out:.2f} m off the lane, want '
                f'{self.min_clearance:.2f}', throttle_duration_sec=2.0)
            return

        # Where the lane behind was watched, THAT decides. The window sits in
        # the lane the car left, runs from past its own tail to past its nose,
        # and is one lane wide - so nothing in it for side_clear_time means the
        # lane really is free to go back into, whatever the car did to get out
        # of it. That is the whole reason the lidar is here, being the only
        # sensor that can see abeam at all: the depth camera sees +/-33 deg of
        # straight ahead and never the side.
        #
        # pass_distance does NOT gate this. It used to, and then it decided
        # every return by itself - on the track, merges at 0.51, 0.54 and 0.53 m
        # against a 0.50 m floor - which left the side scan contributing
        # nothing. Merging back into a GAP between two obstacles is prevented by
        # the forward window above instead, which is the condition that actually
        # describes that danger; a distance floor only approximated it.
        # How far the car is beyond where the obstacle was, along the lane.
        beyond = (None if self.obstacle_s is None
                  else self.lane_pose[0] - self.obstacle_s)

        if self.side_seen:
            if held >= self.side_clear_time:
                self.enter(FOLLOW, f'lane behind clear for {held:.1f} s'
                           + ('' if beyond is None else f', {beyond:+.2f} m beyond it'))
            return

        # Nothing was ever seen abeam - an obstacle under the lidar plane looks
        # exactly like open road from the side - so there is no measurement to
        # wait for, and the distance is the whole of the evidence.
        #
        # It has to be distance BEYOND THE OBSTACLE, not distance driven. Path
        # length since the obstacle left the forward window says nothing about
        # where the obstacle is: the window is only 0.44 m wide, so the thing
        # leaves it the moment the car turns, while still well ahead. Measured
        # on a fixture, that let the car merge back 0.83 m into the manoeuvre
        # with the block still 1.55 m in front of it - and then trigger on the
        # same block all over again 0.4 s later.
        if beyond is None:
            if past is None or past >= self.pass_distance:
                self.enter(FOLLOW, 'never seen abeam, and no idea where it was'
                           + ('' if past is None else f', {past:.2f} m driven'))
        elif beyond >= self.pass_distance:
            self.enter(FOLLOW,
                       f'never seen abeam, {beyond:.2f} m beyond where it was')

    def enter(self, state, why):
        if state == self.state:
            return
        self.get_logger().info(f'{self.state} -> {state} ({why})')
        if state == STOP:
            self.pre_stop = self.state
        self.state = state
        self.side_clear_since = None
        # Re-entering avoid after a stop restarts the distance gate, which errs
        # towards staying out a little longer rather than merging back early.
        self.avoid_start_path = self.odom_path if state == AVOID else None
        self.level_path = None
        self.side_seen = False
        # A stop in the middle of an overtake is still that overtake, so the
        # lane is kept across it and only dropped on the way back into it.
        if state == FOLLOW:
            self.lane = self.lane_hits = self.obstacle_s = None

    def draw_windows(self, pts):
        """Publish the two windows, coloured by what is in them.

        Drawn on every tick, including the ticks with no scan at all - amber
        for "no data" rather than nothing at all. The moment the boxes are
        most worth looking at is the moment the sensors have gone quiet, and a
        display that disappears exactly then is no use.
        """
        if pts is None:
            front_busy = behind_busy = None
        else:
            front_busy = self.front_distance(pts) <= self.trigger_distance
            hits = self.lane_busy(pts)
            behind_busy = None if hits is None else hits >= self.min_hits
        arr = MarkerArray()
        arr.markers.append(self.box_marker(
            'front', 0, front_busy,
            [(0.0, -self.corridor_half_width),
             (self.trigger_distance, -self.corridor_half_width),
             (self.trigger_distance, self.corridor_half_width),
             (0.0, self.corridor_half_width)]))
        # The lane window is drawn where it actually is, which is the point of
        # it: it stays over the lane the car left while the car sits in the
        # other one, instead of swinging out with the bodywork.
        if self.lane is None:
            behind = None
        else:
            car_s = self.lane_pose[0]
            behind = [self.lane_to_body(car_s + sx, sy) for sx, sy in (
                (self.side_x[0], -self.lane_half_width),
                (self.side_x[1], -self.lane_half_width),
                (self.side_x[1], self.lane_half_width),
                (self.side_x[0], self.lane_half_width))]
        arr.markers.append(self.box_marker('behind', 1, behind_busy, behind))
        self.win_pub.publish(arr)

    def box_marker(self, name, ident, busy, corners):
        """One closed outline in base_link, or a DELETE when there is none."""
        m = Marker()
        m.header.frame_id = self.robot_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type = name, ident, Marker.LINE_STRIP
        if corners is None:
            # No lane recorded means no window - say so by removing it, rather
            # than leaving the last one on screen to be read as current.
            m.action = Marker.DELETE
            return m
        m.action = Marker.ADD
        m.scale.x = 0.015
        # Markers with no lifetime outlive the node that drew them. RViz then
        # shows the last set for ever, which reads exactly like a live display
        # that has stopped changing - boxes still on screen with nothing
        # publishing them at all.
        m.lifetime = Duration(seconds=1.0).to_msg()
        m.color.a = 1.0
        m.color.r, m.color.g, m.color.b = (
            (0.9, 0.7, 0.1) if busy is None else       # no scan at all
            (1.0, 0.2, 0.2) if busy else               # something in it
            (0.2, 1.0, 0.2))                           # clear
        m.pose.orientation.w = 1.0
        for px, py in list(corners) + [corners[0]]:
            p = Point()
            p.x, p.y, p.z = float(px), float(py), 0.02
            m.points.append(p)
        return m

    def publish(self):
        self.state_pub.publish(String(data=self.state))


def main():
    rclpy.init()
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Do not leave `avoid` as the last word on the topic: the follower holds
        # the most recent state until it goes stale, and a car parked in the
        # oncoming lane is the worst place to stop.
        node.state = FOLLOW
        node.publish()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
