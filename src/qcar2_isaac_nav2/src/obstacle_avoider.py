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
Everything this node decides comes from counting points in two boxes in
base_link (x forward, y LEFT - so the right-hand side is negative y):

    FRONT     |y| <= corridor_half_width,  x <= trigger_distance
              Something here means our lane is blocked.  A rectangle, not a
              range circle: a wall 1.2 m away at 60 deg is not in our way.

    RIGHT     side_x_min <= x <= side_x_max,  side_y_min <= -y <= side_y_max
              Used only while avoiding.  Empty for side_clear_time means the
              obstacle is behind us and the lane we came from is free again.
              It reaches AHEAD of the car, not just abeam, so we merge back
              once the road is clear rather than the instant the bumper clears.

              This window is the lidar's job: the depth camera's field of view
              is +/-33 deg and cannot see abeam at all.  So for an obstacle the
              lidar cannot see - anything below its 0.194 m plane - the window
              reads clear the whole way past and `min_avoid_distance` is the
              ONLY thing holding the car out.  Size that parameter to the
              longest obstacle you expect, not to the shortest manoeuvre.

A single stray ray sets nothing off: `min_hits` points must fall inside a window
before it counts as occupied.  An obstacle at 1.2 m subtends many rays on both
sensors; a lone hit is noise or a floor glint.

Getting out is gated, getting in is not
---------------------------------------
The moment the front window trips we go to `avoid`.  Coming back needs
`pass_distance` driven since the obstacle LEFT THE FRONT WINDOW, and then, if
the right window ever held anything during the manoeuvre, that window clear for
side_clear_time as well.

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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
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
        self.declare_parameter('side_y_min', 0.10)
        self.declare_parameter('side_y_max', 1.20)
        self.declare_parameter('side_clear_time', 0.60)
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
        self.side_y = (g('side_y_min').value, g('side_y_max').value)
        self.side_clear_time = g('side_clear_time').value
        self.pass_distance = g('pass_distance').value
        self.max_avoid_distance = g('max_avoid_distance').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # topic -> (stamp, Nx3 points in base_link).  Kept per source so one
        # dead sensor ages out on its own instead of poisoning the other.
        self.clouds = {}
        self.ever_seen = False
        self.odom_xy = None
        self.odom_path = 0.0       # cumulative path length, metres
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

    def right_side_busy(self, pts):
        """Is the lane we came from still occupied beside or ahead of us?"""
        y = -pts[:, 1]                                  # +y is left, so flip
        sel = ((pts[:, 0] >= self.side_x[0]) & (pts[:, 0] <= self.side_x[1]) &
               (y >= self.side_y[0]) & (y <= self.side_y[1]))
        return int(sel.sum()) >= self.min_hits

    def travelled(self):
        """Metres driven since entering avoid, or None without odometry."""
        if self.odom_xy is None or self.avoid_start_path is None:
            return None
        return self.odom_path - self.avoid_start_path

    # -- decision --------------------------------------------------------

    def tick(self):
        pts, silent = self.fresh_points()
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
            elif self.pre_stop == AVOID and self.right_side_busy(pts):
                resumed = AVOID
            else:
                resumed = FOLLOW
            self.enter(resumed, f'path opened to {front:.2f} m')
        elif self.state == FOLLOW:
            if front <= self.trigger_distance:
                self.enter(AVOID, f'obstacle at {front:.2f} m')
        elif self.state == AVOID:
            self.update_avoid(pts, front)

        self.publish()

    def update_avoid(self, pts, front):
        now = self.now()
        if self.right_side_busy(pts):
            self.side_clear_since = None
            self.side_seen = True
        elif self.side_clear_since is None:
            self.side_clear_since = now

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
        past = (self.odom_path - self.level_path) if gone is not None else None
        side_clear = (self.side_clear_since is not None and
                      now - self.side_clear_since >= self.side_clear_time)

        # Distance first, and it is never skipped.
        #
        # The side window cannot tell WHICH object it is holding. On a course
        # with blocks along the verge it is busy almost continuously, so the
        # first gap the car turns past reads as "the obstacle is behind us" -
        # measured on this track, returns at 0.28, 0.31 and 0.44 m past, each
        # followed immediately by the same obstacle re-entering the forward
        # window at 0.33-0.42 m. The car ends up flapping between lanes and
        # drifts off the paint. So the window gets a vote on when to come back,
        # never a veto on how soon.
        if past is not None and past < self.pass_distance:
            return
        # ...and where the window has something to say, it must agree. Where it
        # never saw the obstacle at all - anything under the lidar plane looks
        # exactly like open road from the side - "clear" carries no information
        # and the distance is the whole of the evidence.
        if self.side_seen and not side_clear:
            return
        why = ('right side clear' if self.side_seen else 'never seen abeam')
        self.enter(FOLLOW, why + ('' if past is None else f', {past:.2f} m past'))

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
