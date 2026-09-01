#!/usr/bin/env python3
"""Camera lane following for the QCar2 in Isaac Sim.

One file, two modes, ONE perception pipeline shared between them:

    ros2 launch qcar2_isaac_nav2 qcar2_lane_follow_launch.py tune:=true
        Opens an OpenCV window on the live camera with HSV sliders for the
        white edge lines and the blue centre line, drawn on top of exactly
        what the follower sees (ROI, look-ahead band, detected boundaries,
        target point).  Press 's' to save, 'q' to quit.  Publishes nothing.

    ros2 launch qcar2_isaac_nav2 qcar2_lane_follow_launch.py
        Same pipeline, no GUI, and the lateral error is turned into a
        steering command on /cmd_vel_twist.

The tuner and the follower run the *same* LaneDetector, so what you tuned is
literally what drives the car - there is no second copy of the thresholds to
drift out of step.

Where the tuned values live
---------------------------
The tuner writes ~/.ros/qcar2_lane_colors.yaml, NOT config/lane_colors.yaml in
the source tree.  config/ is installed by CMake, so a value edited there does
nothing until the next `colcon build` - the classic silent no-op in this
workspace.  Writing to ~/.ros keeps the tune-drive loop a two-command cycle,
the same way the V-SLAM route keeps its database there.  The launch file uses
~/.ros/qcar2_lane_colors.yaml when it exists and falls back to the packaged
config/lane_colors.yaml otherwise, and both are plain ROS 2 parameter files, so
nothing in this node parses YAML itself.

Why this bypasses Nav2
----------------------
Lane following IS a controller: it produces a steering command from what the
camera sees, with no map, no costmap and no goal pose.  Feeding it through Nav2
would mean inventing goal poses on a map we are deliberately not using.  So the
node publishes geometry_msgs/Twist straight to the Isaac Sim drive graph on
/cmd_vel_twist, the same topic twist_stamped_to_twist.py feeds.  Consequently:

    Nav2 and this node must not run at the same time - two publishers on
    /cmd_vel_twist fight over the steering.

angular.z here is a front-wheel STEERING ANGLE in radians, not a yaw rate -
that is what the Isaac QCar2 drive graph reads (see twist_stamped_to_twist.py
for the measurements behind that claim).  Positive is a left turn, so a lane
centre that sits to the right of the image centre produces a negative angular.z.

Which line to hug (right-hand traffic)
--------------------------------------
The map has two kinds of road and one rule covers both, because the rule is
about the nearest line on each side rather than about the road type:

    1 lane, white on both sides      ->  drive midway between the two whites
    2 lanes, blue centre line        ->  blue is one boundary, white the other

The only place the traffic side matters is when a single boundary is visible.
Then the car is held half a lane width from it - and if that lone boundary is
the BLUE centre line seen on our right, we are in the oncoming lane, so the
target is placed on the far side of it to get back across.

cv_bridge is deliberately not used
----------------------------------
Same reason as yolo_detector.py: Jazzy's cv_bridge is built against NumPy 1.x
and segfaults with no traceback when Isaac Sim's NumPy is first on PYTHONPATH.
An 8-bit colour Image is height*width*3 bytes plus a row stride, so decoding is
a few lines of NumPy.  cv2 itself is only used for colour conversion, masking
and drawing, which do not cross that boundary.
"""

import os
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

DEFAULT_COLORS_FILE = os.path.expanduser('~/.ros/qcar2_lane_colors.yaml')

# Tint used to show each mask in the debug overlay.  Deliberately NOT white and
# blue: a white mask drawn in white is invisible on the very pixels it matched.
MASK_TINT = {'white': (255, 0, 255), 'blue': (0, 255, 255)}


# --------------------------------------------------------------------------
# image codec (no cv_bridge - see module docstring)
# --------------------------------------------------------------------------

def decode_rgb(msg):
    """sensor_msgs/Image -> HxWx3 uint8 RGB."""
    if msg.encoding in ('rgb8', 'bgr8'):
        channels = 3
    elif msg.encoding in ('rgba8', 'bgra8'):
        channels = 4
    else:
        raise ValueError(f'unsupported encoding {msg.encoding!r}')
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    # `step` is the row stride in bytes and is not always width*channels; a
    # padded row reshaped on width alone shears the image.
    img = buf.reshape(msg.height, msg.step // channels, channels)
    img = img[:, :msg.width, :3]
    if msg.encoding.startswith('bgr'):
        img = img[:, :, ::-1]
    return np.ascontiguousarray(img)


def encode_rgb(img, header):
    """HxWx3 uint8 RGB -> sensor_msgs/Image."""
    msg = Image()
    msg.header = header
    msg.height, msg.width = img.shape[:2]
    msg.encoding = 'rgb8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(img, dtype=np.uint8).tobytes()
    return msg


# --------------------------------------------------------------------------
# perception
# --------------------------------------------------------------------------

class LaneReading:
    """What one frame told us.  All x values are image columns in pixels."""

    __slots__ = ('left_x', 'left_color', 'right_x', 'right_color', 'target_x',
                 'error', 'roi_top_px', 'band', 'half_width', 'masks', 'note')

    def __init__(self):
        self.left_x = None
        self.left_color = None
        self.right_x = None
        self.right_color = None
        self.target_x = None
        self.error = None          # normalised, -1 (lane is far left) .. +1
        self.roi_top_px = 0
        self.band = (0, 0)         # (y0, y1) of the look-ahead band, full-image
        self.half_width = 0.0
        self.masks = {}            # name -> ROI-sized uint8 mask
        self.note = ''


def _runs(col_hit, min_px):
    """Contiguous True runs in a 1-D boolean column profile.

    Returns (starts, ends) as float arrays, ends exclusive.  Runs narrower than
    `min_px` are dropped: a lane line is several pixels wide even far away, and
    single-pixel hits are almost always specular glints off the floor.
    """
    padded = np.concatenate(([0], col_hit.astype(np.int8), [0]))
    edges = np.flatnonzero(np.diff(padded))
    starts, ends = edges[0::2].astype(float), edges[1::2].astype(float)
    keep = (ends - starts) >= min_px
    return starts[keep], ends[keep]


def _inner_edges(col_hit, centre, min_px):
    """Nearest line on each side, reported by its edge facing the car.

    The inner edge - not the run's midpoint - is what bounds the drivable
    corridor, so centring between inner edges stays correct when a near line is
    thirty pixels wide and the far one is three.
    """
    starts, ends = _runs(col_hit, min_px)
    if starts.size == 0:
        return None, None
    mids = 0.5 * (starts + ends)
    left = right = None
    li = np.flatnonzero(mids < centre)
    if li.size:
        left = float(ends[li[np.argmax(mids[li])]])
    ri = np.flatnonzero(mids >= centre)
    if ri.size:
        right = float(starts[ri[np.argmin(mids[ri])]])
    return left, right


class LaneDetector:
    """RGB frame -> lateral error, for both the tuner and the follower.

    Stateful in one respect only: the lane half width is learned online (EMA)
    whenever both boundaries are visible, so the single-boundary case uses a
    measured offset instead of a guessed one.  Perspective makes that width
    depend on the look-ahead row, which is why the band is fixed rather than
    swept - one band, one width, no homography to calibrate.
    """

    def __init__(self, white_low, white_high, blue_low, blue_high,
                 roi_top=0.55, band_frac=0.35, min_fill=0.4, min_run_px=3,
                 lane_half_width_frac=0.30):
        self.bounds = {'white': [np.array(white_low, np.uint8),
                                 np.array(white_high, np.uint8)],
                       'blue': [np.array(blue_low, np.uint8),
                                np.array(blue_high, np.uint8)]}
        self.roi_top = roi_top
        self.band_frac = band_frac
        self.min_fill = min_fill
        self.min_run_px = min_run_px
        self.lane_half_width_frac = lane_half_width_frac
        self.half_width = None
        self._kernel = np.ones((3, 3), np.uint8)

    def set_bounds(self, color, low, high):
        self.bounds[color] = [np.array(low, np.uint8), np.array(high, np.uint8)]

    def detect(self, rgb):
        h, w = rgb.shape[:2]
        centre = w * 0.5
        if self.half_width is None:
            self.half_width = self.lane_half_width_frac * w

        r = LaneReading()
        r.roi_top_px = y0 = int(self.roi_top * h)
        roi = rgb[y0:]
        rh = roi.shape[0]
        if rh < 8:
            r.note = 'roi_top too high'
            return r

        # The look-ahead band is measured up from the bottom of the ROI: the
        # bottom row is under the bumper and steers far too late, the top of
        # the ROI is horizon and noise.
        band_h = max(5, rh // 8)
        by1 = int(rh - self.band_frac * rh)
        by0 = max(0, by1 - band_h)
        by1 = max(by0 + 5, by1)
        r.band = (y0 + by0, y0 + by1)

        hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)
        need = max(1, int(self.min_fill * (by1 - by0)))
        found = {}
        for name, (low, high) in self.bounds.items():
            mask = cv2.inRange(hsv, low, high)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
            r.masks[name] = mask
            hits = (mask[by0:by1] > 0).sum(axis=0)
            found[name] = _inner_edges(hits >= need, centre, self.min_run_px)

        white_l, white_r = found['white']
        blue_l, blue_r = found['blue']

        # Nearest boundary on each side, whatever colour painted it.
        left = max(((x, c) for x, c in ((white_l, 'white'), (blue_l, 'blue'))
                    if x is not None), key=lambda t: t[0], default=(None, None))
        right = min(((x, c) for x, c in ((white_r, 'white'), (blue_r, 'blue'))
                     if x is not None), key=lambda t: t[0], default=(None, None))
        r.left_x, r.left_color = left
        r.right_x, r.right_color = right

        if r.left_x is not None and r.right_x is not None:
            # Both sides visible: this is the only situation that measures the
            # lane, so it is the only one allowed to update the width.
            r.target_x = 0.5 * (r.left_x + r.right_x)
            self.half_width = 0.9 * self.half_width + \
                0.1 * max(1.0, 0.5 * (r.right_x - r.left_x))
            r.note = f'{r.left_color}|{r.right_color}'
        elif r.right_color == 'blue':
            # Right-hand traffic: the blue centre line belongs on our LEFT.
            # Seeing it alone on the right means we are in the oncoming lane,
            # so aim across it rather than centring on the wrong side.
            r.target_x = r.right_x + self.half_width
            r.note = 'blue on right - crossing back'
        elif r.left_x is not None:
            r.target_x = r.left_x + self.half_width
            r.note = f'{r.left_color} left only'
        elif r.right_x is not None:
            r.target_x = r.right_x - self.half_width
            r.note = f'{r.right_color} right only'
        else:
            r.note = 'no lane'
            return r

        r.half_width = self.half_width
        r.error = float((r.target_x - centre) / centre)
        return r


def draw_overlay(rgb, r):
    """Annotate a copy of the frame with everything `detect` used and decided."""
    out = rgb.copy()
    h, w = out.shape[:2]
    y0 = r.roi_top_px
    roi = out[y0:]
    for name, mask in r.masks.items():
        sel = mask > 0
        if sel.any():
            roi[sel] = (0.35 * roi[sel] +
                        0.65 * np.array(MASK_TINT[name], np.float32)).astype(np.uint8)
    cv2.line(out, (0, y0), (w, y0), (255, 160, 0), 1)
    cv2.rectangle(out, (0, r.band[0]), (w - 1, r.band[1]), (255, 160, 0), 1)
    cv2.line(out, (w // 2, y0), (w // 2, h - 1), (120, 120, 120), 1)
    by = (r.band[0] + r.band[1]) // 2
    for x, colour in ((r.left_x, (0, 255, 0)), (r.right_x, (0, 255, 0))):
        if x is not None:
            cv2.line(out, (int(x), r.band[0]), (int(x), r.band[1]), colour, 2)
    if r.target_x is not None:
        cv2.circle(out, (int(r.target_x), by), 6, (255, 0, 0), -1)
        cv2.line(out, (w // 2, h - 1), (int(r.target_x), by), (255, 0, 0), 2)
    txt = r.note if r.error is None else f'{r.note}  e={r.error:+.3f}'
    cv2.putText(out, txt, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 0), 2, cv2.LINE_AA)
    return out


# --------------------------------------------------------------------------
# shared node setup
# --------------------------------------------------------------------------

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)


def declare_detector(node):
    """Declare the perception parameters and build the detector from them.

    These are exactly the values the tuner writes, so the follower picks them
    up as an ordinary ROS parameter file with no custom loader.
    """
    node.declare_parameter('white_hsv_low', [0, 0, 170])
    node.declare_parameter('white_hsv_high', [179, 60, 255])
    node.declare_parameter('blue_hsv_low', [95, 90, 60])
    node.declare_parameter('blue_hsv_high', [130, 255, 255])
    node.declare_parameter('roi_top', 0.55)
    node.declare_parameter('band_frac', 0.35)
    node.declare_parameter('min_fill', 0.4)
    node.declare_parameter('min_run_px', 3)
    node.declare_parameter('lane_half_width_frac', 0.30)
    g = node.get_parameter
    return LaneDetector(
        g('white_hsv_low').value, g('white_hsv_high').value,
        g('blue_hsv_low').value, g('blue_hsv_high').value,
        roi_top=g('roi_top').value, band_frac=g('band_frac').value,
        min_fill=g('min_fill').value, min_run_px=g('min_run_px').value,
        lane_half_width_frac=g('lane_half_width_frac').value)


# --------------------------------------------------------------------------
# drive mode
# --------------------------------------------------------------------------

class LaneFollower(Node):

    def __init__(self):
        super().__init__('lane_follower')
        self.detector = declare_detector(self)

        self.declare_parameter('image_topic', '/csi_front/image_raw')
        self.declare_parameter('cmd_topic', '/cmd_vel_twist')
        self.declare_parameter('speed', 0.30)
        self.declare_parameter('kp', 0.70)
        self.declare_parameter('kd', 0.10)
        # Matches twist_stamped_to_twist.py; Isaac Sim clamps around 0.5 rad.
        self.declare_parameter('max_steering_angle', 0.50)
        self.declare_parameter('lost_timeout', 0.5)
        self.declare_parameter('publish_debug', True)

        g = self.get_parameter
        self.speed = g('speed').value
        self.kp = g('kp').value
        self.kd = g('kd').value
        self.max_steer = g('max_steering_angle').value
        self.lost_timeout = g('lost_timeout').value
        self.publish_debug = g('publish_debug').value

        self.prev_error = None
        self.prev_time = None
        self.last_good = None      # last time a lane was seen, seconds
        self.stopped = True

        self.cmd_pub = self.create_publisher(Twist, g('cmd_topic').value, 10)
        self.dbg_pub = (self.create_publisher(Image, '/lane/debug_image', 1)
                        if self.publish_debug else None)
        self.sub = self.create_subscription(
            Image, g('image_topic').value, self.on_image, SENSOR_QOS)
        # The image callback is the control loop, but a dead camera would
        # otherwise leave the last command latched in the drive graph forever.
        self.create_timer(0.1, self.watchdog)

        self.get_logger().info(
            f"lane follower: {g('image_topic').value} -> {g('cmd_topic').value} "
            f'(steering angle rad), speed={self.speed:.2f} m/s, '
            f'kp={self.kp:.2f} kd={self.kd:.2f}, '
            f'max_steer={self.max_steer:.2f} rad')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_image(self, msg):
        try:
            rgb = decode_rgb(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return

        r = self.detector.detect(rgb)
        t = self.now()

        if r.error is None:
            # Say nothing on the command topic: the drive graph keeps driving on
            # the last Twist, and a lane line dropping out for a frame or two is
            # normal at 10 Hz - braking on every dropout makes the car crawl.
            # The watchdog stops us if it persists past lost_timeout.
            self.get_logger().warn('lane lost: ' + r.note,
                                   throttle_duration_sec=2.0)
        else:
            self.last_good = t
            dt = (t - self.prev_time) if self.prev_time else 0.0
            derr = ((r.error - self.prev_error) / dt
                    if dt > 1e-3 and self.prev_error is not None else 0.0)
            self.prev_error, self.prev_time = r.error, t

            # +angular.z is a left turn, so a lane centre to the RIGHT of the
            # image centre (error > 0) must produce a negative steering angle.
            steer = -(self.kp * r.error + self.kd * derr)
            steer = max(-self.max_steer, min(self.max_steer, steer))
            # Ease off in the corners: at full lock the car is already at the
            # edge of what 10 Hz of vision can correct.
            speed = self.speed * (1.0 - 0.5 * abs(steer) / self.max_steer)

            cmd = Twist()
            cmd.linear.x = speed
            cmd.angular.z = steer
            self.stopped = False
            self.cmd_pub.publish(cmd)

        if self.dbg_pub is not None:
            self.dbg_pub.publish(encode_rgb(draw_overlay(rgb, r), msg.header))

    def watchdog(self):
        """Stop when the lane - or the camera - has been gone too long.

        This never re-sends the driving command.  The Isaac drive graph holds
        the last Twist it received, so a keep-alive would only double the rate
        on /cmd_vel_twist and interleave stale commands with fresh ones.  Its
        one job is the transition to stopped.
        """
        if self.stopped:
            return
        if self.last_good is None or self.now() - self.last_good > self.lost_timeout:
            self.cmd_pub.publish(Twist())
            self.stopped = True
            self.get_logger().warn('no lane for %.2f s - stopping' % self.lost_timeout)


# --------------------------------------------------------------------------
# tune mode
# --------------------------------------------------------------------------

TRACKBARS = (('H min', 179), ('H max', 179), ('S min', 255), ('S max', 255),
             ('V min', 255), ('V max', 255))
WINDOW = 'qcar2 lane tuner'


def _fix_qt_fonts():
    """Point OpenCV's bundled Qt at a font directory that exists.

    The pip opencv wheel sets QT_QPA_FONTDIR to a `qt/fonts` directory it does
    not actually ship, so Qt loads no fonts and every trackbar label renders
    blank - which makes an HSV tuner unusable.  Qt reads the variable when the
    GUI starts rather than when cv2 is imported, so setting it here is in time.
    """
    if os.path.isdir(os.environ.get('QT_QPA_FONTDIR', '')):
        return
    for path in ('/usr/share/fonts/truetype/dejavu',
                 '/usr/share/fonts/truetype/liberation', '/usr/share/fonts'):
        if os.path.isdir(path):
            os.environ['QT_QPA_FONTDIR'] = path
            return


class LaneTuner(Node):
    """Interactive HSV tuning on the live camera, writing a ROS parameter file.

    Publishes nothing.  It shows the follower's own overlay rather than a bare
    mask, because the question being answered is not "does this threshold catch
    the paint" but "does the car end up aiming down the lane".
    """

    def __init__(self):
        super().__init__('lane_tuner')
        self.detector = declare_detector(self)
        self.declare_parameter('image_topic', '/csi_front/image_raw')
        self.declare_parameter('colors_out', DEFAULT_COLORS_FILE)
        self.out_path = os.path.expanduser(self.get_parameter('colors_out').value)
        self.frame = None
        self.sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self.on_image, SENSOR_QOS)

    def on_image(self, msg):
        try:
            self.frame = decode_rgb(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)

    # -- gui ---------------------------------------------------------------

    def build_window(self):
        _fix_qt_fonts()
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        noop = lambda _v: None                                  # noqa: E731
        cv2.createTrackbar('colour 0=white 1=blue', WINDOW, 0, 1, noop)
        for name, hi in TRACKBARS:
            cv2.createTrackbar(name, WINDOW, 0, hi, noop)
        cv2.createTrackbar('roi top %', WINDOW, int(self.detector.roi_top * 100),
                           95, noop)
        cv2.createTrackbar('band %', WINDOW, int(self.detector.band_frac * 100),
                           90, noop)
        self.load_bars('white')

    def load_bars(self, color):
        low, high = self.detector.bounds[color]
        for name, value in zip((n for n, _ in TRACKBARS),
                               (low[0], high[0], low[1], high[1], low[2], high[2])):
            cv2.setTrackbarPos(name, WINDOW, int(value))

    def read_bars(self):
        v = [cv2.getTrackbarPos(n, WINDOW) for n, _ in TRACKBARS]
        return ([v[0], v[2], v[4]], [v[1], v[3], v[5]])

    def run(self):
        self.build_window()
        color = 'white'
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            picked = 'blue' if cv2.getTrackbarPos(
                'colour 0=white 1=blue', WINDOW) else 'white'
            if picked != color:
                # Switching colour reloads the sliders from the stored bounds;
                # reading them on this pass would overwrite the new colour with
                # the old colour's slider positions.
                color = picked
                self.load_bars(color)
            else:
                self.detector.set_bounds(color, *self.read_bars())
            self.detector.roi_top = cv2.getTrackbarPos('roi top %', WINDOW) / 100.0
            self.detector.band_frac = cv2.getTrackbarPos('band %', WINDOW) / 100.0

            if self.frame is None:
                self.get_logger().warn('waiting for images...',
                                       throttle_duration_sec=3.0)
                if cv2.waitKey(30) & 0xFF in (ord('q'), 27):
                    break
                continue

            r = self.detector.detect(self.frame)
            view = draw_overlay(self.frame, r)
            cv2.putText(view, f"tuning {color}  [s]ave  [q]uit",
                        (8, view.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow(WINDOW, view[:, :, ::-1])          # imshow wants BGR
            mask = r.masks.get(color)
            if mask is not None:
                cv2.imshow(f'{color} mask', mask)

            key = cv2.waitKey(30) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('s'):
                self.save()
        cv2.destroyAllWindows()

    def save(self):
        b = self.detector.bounds
        os.makedirs(os.path.dirname(self.out_path), exist_ok=True)
        with open(self.out_path, 'w') as fh:
            fh.write(
                '# Lane colours for qcar2_isaac_nav2, written by\n'
                '#   ros2 launch qcar2_isaac_nav2 qcar2_lane_follow_launch.py tune:=true\n'
                '# A ROS 2 parameter file: the follower loads it directly.\n'
                '# HSV is OpenCV\'s: H 0-179, S 0-255, V 0-255.\n'
                '/**:\n'
                '  ros__parameters:\n'
                f'    white_hsv_low: {list(map(int, b["white"][0]))}\n'
                f'    white_hsv_high: {list(map(int, b["white"][1]))}\n'
                f'    blue_hsv_low: {list(map(int, b["blue"][0]))}\n'
                f'    blue_hsv_high: {list(map(int, b["blue"][1]))}\n'
                f'    roi_top: {self.detector.roi_top:.2f}\n'
                f'    band_frac: {self.detector.band_frac:.2f}\n')
        self.get_logger().info(f'saved {self.out_path}')


# --------------------------------------------------------------------------

def main():
    argv = remove_ros_args(sys.argv)
    rclpy.init()
    if '--tune' in argv:
        node = LaneTuner()
        try:
            node.run()
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
        return

    node = LaneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the car stopped, not latched on the last steering angle.
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
