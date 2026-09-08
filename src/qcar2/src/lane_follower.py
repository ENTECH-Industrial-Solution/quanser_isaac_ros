#!/usr/bin/env python3
"""Camera lane following for the QCar2 in Isaac Sim.

One file, two modes, ONE perception pipeline shared between them:

    ros2 launch qcar2 qcar2_lane_follow_launch.py tune:=true
        Opens an OpenCV window on the live camera with HSV sliders for the
        white edge lines and the blue centre line, drawn on top of exactly
        what the follower sees (ROI, look-ahead band, detected boundaries,
        target point).  Press 's' to save, 'q' to quit.  Publishes nothing.

    ros2 launch qcar2 qcar2_lane_follow_launch.py
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

Two look-ahead bands, because offset alone is not enough
--------------------------------------------------------
The detector reads the lane at TWO distances, not one.  The near band gives the
lateral error - how far off centre the car is.  The pair gives the lane's
direction: the same boundary seen at both distances moves sideways between them
by an amount that is the lane's angle relative to the car.

A controller steering on offset alone cannot tell a car parked off to one side
from a car driving across the lane at speed, so it holds the turn until the aim
point is centred - by which time the car is yawed and carries on going.
Measured on a 0.66 m lane: a lane change overshot the target lane centre by
0.33 m and settled 32 degrees off.  Feeding the heading in as well brings that
to 0.02 m.  It is the difference between arriving in the next lane and crossing
it.

Which line to hug (right-hand traffic)
--------------------------------------
The map has two kinds of road and one rule covers both, because the rule is
about the nearest line on each side rather than about the road type:

    1 lane, white on both sides      ->  drive midway between the two whites
    2 lanes, blue centre line        ->  blue is one boundary, white the other

The only place the traffic side matters is when a single boundary is visible.
Then the car is held half a lane width from it.  Seeing the BLUE centre line on
our right means we are in the oncoming lane whatever else is visible, so that
case is checked first and the target is placed on the far side of the line to
get back across.

Overtaking, when obstacle_avoider.py says so
--------------------------------------------
obstacle_avoider.py publishes one word on /lane/avoid:

    follow   drive our own lane (also what is assumed when nobody publishes)
    avoid    hold the LEFT lane - there is something in ours
    stop     something is too close to drive round

`avoid` does not inject a steering offset or run a timed manoeuvre.  It swaps
which line the target is measured from, and which line that is depends on the
road - the same distinction the table above already makes:

    two lanes, blue centre line   half a lane LEFT of the centre line.  Read
                                  from our own lane, where the line is on our
                                  left, that asks for a full lane of crossing;
                                  read from the oncoming lane, where the same
                                  line is now on our right, it asks for nothing.

    one carriageway, white edges  `avoid_offset_frac` of a half width from the
                                  LEFT edge instead of the usual whole one.
                                  There is no left lane here and crossing the
                                  edge line drives off the road, so the room to
                                  pass is inside the corridor we are in.

Which of the two applies is `blue_is_centre_line`, and it defaults to FALSE.
"Blue on my right means I am in the oncoming lane" is a claim about the road, not
about the picture, and it is only true where a blue line actually divides two
directions of traffic.  Measured on the Isaac map this package ships against, it
is not: the lines there are white paint with a faint blue cast (H 104, S 37) and
the tuned blue window starts at S 15, so ordinary edge lines register as blue.
The rule then reads every one of them as "you are on the wrong side" and asks for
a target a lane width beyond the picture - an error of 1.12, i.e. full lock, held
for as long as the line is in view.  So the two-lane behaviour is opt-in, and
without it blue is simply another boundary colour.

Either way the target is a fixed distance from a line the camera can see, so the
manoeuvre ends by itself where it should and holds there, with no timing to tune.
Dropping back to `follow` hands the car to the crossing-back rule above (two
lanes) or to plain centring (one carriageway), which brings it home the same way.
Every phase is closed loop.

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
from std_msgs.msg import String

DEFAULT_COLORS_FILE = os.path.expanduser('~/.ros/qcar2_lane_colors.yaml')

# The /lane/avoid vocabulary.  Repeated verbatim in obstacle_avoider.py: the two
# nodes are separate executables in lib/, not an importable package, and this is
# a wire contract rather than shared code.
FOLLOW, AVOID, STOP = 'follow', 'avoid', 'stop'

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
                 'error', 'heading', 'roi_top_px', 'band', 'band_far',
                 'half_width', 'masks', 'note')

    def __init__(self):
        self.left_x = None
        self.left_color = None
        self.right_x = None
        self.right_color = None
        self.target_x = None
        self.error = None          # normalised, -1 (lane is far left) .. +1
        self.heading = 0.0         # + when the lane runs away to the right,
                                   # i.e. when the car is yawed left
        self.roi_top_px = 0
        self.band = (0, 0)         # (y0, y1) of the look-ahead band, full-image
        self.band_far = None       # the second band, when there is room for it
        self.half_width = 0.0
        self.masks = {}            # name -> ROI-sized uint8 mask
        self.note = ''


def _runs(col_hit, min_px, max_px):
    """Contiguous True runs in a 1-D boolean column profile.

    Returns (starts, ends) as float arrays, ends exclusive.  Runs narrower than
    `min_px` are dropped: a lane line is several pixels wide even far away, and
    single-pixel hits are almost always specular glints off the floor.

    Runs WIDER than `max_px` are dropped for the mirror-image reason, and it is
    the one that crashes cars: a white obstacle, a wall, or a patch of
    blown-out floor is a white run hundreds of pixels across in the look-ahead
    band.  Without this, a white box standing in the lane becomes the nearest
    "boundary" and the car aims a lane width to one side of the thing it is
    about to hit - which looks exactly like a late, panicky swerve.
    """
    padded = np.concatenate(([0], col_hit.astype(np.int8), [0]))
    edges = np.flatnonzero(np.diff(padded))
    starts, ends = edges[0::2].astype(float), edges[1::2].astype(float)
    width = ends - starts
    keep = (width >= min_px) & (width <= max_px)
    return starts[keep], ends[keep]


def _inner_edges(col_hit, centre, min_px, max_px):
    """Nearest line on each side, reported by its edge facing the car.

    The inner edge - not the run's midpoint - is what bounds the drivable
    corridor, so centring between inner edges stays correct when a near line is
    thirty pixels wide and the far one is three.
    """
    starts, ends = _runs(col_hit, min_px, max_px)
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
    whenever both boundaries are visible AND the width they imply is within
    `half_width_tolerance` of the configured `lane_half_width_frac`, so the
    single-boundary case uses a measured offset instead of a guessed one without
    letting one bad frame redefine what a lane is.  Perspective makes that width
    depend on the look-ahead row, which is why the band is fixed rather than
    swept - one band, one width, no homography to calibrate.
    """

    def __init__(self, white_low, white_high, blue_low, blue_high,
                 roi_top=0.55, band_frac=0.35, min_fill=0.4, min_run_px=3,
                 lane_half_width_frac=0.30, max_run_frac=0.25,
                 avoid_offset_frac=0.45, blue_is_centre_line=False,
                 half_width_tolerance=2.0, heading_band_gap=0.20,
                 horizon_frac=None):
        self.bounds = {'white': [np.array(white_low, np.uint8),
                                 np.array(white_high, np.uint8)],
                       'blue': [np.array(blue_low, np.uint8),
                                np.array(blue_high, np.uint8)]}
        self.roi_top = roi_top
        self.band_frac = band_frac
        self.min_fill = min_fill
        self.min_run_px = min_run_px
        self.lane_half_width_frac = lane_half_width_frac
        self.max_run_frac = max_run_frac
        self.avoid_offset_frac = avoid_offset_frac
        self.blue_is_centre_line = blue_is_centre_line
        self.half_width_tolerance = max(1.0, half_width_tolerance)
        self.heading_band_gap = heading_band_gap
        # Where the road's vanishing point sits vertically. It is a property of
        # how the camera is mounted, not of the scene, and it is the row the ROI
        # is already cut just below - so roi_top is the right default.
        self.horizon_frac = roi_top if horizon_frac is None else horizon_frac
        self.half_width = None
        self._kernel = np.ones((3, 3), np.uint8)

    def set_bounds(self, color, low, high):
        self.bounds[color] = [np.array(low, np.uint8), np.array(high, np.uint8)]

    def _scan_band(self, masks, by0, by1, centre, need, w):
        """Nearest boundary each side of one band, plus the blue edges alone."""
        found = {}
        for name, mask in masks.items():
            hits = (mask[by0:by1] > 0).sum(axis=0)
            found[name] = _inner_edges(hits >= need, centre, self.min_run_px,
                                       self.max_run_frac * w)
        white_l, white_r = found['white']
        blue_l, blue_r = found['blue']
        left = max(((x, c) for x, c in ((white_l, 'white'), (blue_l, 'blue'))
                    if x is not None), key=lambda t: t[0], default=(None, None))
        right = min(((x, c) for x, c in ((white_r, 'white'), (blue_r, 'blue'))
                     if x is not None), key=lambda t: t[0], default=(None, None))
        return left, right, blue_l, blue_r

    def detect(self, rgb, mode=FOLLOW):
        h, w = rgb.shape[:2]
        centre = w * 0.5
        # What counts as a believable lane. The online estimate is only allowed
        # to move inside this, and measurements outside it are thrown away
        # rather than averaged in - see the update below.
        lo = self.lane_half_width_frac * w / self.half_width_tolerance
        hi = self.lane_half_width_frac * w * self.half_width_tolerance
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
            r.masks[name] = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)

        left, right, blue_l, blue_r = self._scan_band(r.masks, by0, by1, centre,
                                                      need, w)
        r.left_x, r.left_color = left
        r.right_x, r.right_color = right

        # Second, further band - the two together give the lane's DIRECTION.
        # A single band can only say how far off centre the car is; it cannot
        # tell a car sitting off to one side from a car driving across the lane,
        # and steering on offset alone therefore keeps turning after the car has
        # arrived. Measured: a lane change that should have stopped at the lane
        # centre carried 0.33 m past it on a 0.66 m lane.
        #
        # The far band is HALF as tall, and that is not a detail. Perspective
        # compresses distance into rows, so the same row count covers a much
        # longer stretch of road further out - here 23 rows span 0.40-0.52 m
        # near and 0.65-1.06 m far. Over that longer stretch a lane line slants
        # across ~70 columns, and no single column is then lit for the min_fill
        # fraction of the band's rows, so the far band finds nothing at all and
        # the heading reads a constant zero. A thinner band keeps the slant
        # inside it small enough to register.
        far_h = max(5, band_h // 2)
        fy1 = int(rh - min(0.95, self.band_frac + self.heading_band_gap) * rh)
        fy0 = fy1 - far_h
        if fy0 >= 0 and fy1 > fy0:
            r.band_far = (y0 + fy0, y0 + fy1)
            f_need = max(1, int(self.min_fill * (fy1 - fy0)))
            f_left, f_right, _, _ = self._scan_band(r.masks, fy0, fy1, centre,
                                                    f_need, w)
            # The VANISHING POINT, not the change in position.
            #
            # Yawing the camera translates the whole image sideways by
            # F*tan(yaw), the same at every depth, so how far a line moves
            # between two bands says nothing about heading - it measures how far
            # off centre the car is, and averaging the two sides cancels even
            # that. Measured: +88 px on the left boundary against -88 px on the
            # right, summing to exactly zero at every yaw angle.
            #
            # What the yaw does move is where the lane's lines CONVERGE. Each
            # boundary is a straight line in the image, so two samples give its
            # slope, and extrapolating to the horizon row gives the point they
            # aim at. That point sits at the image centre when the car is
            # aligned with the lane and F*tan(yaw) off it when it is not -
            # checked against this geometry to the pixel.
            horizon = self.horizon_frac * h
            vps = []
            for n, f in ((left, f_left), (right, f_right)):
                if n[0] is None or f[0] is None:
                    continue
                drow = (fy1 + fy0) / 2.0 - (by1 + by0) / 2.0
                if abs(drow) < 1.0:
                    continue
                slope = (f[0] - n[0]) / drow
                vps.append(n[0] + slope * (horizon - y0 - (by1 + by0) / 2.0))
            if vps:
                r.heading = float((sum(vps) / len(vps) - centre) / centre)

        # The centre line, whichever side of us it currently is.  Both halves
        # of an overtake are measured from THIS line rather than from the lane
        # we happen to be sitting in, which is what makes them closed loop: the
        # error only reaches zero in the lane we are supposed to be in.
        blue = [x for x in (blue_l, blue_r) if x is not None]
        blue_x = min(blue, key=lambda x: abs(x - centre)) if blue else None

        if mode == AVOID:
            if self.blue_is_centre_line and blue_x is not None:
                # Two lanes: half a lane to the LEFT of the centre line. From
                # our own lane, where the line is on our left, that asks for a
                # full lane of crossing; from the oncoming lane, where the same
                # line is on our right, it asks for nothing. The manoeuvre ends
                # itself in the middle of the left lane.
                r.target_x = blue_x - self.half_width
                r.note = 'avoid: left of centre line'
            elif r.left_x is not None:
                # One carriageway with painted edges and no centre line. There
                # is no left LANE to cross into, so crossing the left edge
                # would drive off the road; the room to pass is inside the
                # corridor we are already in. Hug its left edge instead: the
                # car normally sits one half width from that line, so
                # `avoid_offset_frac` under 1 moves it over by the remainder,
                # and it settles there rather than drifting further.
                r.target_x = r.left_x + self.avoid_offset_frac * self.half_width
                r.note = 'avoid: hugging left edge'
            elif r.right_x is not None:
                # Same target, reached from the only line we can see: the left
                # edge is two half widths left of the right one.
                r.target_x = r.right_x - (2.0 - self.avoid_offset_frac) * self.half_width
                r.note = 'avoid: hugging left edge (from right line)'
            else:
                r.note = 'avoid: no lane'
                return r
        elif self.blue_is_centre_line and blue_r is not None:
            # Right-hand traffic: the blue centre line belongs on our LEFT, so
            # seeing it on the right means we are in the oncoming lane - after
            # an overtake, or after drifting.  Aim across it.  This is checked
            # BEFORE the both-sides case, which would otherwise centre us
            # neatly in the oncoming lane and leave us there.
            r.target_x = blue_r + self.half_width
            r.note = 'blue on right - crossing back'
        elif r.left_x is not None and r.right_x is not None:
            # Both sides visible: this is the only situation that measures the
            # lane, so it is the only one allowed to update the width.
            r.target_x = 0.5 * (r.left_x + r.right_x)
            measured = 0.5 * (r.right_x - r.left_x)
            # Two runs straddling the image centre are not necessarily the two
            # sides of a lane: dashes, a fork, or the near and far edge of one
            # thick line all produce a pair. Measured on this map, that drove
            # the estimate down to 60 px on an 820 px image - a "lane" a fifth
            # the width of the car's own view - and everything derived from it
            # went with it: the single-boundary target, and the whole sideways
            # step `avoid` is supposed to make (33 px instead of half a lane,
            # which reads as the car ignoring the obstacle entirely).
            if lo <= measured <= hi:
                self.half_width = 0.9 * self.half_width + 0.1 * measured
            r.note = f'{r.left_color}|{r.right_color}'
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
        # Clamp: the error is "how far off centre, in half-image widths", so
        # anything past +/-1 means the aim point is off the picture entirely.
        # Left unclamped it also feeds a step of several units into the D term
        # on the frame a rule switches, which the steering clamp then turns
        # into one frame at full lock in whichever direction.
        r.error = float(min(1.0, max(-1.0, (r.target_x - centre) / centre)))
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
    if r.band_far is not None:
        cv2.rectangle(out, (0, r.band_far[0]), (w - 1, r.band_far[1]),
                      (255, 220, 120), 1)
    cv2.line(out, (w // 2, y0), (w // 2, h - 1), (120, 120, 120), 1)
    by = (r.band[0] + r.band[1]) // 2
    for x, colour in ((r.left_x, (0, 255, 0)), (r.right_x, (0, 255, 0))):
        if x is not None:
            cv2.line(out, (int(x), r.band[0]), (int(x), r.band[1]), colour, 2)
    if r.target_x is not None:
        cv2.circle(out, (int(r.target_x), by), 6, (255, 0, 0), -1)
        cv2.line(out, (w // 2, h - 1), (int(r.target_x), by), (255, 0, 0), 2)
    txt = (r.note if r.error is None
           else f'{r.note}  e={r.error:+.3f}  hdg={r.heading:+.3f}')
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
    node.declare_parameter('max_run_frac', 0.25)
    node.declare_parameter('avoid_offset_frac', 0.45)
    node.declare_parameter('blue_is_centre_line', False)
    node.declare_parameter('half_width_tolerance', 2.0)
    node.declare_parameter('heading_band_gap', 0.20)
    # Row the road's vanishing point sits on, as a fraction of image height.
    # 0 means "the same row as roi_top", which is where it belongs: the ROI is
    # cut just below the horizon, so the two are the same measurement of how the
    # camera is aimed. Set it only if you move roi_top for some other reason.
    node.declare_parameter('horizon_frac', 0.0)
    g = node.get_parameter
    return LaneDetector(
        g('white_hsv_low').value, g('white_hsv_high').value,
        g('blue_hsv_low').value, g('blue_hsv_high').value,
        roi_top=g('roi_top').value, band_frac=g('band_frac').value,
        min_fill=g('min_fill').value, min_run_px=g('min_run_px').value,
        lane_half_width_frac=g('lane_half_width_frac').value,
        max_run_frac=g('max_run_frac').value,
        avoid_offset_frac=g('avoid_offset_frac').value,
        blue_is_centre_line=g('blue_is_centre_line').value,
        half_width_tolerance=g('half_width_tolerance').value,
        heading_band_gap=g('heading_band_gap').value,
        horizon_frac=(g('horizon_frac').value or None))


# --------------------------------------------------------------------------
# drive mode
# --------------------------------------------------------------------------

class LaneFollower(Node):

    def __init__(self):
        super().__init__('lane_follower')
        self.detector = declare_detector(self)

        # THESE NUMBERS ARE FALLBACKS, NOT THE VALUES THE CAR RUNS ON.
        #
        # declare_parameter says "this parameter exists, with this type, and
        # this is what it is worth if nobody supplies one". The launch file
        # supplies one: config/lane_avoid.yaml goes in as --params-file, and
        # anything typed on the command line after it. So the tuned values live
        # in the profile, and these only take effect under `profile:=none`.
        # They are deliberately the more cautious of the two.
        self.declare_parameter('image_topic', '/csi_front/image_raw')
        self.declare_parameter('cmd_topic', '/cmd_vel_twist')
        self.declare_parameter('speed', 1.00)
        self.declare_parameter('kp', 0.70)
        self.declare_parameter('kd', 0.10)
        # Matches twist_stamped_to_twist.py; Isaac Sim clamps around 0.5 rad.
        self.declare_parameter('max_steering_angle', 0.50)
        self.declare_parameter('lost_timeout', 0.5)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('avoid_topic', '/lane/avoid')
        self.declare_parameter('avoid_timeout', 1.0)
        self.declare_parameter('max_error_rate', 0.8)
        self.declare_parameter('k_head', 0.5)

        g = self.get_parameter
        self.speed = g('speed').value
        self.kp = g('kp').value
        self.kd = g('kd').value
        self.max_steer = g('max_steering_angle').value
        self.lost_timeout = g('lost_timeout').value
        self.publish_debug = g('publish_debug').value
        self.avoid_topic = g('avoid_topic').value
        self.avoid_timeout = g('avoid_timeout').value
        self.max_error_rate = g('max_error_rate').value
        self.k_head = g('k_head').value

        self.prev_error = None
        self.prev_time = None
        self.last_good = None      # last time a lane was seen, seconds
        self.stopped = True
        self.mode = FOLLOW
        self.mode_time = None      # last time obstacle_avoider.py spoke
        self.cmd_error = None      # rate-limited copy of the detector's error

        self.cmd_pub = self.create_publisher(Twist, g('cmd_topic').value, 10)
        self.dbg_pub = (self.create_publisher(Image, '/lane/debug_image', 1)
                        if self.publish_debug else None)
        self.sub = self.create_subscription(
            Image, g('image_topic').value, self.on_image, SENSOR_QOS)
        # Reliable, not best-effort: this is a state machine's output at 10 Hz,
        # and losing the message that ends an overtake would leave the car in
        # the oncoming lane until the staleness timeout notices.
        self.mode_sub = self.create_subscription(
            String, self.avoid_topic, self.on_mode, 10)
        # The image callback is the control loop, but a dead camera would
        # otherwise leave the last command latched in the drive graph forever.
        self.create_timer(0.1, self.watchdog)

        self.get_logger().info(
            f"lane follower: {g('image_topic').value} -> {g('cmd_topic').value} "
            f'(steering angle rad), speed={self.speed:.2f} m/s, '
            f'kp={self.kp:.2f} kd={self.kd:.2f}, '
            f'max_steer={self.max_steer:.2f} rad; '
            f'obstacle mode from {self.avoid_topic}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_mode(self, msg):
        mode = msg.data
        if mode not in (FOLLOW, AVOID, STOP):
            self.get_logger().warn(f'unknown mode {mode!r} on {self.avoid_topic}'
                                   ' - driving our own lane',
                                   throttle_duration_sec=5.0)
            mode = FOLLOW
        self.mode_time = self.now()
        if mode != self.mode:
            self.get_logger().info(f'mode {self.mode} -> {mode}')
            self.set_mode(mode)

    def set_mode(self, mode):
        self.mode = mode
        # Keep cmd_error: it is what makes the transition gradual. Only the
        # derivative history goes, so the slew itself is not read as a spike.
        # The aim point moves a whole lane across this transition. A derivative
        # taken over that step is a spike of several rad/s, which the clamp
        # turns into one frame at full lock in whichever direction - so drop
        # the history and let the next frame start the D term again.
        self.prev_error = None
        self.prev_time = None

    def active_mode(self):
        """What the avoider last said, or `follow` once that has gone stale.

        obstacle_avoider.py is an optional node: running the follower on its own
        is still the way to tune the lane colours, so silence has to mean "drive
        normally". It republishes at 10 Hz, so a gap longer than avoid_timeout
        means the node died or its sensors did - and the safe reading of that is
        our own lane, not a car parked in the oncoming one.
        """
        if self.mode_time is None:
            return FOLLOW
        if self.now() - self.mode_time > self.avoid_timeout:
            if self.mode != FOLLOW:
                self.get_logger().warn(
                    '%s silent for %.1f s - back to our own lane'
                    % (self.avoid_topic, self.avoid_timeout))
                self.set_mode(FOLLOW)
            return FOLLOW
        return self.mode

    def on_image(self, msg):
        try:
            rgb = decode_rgb(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return

        mode = self.active_mode()
        r = self.detector.detect(rgb, mode)
        t = self.now()

        if mode == STOP:
            # Too close to drive round. Brake, but keep detecting: the avoider
            # publishes `avoid` again the moment there is room, and the lane we
            # will resume into has to be in view when it does.
            if not self.stopped:
                self.cmd_pub.publish(Twist())
                self.stopped = True
                self.get_logger().warn('obstacle too close - stopping')
            self.prev_error = None
            self.prev_time = None
        elif r.error is None:
            # Say nothing on the command topic: the drive graph keeps driving on
            # the last Twist, and a lane line dropping out for a frame or two is
            # normal at 10 Hz - braking on every dropout makes the car crawl.
            # The watchdog stops us if it persists past lost_timeout.
            self.get_logger().warn('lane lost: ' + r.note,
                                   throttle_duration_sec=2.0)
        else:
            self.last_good = t
            dt = (t - self.prev_time) if self.prev_time else 0.0

            # Slew the aim point instead of letting it jump. Switching mode
            # moves the target a whole lane in one frame; steering on that step
            # puts the car at full lock immediately, which swings the camera
            # clean off the road - measured here as `avoid: no lane` two frames
            # into an overtake, with the watchdog then stopping the car mid
            # manoeuvre. Limiting how fast the target may travel sideways turns
            # the same manoeuvre into a lane change the camera can follow.
            if self.cmd_error is None or dt <= 1e-3:
                self.cmd_error = r.error
            else:
                step = self.max_error_rate * dt
                self.cmd_error += max(-step, min(step, r.error - self.cmd_error))
            error = self.cmd_error

            derr = ((error - self.prev_error) / dt
                    if dt > 1e-3 and self.prev_error is not None else 0.0)
            self.prev_error, self.prev_time = error, t

            # +angular.z is a left turn, so a lane centre to the RIGHT of the
            # image centre (error > 0) must produce a negative steering angle.
            #
            # r.heading is the same sign: the lane running away to the right
            # means the car is pointed left of it, and the correction is a right
            # turn. Without it the loop knows how far off centre it is but never
            # which way it is going, so it carries a turn into the lane it was
            # aiming for. Closed loop on a 0.66 m lane, k_head 0 vs 0.5:
            # overshoot 0.051 m -> 0.000, peak heading 43 deg -> 34, and the
            # lane it settles in 0.034 m off centre -> 0.014.
            steer = -(self.kp * error + self.k_head * r.heading + self.kd * derr)
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
                '# Lane colours for qcar2, written by\n'
                '#   ros2 launch qcar2 qcar2_lane_follow_launch.py tune:=true\n'
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
