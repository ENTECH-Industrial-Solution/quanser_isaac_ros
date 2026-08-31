#!/usr/bin/env python3
"""YOLO object detection on the QCar2's 360 deg CSI camera ring.

One node subscribes to every enabled CSI camera, runs a single shared YOLO model
over their frames, and publishes per camera:

    /csi_<pos>/detections   vision_msgs/Detection2DArray
    /csi_<pos>/annotated    sensor_msgs/Image   (boxes drawn on, rgb8)

plus /csi/mosaic - all four annotated views tiled into ONE image - and
/csi/detections_count for a quick "is it seeing anything" check.

The mosaic exists because RViz has no grid layout: four Image displays dock as
four tabs, so you see one camera at a time unless you hand-craft a Qt
QMainWindow geometry blob into the .rviz file, which breaks the moment anyone
resizes a panel. Tiling server-side gives one topic that shows the whole ring at
once, and it records and replays as a single stream.

Started by qcar2_yolo_launch.py. The Nav2 stack is not touched: nothing here
publishes TF, a costmap layer, or anything on cmd_vel.

Two things about this file are load-bearing.

cv_bridge is deliberately NOT used
----------------------------------
ROS 2 Jazzy's cv_bridge is compiled against NumPy 1.x, while this machine puts
Isaac Sim's NumPy 2.5.2 first on PYTHONPATH (setup_python_env.sh, sourced from
~/.bashrc). Importing cv_bridge and converting one image segfaults the process -
exit 139, no Python traceback, which reads like a GPU or driver crash rather
than a dependency problem:

    python3 -c "from cv_bridge import CvBridge; import numpy as np; \
        CvBridge().cv2_to_imgmsg(np.zeros((4,4,3),np.uint8),'bgr8')"
    Segmentation fault (core dumped)

An `Image` of 8-bit colour is just height*width*3 bytes plus a step, so the
conversion is four lines of NumPy and needs no compiled extension at all. That
is what encode/decode below do. Do not "simplify" them back to cv_bridge.

One model, one callback lock
----------------------------
Four cameras share ONE YOLO model on one GPU that is also rendering the scene.
Running four callbacks concurrently through the same torch module does not make
it faster - it interleaves four inferences on the same CUDA stream and adds
contention on top. The lock below serialises them, and `drop_stale` throws away
frames that queued up behind an inference rather than working through a backlog
that is already out of date. Detections you get late are worse than detections
you skip.
"""

import os
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Int32
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

# Colours cycled per class id when drawing boxes (BGR-agnostic: we draw into an
# rgb8 buffer directly, so these are plain RGB triples).
PALETTE = [
    (255, 87, 51), (46, 204, 113), (52, 152, 219), (241, 196, 15),
    (155, 89, 182), (26, 188, 156), (231, 76, 60), (149, 165, 166),
]


def decode_rgb(msg):
    """sensor_msgs/Image -> HxWx3 uint8 RGB array, without cv_bridge.

    Isaac Sim's ROS2CameraHelper publishes `rgb8` (4-channel `rgba8` if the
    render product carries alpha). Both are handled; anything else is rejected
    loudly rather than silently misinterpreted as colour, because a wrong
    channel count produces a plausible-looking scrambled image that YOLO will
    happily return nothing for.
    """
    if msg.encoding in ("rgb8", "bgr8"):
        channels = 3
    elif msg.encoding in ("rgba8", "bgra8"):
        channels = 4
    else:
        raise ValueError(f"unsupported encoding {msg.encoding!r}")

    buf = np.frombuffer(msg.data, dtype=np.uint8)
    # `step` is the row stride in bytes and is not always width*channels - a
    # padded row would shear the image if we reshaped on width alone.
    img = buf.reshape(msg.height, msg.step // channels, channels)
    img = img[:, :msg.width, :3]
    if msg.encoding.startswith("bgr"):
        img = img[:, :, ::-1]
    return np.ascontiguousarray(img)


def encode_rgb(img, header):
    """HxWx3 uint8 RGB array -> sensor_msgs/Image, without cv_bridge."""
    msg = Image()
    msg.header = header
    msg.height, msg.width = img.shape[:2]
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(img, dtype=np.uint8).tobytes()
    return msg


def draw_box(img, x0, y0, x1, y1, colour, thickness=2):
    """Rectangle outline, clipped to the image. Plain NumPy slice assignment.

    cv2 would do this in one call, but the cv2 that wins on sys.path here is
    Isaac Sim's bundled build, and keeping this file free of both cv2 and
    cv_bridge means it runs under any of the three pythons on this machine.
    """
    h, w = img.shape[:2]
    x0 = max(0, min(w - 1, int(x0)))
    x1 = max(0, min(w - 1, int(x1)))
    y0 = max(0, min(h - 1, int(y0)))
    y1 = max(0, min(h - 1, int(y1)))
    if x1 <= x0 or y1 <= y0:
        return
    t = thickness
    img[y0:y0 + t, x0:x1] = colour          # top
    img[max(y0, y1 - t):y1, x0:x1] = colour  # bottom
    img[y0:y1, x0:x0 + t] = colour          # left
    img[y0:y1, max(x0, x1 - t):x1] = colour  # right


# Per-camera identity colour, drawn as a border on the mosaic tile. Four
# near-identical grey warehouse views are genuinely hard to tell apart otherwise.
CAMERA_COLOURS = {
    "front": (80, 220, 100),
    "right": (90, 160, 255),
    "back": (255, 170, 60),
    "left": (230, 110, 220),
}

# Clockwise from the front, filled row-major into a 2x2. Reading order then
# matches walking around the car.
MOSAIC_ORDER = ("front", "right", "back", "left")


def tile_mosaic(frames, order, downscale=2, border=3):
    """Tile per-camera frames into one 2x2 image.

    `frames` maps camera name -> HxWx3 uint8. Missing cameras become black
    tiles, so a subset of cameras still produces a stable-sized image instead of
    a mosaic that changes shape whenever one stream stalls - RViz reallocates its
    texture on every size change and flickers.
    """
    present = [c for c in order if c in frames]
    if not present:
        return None

    h, w = frames[present[0]].shape[:2]
    th, tw = h // downscale, w // downscale
    mosaic = np.zeros((th * 2, tw * 2, 3), dtype=np.uint8)

    for i, cam in enumerate(order):
        if cam not in frames:
            continue
        r, c = divmod(i, 2)
        # Nearest-neighbour decimation by slicing: no scipy, no cv2, and at
        # these sizes the quality difference does not matter for a monitor view.
        tile = frames[cam][::downscale, ::downscale][:th, :tw]
        if tile.shape[:2] != (th, tw):
            padded = np.zeros((th, tw, 3), dtype=np.uint8)
            padded[:tile.shape[0], :tile.shape[1]] = tile
            tile = padded
        tile = tile.copy()
        colour = CAMERA_COLOURS.get(cam, (200, 200, 200))
        tile[:border, :] = colour
        tile[-border:, :] = colour
        tile[:, :border] = colour
        tile[:, -border:] = colour
        mosaic[r * th:(r + 1) * th, c * tw:(c + 1) * tw] = tile

    return mosaic


def draw_label_bar(img, x0, y0, x1, colour, height=14):
    """Solid bar above a box, so the class colour is readable without a font."""
    h, w = img.shape[:2]
    x0 = max(0, min(w - 1, int(x0)))
    x1 = max(0, min(w, int(x1)))
    y1 = max(0, min(h, int(y0)))
    y0 = max(0, y1 - height)
    if x1 > x0 and y1 > y0:
        img[y0:y1, x0:x1] = colour


class YoloDetector(Node):

    def __init__(self):
        super().__init__("yolo_detector")

        self.declare_parameter("cameras", ["front", "back", "left", "right"])
        self.declare_parameter("model", "yolo11n.pt")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("confidence", 0.35)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("half", True)
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("max_detections", 30)
        self.declare_parameter("publish_mosaic", True)
        self.declare_parameter("mosaic_rate", 5.0)
        self.declare_parameter("mosaic_downscale", 2)

        cameras = list(self.get_parameter("cameras").value)
        self.conf = float(self.get_parameter("confidence").value)
        self.iou = float(self.get_parameter("iou").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.half = bool(self.get_parameter("half").value)
        self.max_det = int(self.get_parameter("max_detections").value)
        self.publish_annotated = bool(self.get_parameter("publish_annotated").value)
        self.publish_mosaic = bool(self.get_parameter("publish_mosaic").value)
        self.mosaic_downscale = max(1, int(self.get_parameter("mosaic_downscale").value))
        mosaic_rate = float(self.get_parameter("mosaic_rate").value)
        device = self.get_parameter("device").value
        model_name = self.get_parameter("model").value

        # Import here, not at module scope: ultralytics pulls in torch and takes
        # several seconds, and doing it inside __init__ means the failure is
        # reported by a live node with a real logger instead of an import
        # traceback before rclpy is even up.
        from ultralytics import YOLO
        import torch

        if device.startswith("cuda") and not torch.cuda.is_available():
            self.get_logger().warn(
                "cuda requested but torch.cuda.is_available() is False - "
                "falling back to cpu, expect ~1 Hz on four cameras")
            device = "cpu"
        # fp16 on CPU is slower than fp32 and unsupported for some ops.
        if device == "cpu":
            self.half = False
        # ultralytics 8.4 replaced the `half` bool with `quantize`, and passing
        # the old name logs a deprecation warning on EVERY predict() call - at
        # 40 inferences a second that buries the node's own logs completely.
        self.quantize = "fp16" if self.half else "fp32"

        # Ultralytics downloads a bare weight name into the PROCESS's working
        # directory, which for a launched node is wherever the user happened to
        # run `ros2 launch`. That silently scatters 5 MB .pt files around the
        # filesystem and re-downloads on every new directory. Resolve bare names
        # against one cache dir instead; an explicit path is left alone.
        if os.sep not in str(model_name):
            cache = os.path.join(
                os.environ.get("XDG_CACHE_HOME",
                               os.path.join(os.path.expanduser("~"), ".cache")),
                "qcar2_yolo")
            os.makedirs(cache, exist_ok=True)
            model_name = os.path.join(cache, str(model_name))

        self.get_logger().info(f"loading {model_name} on {device} ...")
        self.model = YOLO(model_name)
        self.model.to(device)
        self.device = device
        self.names = self.model.names

        # Warm up once, so the first real frame is not 3 s slower than the rest
        # (cudnn autotune + lazy kernel load). Without this the first callback
        # blows the drop_stale budget and the log opens with a burst of drops
        # that look like a performance problem.
        self.model.predict(
            np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8),
            device=device, quantize=self.quantize, imgsz=self.imgsz,
            verbose=False)
        self.get_logger().info("model ready")

        self._lock = threading.Lock()
        self._busy = False
        self._dropped = 0
        self._done = 0

        # Isaac Sim's camera publishers are RELIABLE; depth 1 keeps only the
        # newest frame, which is the right thing when inference is the
        # bottleneck - a deeper queue just ages the frames we eventually run on.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.det_pubs = {}
        self.ann_pubs = {}
        for cam in cameras:
            topic = f"/csi_{cam}/image_raw"
            self.det_pubs[cam] = self.create_publisher(
                Detection2DArray, f"/csi_{cam}/detections", 10)
            if self.publish_annotated:
                self.ann_pubs[cam] = self.create_publisher(
                    Image, f"/csi_{cam}/annotated", qos)
            self.create_subscription(
                Image, topic,
                lambda msg, c=cam: self.on_image(msg, c), qos)
            self.get_logger().info(f"subscribed {topic}")

        self.count_pub = self.create_publisher(Int32, "/csi/detections_count", 10)

        # Latest annotated frame per camera, tiled on a timer rather than on
        # every callback: at four cameras a per-callback mosaic would rebuild
        # the same image four times per round and publish three stale versions.
        self._frames = {}
        self._frames_lock = threading.Lock()
        if self.publish_mosaic:
            self.mosaic_pub = self.create_publisher(Image, "/csi/mosaic", qos)
            self.create_timer(1.0 / max(0.1, mosaic_rate), self.publish_mosaic_frame)

        self.create_timer(5.0, self.report)

    def publish_mosaic_frame(self):
        with self._frames_lock:
            frames = dict(self._frames)
        if not frames:
            return
        mosaic = tile_mosaic(frames, MOSAIC_ORDER, self.mosaic_downscale)
        if mosaic is None:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        # The tiles come from four different optical frames, so no single camera
        # frame is honest here. base_link is the one thing they share.
        header.frame_id = "base_link"
        self.mosaic_pub.publish(encode_rgb(mosaic, header))

    def on_image(self, msg, cam):
        # drop_stale: if an inference is already running, throw this frame away
        # instead of queueing behind it. See the module docstring.
        with self._lock:
            if self._busy:
                self._dropped += 1
                return
            self._busy = True
        try:
            self.process(msg, cam)
            self._done += 1
        except Exception as exc:  # a bad frame must not kill the node
            self.get_logger().error(f"{cam}: {type(exc).__name__}: {exc}")
        finally:
            with self._lock:
                self._busy = False

    def process(self, msg, cam):
        img = decode_rgb(msg)

        result = self.model.predict(
            img, device=self.device, quantize=self.quantize, imgsz=self.imgsz,
            conf=self.conf, iou=self.iou, max_det=self.max_det,
            verbose=False)[0]

        det_array = Detection2DArray()
        det_array.header = msg.header

        boxes = result.boxes
        annotated = img.copy() if self.publish_annotated else None

        for i in range(len(boxes)):
            x0, y0, x1, y1 = (float(v) for v in boxes.xyxy[i].tolist())
            score = float(boxes.conf[i])
            class_id = int(boxes.cls[i])

            det = Detection2D()
            det.header = msg.header
            # vision_msgs describes a box by its CENTRE plus size, not corners.
            # Emitting the top-left here instead is a classic quiet bug: RViz
            # still draws something, just offset by half a box.
            det.bbox = BoundingBox2D()
            det.bbox.center.position.x = (x0 + x1) / 2.0
            det.bbox.center.position.y = (y0 + y1) / 2.0
            det.bbox.center.theta = 0.0
            det.bbox.size_x = x1 - x0
            det.bbox.size_y = y1 - y0

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(self.names.get(class_id, class_id))
            hyp.hypothesis.score = score
            det.results.append(hyp)
            det_array.detections.append(det)

            if annotated is not None:
                colour = PALETTE[class_id % len(PALETTE)]
                draw_box(annotated, x0, y0, x1, y1, colour)
                draw_label_bar(annotated, x0, y0, x1, colour)

        self.det_pubs[cam].publish(det_array)
        if annotated is not None:
            self.ann_pubs[cam].publish(encode_rgb(annotated, msg.header))
            if self.publish_mosaic:
                with self._frames_lock:
                    self._frames[cam] = annotated

        self.count_pub.publish(Int32(data=len(det_array.detections)))

    def report(self):
        total = self._done + self._dropped
        if not total:
            self.get_logger().warn(
                "no frames yet - are the CSI render products enabled? "
                "(isaac_camera_streams.py, preset 'csi')")
            return
        self.get_logger().info(
            f"{self._done} frames in 5 s "
            f"({self._done / 5.0:.1f} Hz across all cameras), "
            f"{self._dropped} dropped ({100.0 * self._dropped / total:.0f}%)")
        self._done = 0
        self._dropped = 0


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
