#!/usr/bin/env python3
"""RealSense RGB-D publisher for the QArm.

Publishes sensor_msgs/Image directly rather than through image_transport_py:
that package only exists from ROS 2 Kilted onwards and this workspace is Jazzy.
Raw images on the base topic are what image_transport's own "raw" transport
publishes, so image_transport subscribers still work.

cv_bridge is deliberately not used - it segfaults on this machine (Jazzy's
cv_bridge is built against NumPy 1.x while Isaac Sim puts NumPy 2.x first on
PYTHONPATH), so the conversion is done with plain NumPy, as in yolo_detector.py.
"""

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image

from pal.products.qarm import QArmRealSense


def to_image_msg(array, encoding, stamp, frame_id):
    """sensor_msgs/Image from a HxW or HxWxC numpy array, without cv_bridge."""
    array = np.ascontiguousarray(array)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = array.shape[0]
    msg.width = array.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = int(array.strides[0])
    msg.data = array.tobytes()
    return msg


class QArmCamera(Node):
    def __init__(self):
        super().__init__('qarm_camera')

        # Declare camera parameters
        self.declare_parameter('color_width', 640)
        self.declare_parameter('color_height', 480)
        self.declare_parameter('depth_width', 640)
        self.declare_parameter('depth_height', 480)
        self.declare_parameter('fps', 30.0)

        # Set up camera
        self.color_width = self.get_parameter('color_width').get_parameter_value().integer_value
        self.color_height = self.get_parameter('color_height').get_parameter_value().integer_value
        self.depth_width = self.get_parameter('depth_width').get_parameter_value().integer_value
        self.depth_height = self.get_parameter('depth_height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        self.camera = QArmRealSense(
            hardware = 1,
            mode='RGB&DEPTH',
            frameWidthRGB=self.color_width,
            frameHeightRGB=self.color_height,
            frameRateRGB=self.fps,
            frameWidthDepth=self.depth_width,
            frameHeightDepth=self.depth_height,
            frameRateDepth=self.fps,
            )

        qos = QoSProfile(depth=10)

        # Publishers
        self.color_pub = self.create_publisher(Image, 'qarm_camera/color', qos)
        self.depth_pub = self.create_publisher(Image, 'qarm_camera/depth', qos)

        # Timer
        period = 1.0 / self.fps
        self.timer = self.create_timer(
            period,
            self.camera_publish_cb,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info("RGBD camera initialized")

    def camera_publish_cb(self):
        new = self.camera.read_RGB()
        if new == -1:
            self.get_logger().warn(
                "Failed to capture color frame", throttle_duration_sec=2.0)
            return
        new = self.camera.read_depth(dataMode='M')
        if new == -1:
            self.get_logger().warn(
                "Failed to capture depth frame", throttle_duration_sec=2.0)
            return
        stamp = self.get_clock().now().to_msg()

        colour = np.asarray(self.camera.imageBufferRGB, dtype=np.uint8)
        self.color_pub.publish(
            to_image_msg(colour, 'bgr8', stamp, 'camera_color'))

        depth = np.asarray(self.camera.imageBufferDepthM, dtype=np.float32)
        self.depth_pub.publish(
            to_image_msg(depth, '32FC1', stamp, 'camera_depth'))

    def destroy_node(self):
        self.camera.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = QArmCamera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
