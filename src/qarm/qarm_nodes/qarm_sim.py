#!/usr/bin/env python3
"""Simulated QArm hardware.

Drop-in replacement for the qarm_hardware node: identical topics, identical
message types, no Quanser SDK and no physical arm. Joints are driven towards
the commanded position at a bounded speed, which is enough to exercise the
whole MoveQArm pipeline (action server, kinematics, feedback, cancel).
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

from qarm.msg import QArmDiagnostics

JOINT_NAMES = [
    'base_joint',
    'shoulder_joint',
    'arm_joint',
    'wrist_joint',
    'gripper_joint',
]


class QArmSim(Node):
    def __init__(self):
        super().__init__('qarm_sim')

        # rad/s per joint; the real arm is slower, this only sets how long a
        # goal takes to converge.
        self.declare_parameter('max_joint_speed', 1.0)
        self.declare_parameter('update_rate', 200.0)
        self.max_joint_speed = self.get_parameter('max_joint_speed').value
        self.update_rate = self.get_parameter('update_rate').value

        # Commands
        self.joint_command = np.zeros(4, dtype=np.float64)
        self.gripper_command = np.zeros(1, dtype=np.float64)
        self.LED_command = np.zeros(3, dtype=np.float64)

        # Simulated state: 4 arm joints + gripper
        self.position = np.zeros(5, dtype=np.float64)
        self.velocity = np.zeros(5, dtype=np.float64)

        qos_profile = QoSProfile(depth=10)

        self.joint_state_pub_ = self.create_publisher(
            JointState, '/qarm/joint_states', qos_profile)
        self.diag_pub_ = self.create_publisher(
            QArmDiagnostics, '/qarm/diagnostics', qos_profile)

        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray, '/qarm/joint_position_cmd',
            self.joint_cmd_cb, qos_profile)
        self.gripper_cmd_sub = self.create_subscription(
            Float64, '/qarm/gripper_cmd', self.gripper_cmd_cb, qos_profile)
        self.led_cmd_sub = self.create_subscription(
            Float64MultiArray, '/qarm/led_cmd', self.led_cmd_cb, qos_profile)

        self.timer = self.create_timer(
            1.0 / self.update_rate, self.step_and_publish)

        self.get_logger().info(
            f'QArm simulator started (max_joint_speed='
            f'{self.max_joint_speed} rad/s) - no hardware in use')

    def joint_cmd_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 4:
            self.joint_command[:] = msg.data[:4]

    def gripper_cmd_cb(self, msg: Float64):
        self.gripper_command[0] = msg.data

    def led_cmd_cb(self, msg: Float64MultiArray):
        if len(msg.data) == 3:
            self.LED_command[:] = msg.data

    def step_and_publish(self):
        dt = 1.0 / self.update_rate
        target = np.concatenate((self.joint_command, self.gripper_command))

        # Move towards the target, capped at max_joint_speed.
        delta = target - self.position
        step = np.clip(delta, -self.max_joint_speed * dt,
                       self.max_joint_speed * dt)
        self.position += step
        self.velocity = step / dt

        stamp = self.get_clock().now().to_msg()

        msg = JointState()
        msg.header.stamp = stamp
        msg.name = JOINT_NAMES
        msg.position = list(self.position)
        msg.velocity = list(self.velocity)
        msg.effort = [0.0] * len(JOINT_NAMES)
        self.joint_state_pub_.publish(msg)

        diag_msg = QArmDiagnostics()
        diag_msg.header.stamp = stamp
        diag_msg.joint_names = JOINT_NAMES
        diag_msg.joint_currents = [0.0] * len(JOINT_NAMES)
        diag_msg.joint_pwms = [0.0] * len(JOINT_NAMES)
        diag_msg.joint_temperatures = [25.0] * len(JOINT_NAMES)
        self.diag_pub_.publish(diag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = QArmSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
