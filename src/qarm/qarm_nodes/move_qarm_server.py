#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor,ExternalShutdownException

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from qarm.action import MoveQArm

import numpy as np
from hal.products.qarm import QArmUtilities

class QArmActionServer(Node):

    def __init__(self, name):
        super().__init__(name)

        # Initialize QArm Util
        self.myArmUtil = QArmUtilities()

        # Tunables: how close counts as reached, and how long a goal may run
        # before it is abandoned (a goal the arm cannot converge on would
        # otherwise block the server forever).
        self.declare_parameter('position_tolerance', 0.04)
        self.declare_parameter('goal_timeout', 20.0)
        self.declare_parameter('joint_state_timeout', 5.0)
        self.threshhold = self.get_parameter('position_tolerance').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.joint_state_timeout = self.get_parameter('joint_state_timeout').value

        # Create publisher for QArm commands
        self.joint_pub_ = self.create_publisher(
            Float64MultiArray,
            '/qarm/joint_position_cmd',
            10
        )

        self.led_pub_ = self.create_publisher(
            Float64MultiArray,
            '/qarm/led_cmd',
            10
        )

        # Create subscriber for QArm states
        self.joint_state_sub_ = self.create_subscription(
            JointState,
            '/qarm/joint_states',
            self.joint_sub_cb,
            10
        )

        # Create messages for feedback and result
        self.feedback_ = MoveQArm.Feedback()
        self.result_ = MoveQArm.Result()

        # Initialize buffers and other parameters
        self.joint_command = np.zeros(4, dtype=np.float64)
        self.LED_command = np.zeros(3, dtype=np.float64)
        self.latest_joint_positions = np.zeros(5, dtype=np.float64)
        self.joint_state_received = False

        self.action_name_ = name
        self.action_server_ = ActionServer(
            self,
            MoveQArm,
            self.action_name_,
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb,
            callback_group = ReentrantCallbackGroup()
        )
        self.rate = self.create_rate(30)
        self.get_logger().info("Move Arm Action server started")

    def joint_sub_cb(self,joint_state:JointState):
        self.latest_joint_positions = np.array(joint_state.position)
        self.joint_state_received = True

    def set_led(self, colour):
        led_cmd_msg = Float64MultiArray()
        led_cmd_msg.data = [float(c) for c in colour]
        self.led_pub_.publish(led_cmd_msg)

    def finish(self, goal_handle: ServerGoalHandle, success, message):
        self.result_.success = success
        self.result_.message = message
        if success:
            self.get_logger().info(f'{self.action_name_}: Succeeded')
            goal_handle.succeed()
        else:
            self.get_logger().info(f'{self.action_name_}: {message}')
            goal_handle.abort()
        return self.result_

    def reachable(self, phiOptimal, pose_cmd):
        """The Quanser IK returns [0,0,0,0] as its out-of-limits sentinel, which
        is also a legitimate solution for the home pose. Confirm with forward
        kinematics instead of trusting the sentinel alone."""
        if not np.allclose(phiOptimal, 0.0):
            return True
        p, _ = self.myArmUtil.forward_kinematics(np.zeros(4, dtype=np.float64))
        return np.linalg.norm(p - np.asarray(pose_cmd[:3])) <= self.threshhold

    def execute_cb(self, goal_handle: ServerGoalHandle):

        goal = goal_handle.request
        pose_cmd = np.asarray(goal.task_space_pose, dtype=np.float64)

        # A goal sent the moment the stack comes up can arrive before the
        # first joint state, so wait briefly rather than failing outright.
        waited = self.get_clock().now()
        while not self.joint_state_received:
            if (self.get_clock().now() - waited).nanoseconds / 1e9 > \
                    self.joint_state_timeout:
                self.set_led([1, 0, 0])
                return self.finish(
                    goal_handle, False,
                    'No joint states received - is the QArm node running?')
            self.rate.sleep()

        self.get_logger().info("Moving QArm to goal")

        phi, phiOptimal = self.myArmUtil.inverse_kinematics(
            pose_cmd[:3], pose_cmd[3], self.latest_joint_positions[0:4])

        if not self.reachable(phiOptimal, pose_cmd):
            self.get_logger().warn(
                f"Goal pose {pose_cmd} is outside of joint limits")
            self.set_led([1, 0, 0])
            return self.finish(
                goal_handle, False,
                'Goal pose is outside of the QArm joint limits.')

        joint_cmd_msg = Float64MultiArray()
        joint_cmd_msg.data = [float(p) for p in phiOptimal]
        self.joint_pub_.publish(joint_cmd_msg)
        self.set_led([0, 1, 0])

        start = self.get_clock().now()
        while True:
            if goal_handle.is_cancel_requested:
                self.hold_position()
                self.result_.success = False
                self.result_.message = 'Goal canceled by client.'
                goal_handle.canceled()
                self.get_logger().info(f'{self.action_name_}: Canceled')
                return self.result_

            current_p, current_r = self.myArmUtil.forward_kinematics(
                self.latest_joint_positions[0:4])
            position_error_norm = float(
                np.linalg.norm(current_p - pose_cmd[:3]))
            orientation_error = float(
                np.abs(pose_cmd[3] - self.latest_joint_positions[3]))

            # Publish feedback
            self.feedback_.position_error_norm = position_error_norm
            self.feedback_.orientation_error = orientation_error
            goal_handle.publish_feedback(self.feedback_)

            if position_error_norm + orientation_error <= self.threshhold:
                return self.finish(
                    goal_handle, True, 'QArm has reached the target pose.')

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > self.goal_timeout:
                self.hold_position()
                self.set_led([1, 0, 0])
                return self.finish(
                    goal_handle, False,
                    f'QArm failed to reach the target pose within '
                    f'{self.goal_timeout:.1f} s '
                    f'(remaining error {position_error_norm:.3f} m).')

            self.rate.sleep()

    def hold_position(self):
        joint_cmd_msg = Float64MultiArray()
        joint_cmd_msg.data = [float(p) for p in self.latest_joint_positions[:4]]
        self.joint_pub_.publish(joint_cmd_msg)

    def cancel_cb(self,goal_handle:ServerGoalHandle):

        self.get_logger().info('Received cancel request')
        self.get_logger().info('Moving arm to previous time step')
        self.hold_position()
        self.set_led([1, 1, 0])

        return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)
    qarm_action_server = QArmActionServer('move_qarm')
    executor = MultiThreadedExecutor()
    executor.add_node(qarm_action_server)
    try:
        executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        qarm_action_server.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
