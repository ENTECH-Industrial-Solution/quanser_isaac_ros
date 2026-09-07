#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from qarm.action import MoveQArm

class QArmActionClient(Node):

    def __init__(self, name):
        super().__init__(name)
        self.move_qarm_client_ = ActionClient(self,MoveQArm,name)
        self.declare_parameter(
            'goal_pose',
            [0.0,0.0,0.5,0.0])
        # How long to wait for the goal to be acknowledged before resending it,
        # and how many times to try. A goal sent the instant the action server
        # is discovered can be accepted by the server before it has matched
        # this client's reply endpoint, and the response is then lost
        # ("failed to send response (timeout)") - resending clears it.
        self.declare_parameter('goal_response_timeout', 2.0)
        self.declare_parameter('goal_attempts', 5)
        self.goal_pose = self.get_parameter(
            'goal_pose').get_parameter_value().double_array_value
        self.goal_response_timeout = self.get_parameter(
            'goal_response_timeout').value
        self.goal_attempts = self.get_parameter('goal_attempts').value
        self.goal_handle_ = None

    def send_goal(self,goal_pose):
        # Wait for the server
        self.move_qarm_client_.wait_for_server()

        # Construct goal msg
        goal_msg = MoveQArm.Goal()
        goal_msg.task_space_pose=goal_pose

        for attempt in range(1, self.goal_attempts + 1):
            self.get_logger().info(f'Sending goal {list(goal_pose)}')
            send_goal_future = self.move_qarm_client_.send_goal_async(
                goal_msg,
                feedback_callback = self.feedback_cb)
            rclpy.spin_until_future_complete(
                self, send_goal_future,
                timeout_sec=self.goal_response_timeout)

            if not send_goal_future.done():
                send_goal_future.cancel()
                self.get_logger().warn(
                    f'No response to the goal within '
                    f'{self.goal_response_timeout:.1f} s '
                    f'(attempt {attempt}/{self.goal_attempts}), resending')
                continue

            self.goal_handle_ = send_goal_future.result()
            if not self.goal_handle_.accepted:
                self.get_logger().error('Goal was rejected by the server')
                return False
            return True

        self.get_logger().error(
            'Action server never acknowledged the goal - giving up')
        return False

    def wait_for_result(self):
        result_future = self.goal_handle_.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(result.message)
        return result

    def feedback_cb(self, feedback):
        self.get_logger().info (f'Distance to goal:{feedback.feedback.position_error_norm:.3f}, orientation difference:{feedback.feedback.orientation_error:.3f}')

    def cancel(self):
        if self.goal_handle_ is None:
            return
        cancel_future = self.goal_handle_.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
        cancel_response = cancel_future.result()
        if cancel_response is not None and len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')


def main(args=None):
    rclpy.init(args=args)
    qarm_action_client = QArmActionClient('move_qarm')
    try:
        goal_pose = list(qarm_action_client.goal_pose)
        qarm_action_client.get_logger().info(f'Goal pose parameter: {goal_pose}')
        if qarm_action_client.send_goal(goal_pose):
            qarm_action_client.wait_for_result()

    except (KeyboardInterrupt, ExternalShutdownException):
        qarm_action_client.cancel()

    finally:
        qarm_action_client.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
