#!/usr/bin/env python3
"""Bridge Nav2's velocity command to the Isaac Sim QCar2 drive graph.

Two conversions happen here, not one:

1. MESSAGE TYPE.  Nav2 (with enable_stamped_cmd_vel: true) publishes
   geometry_msgs/TwistStamped on /cmd_vel_nav; the Isaac Sim OmniGraph
   subscribes to plain geometry_msgs/Twist on /cmd_vel_twist.

2. ANGULAR UNITS.  This is the important one.  Nav2 speaks *yaw rate*
   (angular.z in rad/s).  The Isaac Sim QCar2 drive controller reads
   angular.z as a *front-wheel steering angle* in radians.  Measured on
   this scene by commanding a constant Twist and reading back /odom:

       cmd angular.z   achieved radius   0.258/tan(cmd)
            0.30           0.797 m          0.834 m
            0.60           0.500 m          clamped at max steer
            1.00           0.482 m          clamped at max steer

   Feeding a yaw rate straight through therefore makes the car steer far
   harder than Nav2 asked for, so it overshoots the path, gets corrected,
   overshoots the other way, and ends up driving in circles.

   The bicycle model relates the two:

       omega = v * tan(delta) / L      =>      delta = atan(omega * L / v)

   with L the wheelbase.  Applying that here makes the achieved yaw rate
   match the requested one, so MPPI's "Ackermann" motion model and the
   simulated car finally agree about what a command means.

Parameters
----------
wheelbase            : float  distance front axle to rear axle [m]
max_steering_angle   : float  steering clamp [rad]
min_speed_for_steering : float  below this |v| the geometry is meaningless
input_topic          : str
output_topic         : str
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistStampedToAckermannTwist(Node):

    def __init__(self):
        super().__init__('twist_stamped_to_twist')

        # Measured from the Isaac Sim TF tree:
        #   base_link -> hub_frontLeft  = ( 0.130, 0.056)
        #   base_link -> wheel_rearLeft = (-0.128, 0.056)
        self.declare_parameter('wheelbase', 0.258)
        # Isaac Sim clamps around 0.5 rad; measured min radius 0.47-0.50 m.
        self.declare_parameter('max_steering_angle', 0.50)
        self.declare_parameter('min_speed_for_steering', 0.05)
        self.declare_parameter('input_topic', '/cmd_vel_nav')
        self.declare_parameter('output_topic', '/cmd_vel_twist')

        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.min_speed = self.get_parameter('min_speed_for_steering').value
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        self.sub = self.create_subscription(
            TwistStamped, in_topic, self.callback, 10)
        self.pub = self.create_publisher(Twist, out_topic, 10)

        self.get_logger().info(
            f'Ackermann bridge running: {in_topic} (TwistStamped, yaw rate) -> '
            f'{out_topic} (Twist, steering angle), '
            f'wheelbase={self.wheelbase:.3f} m, '
            f'max_steer={self.max_steer:.3f} rad '
            f'(min radius {self.wheelbase / math.tan(self.max_steer):.3f} m)')

    def yaw_rate_to_steering(self, v: float, omega: float) -> float:
        """Bicycle model: delta = atan(omega * L / v), clamped to the steering limit."""
        if abs(v) < self.min_speed:
            # Below walking pace the relation blows up, and a car cannot turn
            # on the spot anyway. Asking for it would slam the wheels to full
            # lock while barely moving.
            return 0.0
        delta = math.atan(omega * self.wheelbase / v)
        return max(-self.max_steer, min(self.max_steer, delta))

    def callback(self, msg: TwistStamped):
        v = msg.twist.linear.x
        omega = msg.twist.angular.z

        out = Twist()
        out.linear.x = v
        out.angular.z = self.yaw_rate_to_steering(v, omega)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TwistStampedToAckermannTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
