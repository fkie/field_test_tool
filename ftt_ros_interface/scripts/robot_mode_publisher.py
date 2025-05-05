#!/usr/bin/env python3
"""robot_mode_publisher.py: Publishes the robot mode according to the received command messages from joystick and navigation stack."""

__author__ = "Carlos Tampier Cotoras"
__copyright__ = "Copyright 2021, Fraunhofer FKIE"
__license__ = "MIT"
__maintainer__ = "Carlos Tampier Cotoras"
__email__ = "carlos.tampier.cotoras@fkie.fraunhofer.de"

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool

class RobotModePublisher(Node):

    def __init__(self):
        print("Initiating robot mode publisher")
        super().__init__('robot_mode_publisher')

        self.joy_cmd_timeout = Duration(seconds=self.declare_parameter("~joy_cmd_timeout", 0.5).value)
        self.publish_period = self.declare_parameter("~publish_period", 1.0).value

        self.create_subscription(TwistStamped, "joy_cmd_vel", self.joy_cmd_callback, 10)
        self.create_subscription(TwistStamped, "nav_cmd_vel", self.nav_cmd_callback, 10)

        self.pub = self.create_publisher(Bool, 'robot_mode', 10)

        self.create_timer(self.publish_period, self.publish_mode_timer_callback)

        self.current_mode = -1
        self.joy_cmd_start_time = self.get_clock().now()

        pass

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        print("Closing robot mode publisher.")
        pass


    def joy_cmd_callback(self, cmd):
        if self.current_mode == 0:
            return
        if cmd.twist.linear.x != 0 or cmd.twist.linear.y != 0 or cmd.twist.linear.z != 0 or cmd.twist.angular.x != 0 or cmd.twist.angular.y != 0 or cmd.twist.angular.z != 0:
            self.joy_cmd_start_time = self.get_clock().now()
            self.current_mode = 0
        pass

    def nav_cmd_callback(self, cmd):
        last_joy_elapsed = self.get_clock().now() - self.joy_cmd_start_time
        if self.current_mode == 1 or last_joy_elapsed < self.joy_cmd_timeout:
            return
        self.current_mode = 1
        pass

    def publish_mode_timer_callback(self):
        if (self.current_mode >= 0):
            bool_msg = Bool()
            bool_msg.data = self.current_mode > 0
            self.pub.publish(bool_msg)
        pass


if __name__ == '__main__':
    rclpy.init()
    node = RobotModePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    pass
