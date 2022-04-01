#!/usr/bin/env python3
"""robotModePublisher.py: Publishes the robot mode according to the received command messages from joystick and navigation stack."""

__author__ = "Carlos Tampier Cotoras"
__copyright__ = "Copyright 2021, Fraunhofer FKIE"
__license__ = "MIT"
__maintainer__ = "Carlos Tampier Cotoras"
__email__ = "carlos.tampier.cotoras@fkie.fraunhofer.de"

import rospy
import geometry_msgs.msg
import industrial_msgs.msg

class RobotModePublisher:

    def __init__(self):
        print("Initiating robot mode publisher")
        rospy.init_node('rosbotModePublisher', anonymous=True)

        self.joy_cmd_timeout = rospy.get_param("~joy_cmd_timeout", 0.5)
        self.publish_period = rospy.get_param("~publish_period", 1)

        rospy.Subscriber("joy_cmd_vel", geometry_msgs.msg.TwistStamped, self.joy_cmd_callback)
        rospy.Subscriber("nav_cmd_vel", geometry_msgs.msg.TwistStamped, self.nav_cmd_callback)

        self.pub = rospy.Publisher('robot_mode', industrial_msgs.msg.RobotMode, queue_size=10)

        rospy.Timer(rospy.Duration(self.publish_period), self.publish_mode_timer_callback)

        self.current_mode = industrial_msgs.msg.RobotMode.UNKNOWN
        self.joy_cmd_start_time = rospy.get_time()

        pass

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        print("Closing robot mode publisher.")
        pass


    def joy_cmd_callback(self, cmd):
        if self.current_mode == industrial_msgs.msg.RobotMode.MANUAL:
            return
        if cmd.twist.linear.x != 0 or cmd.twist.linear.y != 0 or cmd.twist.linear.z != 0 or cmd.twist.angular.x != 0 or cmd.twist.angular.y != 0 or cmd.twist.angular.z != 0:
            self.joy_cmd_start_time = rospy.get_time()
            self.current_mode = industrial_msgs.msg.RobotMode.MANUAL
        pass

    def nav_cmd_callback(self, cmd):
        last_joy_elapsed = rospy.get_time() - self.joy_cmd_start_time
        if self.current_mode == industrial_msgs.msg.RobotMode.AUTO or last_joy_elapsed < self.joy_cmd_timeout:
            return
        self.current_mode = industrial_msgs.msg.RobotMode.AUTO
        pass

    def publish_mode_timer_callback(self, event):
        self.pub.publish(self.current_mode)
        pass


if __name__ == '__main__':
    with RobotModePublisher():
        rospy.spin()
    pass
