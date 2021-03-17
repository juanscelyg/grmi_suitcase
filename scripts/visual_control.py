#!/usr/bin/env python

import numpy as np
import rospy
import logging
import time
import os
import sys
from geometry_msgs.msg import PointStamped, Twist

class GRMI_visual():
    def __init__(self):

        # MODEL
        self.power_gain = rospy.get_param('/visual_control/power_gain')
        self.speed_gain = rospy.get_param('/visual_control/speed_gain')
        self.omega_gain = rospy.get_param('/visual_control/omega_gain')

        # ROS INFRAESTRUCRE
        self.error_sub = rospy.Subscriber("/error_pose", PointStamped, self.callback)
        self.vel_pub = rospy.Publisher("/cmd_vel",  Twist, queue_size=1)

    def callback(self, msg_error):
        error_speed = msg_error.point.x
        error_omega = msg_error.point.z
        vel = self.speed_gain*error_speed
        omg = self.omega_gain*error_omega
        msg_vel = Twist()
        msg_vel.linear.x = self.power_gain*vel
        msg_vel.angular.z = self.power_gain*omg
        self.vel_pub.publish(msg_vel)


if __name__ == '__main__':
    rospy.init_node('visual_control')
    try:
        node = GRMI_visual()
        rate = rospy.Rate(10)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
