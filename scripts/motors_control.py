#!/usr/bin/env python

import numpy as np
import rospy
import logging
import time
import os
import sys
from geometry_msgs.msg import Twist, Vector3


class GRMI_motors():
    def __init__(self):
        # ROS TIMER
        self.mytime = 0.35

        # MODEL
        self.l = 0.15
        self.r = 0.035
        self.jac_inv = np.matrix([[1/self.r, self.l/(self.r), 0.0],
        [1/self.r, 0.0 ,self.l/(self.r)]]);
        self.speed = 0.0
        self.omega = 0.0
        self.vel_motors = np.zeros(shape=(2,1))

        # ROS INFRAESTRUCRE
        self.cmdvel_sub = rospy.Subscriber("/cmd_vel", Twist, self.callback)
        self.motor_pub = rospy.Publisher("/vel_motors",  Vector3, queue_size=1)

    def pub_motors(self, event):
        pub_motors_msg = Vector3()
        pub_motors_msg.x = self.vel_motors[0,0]
        pub_motors_msg.y = self.vel_motors[1,0]
        self.motor_pub.publish(pub_motors_msg)


    def callback(self, msg_vel):
        self.speed = msg_vel.linear.x
        self.omega = msg_vel.angular.z
        if self.omega>0:
            vel = np.matrix([[self.speed],[self.omega],[0]])
        else:
            vel = np.matrix([[self.speed],[0],[self.omega]])
        self.vel_motors = np.matmul(self.jac_inv, vel)

if __name__ == '__main__':
    rospy.init_node('motors_control')
    try:
        node = GRMI_motors()
        rospy.Timer(rospy.Duration(node.mytime), node.pub_motors)
        rate = rospy.Rate(10)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
