#!/usr/bin/env python

import numpy as np
import rospy
import logging
import time
import os
import sys
from geometry_msgs.msg import TwistStamped, Vector3


class GRMI_motors():
    def __init__(self):
        # ROS TIMER
        self.mytime = 0.35

        # MODEL
        self.l = 0.15
        self.r = 0.035
        self.jac_inv = np.matrix([[1/self.r, self.l/(2*self.r)],
        [1/self.r, -self.l/(2*self.r)]]);
        self.speed = 0.0
        self.omega = 0.0
        self.vel_motors = np.zeros(shape=(2,1))

        # ROS INFRAESTRUCRE
        self.cmdvel_sub = rospy.Subscriber("/cmd_vel", TwistStamped, self.callback)
        self.motor_pub = rospy.Publisher("/vel_motors",  Vector3, queue=1)

    def vel2cycle(self,vel,mode):
        slope = 1.0
        if mode=='l':
            vel = -vel
        offset = 0.0
        if abs(vel)>1:
            vel=abs(vel)/vel
        return (slope*vel+offset)

    def pub_motors(self):
        pub_motors_msg = Vector3()
        pub_motors_msg.point.x = self.vel_motors[0,1]
        pub_motors_msg.point.y = self.vel_motors[1,1]
        self.motor_pub.publish(pub_motors_msg)


    def callback(self, msg_vel):
        self.speed = msg_vel.Twist.linear.x
        self.omega = msg_vel.Twist.angular.z
        vel = np.matrix([self.speed],[self.omega])
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
    GPIO.cleanup()
    print('exiting')
