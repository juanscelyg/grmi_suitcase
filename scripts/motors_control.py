#!/usr/bin/env python

import numpy as np
import rospy
import logging
import time
import os
import sys
from geometry_msgs.msg import Pose2D
import RPi.GPIO as GPIO

class GRMI_motors():
    def __init__(self):
        # GPIO Config
        self.pin_motor_r = 32 #BCM12
        self.pin_motor_l = 33 #BCM13
        self.pwm_frequency = 500
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin_motor_r, GPIO.OUT)
        GPIO.setup(self.pin_motor_l, GPIO.OUT)
        self.pwm_motor_r = GPIO.PWM(self.pin_motor_r, self.pwm_frequency)
        self.pwm_motor_l = GPIO.PWM(self.pin_motor_l, self.pwm_frequency)
        self.pwm_motor_r.start(self.vel2cycle(0,'r'))
        self.pwm_motor_l.start(self.vel2cycle(0,'l'))
        rospy.loginfo(" Init Motors")
        time.sleep(1)

        # MODEL
        self.power_gain = 0.1
        self.speed_gain = 0.1
        self.omega_gain = 0.1
        self.speed = 0.0
        self.omega = 0.0

        # ROS TIMER
        self.mytime = 0.75

        # ROS INFRAESTRUCRE
        self.error_sub = rospy.Publisher("/error_pose",Pose2D, self.callback)

    def vel2cycle(self,vel,mode):
        slope = 50.0
        if mode=='l':
            slope = -50.0
        offset = 50.0
        if abs(vel)>1:
            vel=abs(vel)/vel
        return int(self.power_gain*(slope*vel+offset))


    def callback(self, msg_error):
        vel_r = self.speed_gain*msg_error.x + self.omega_gain*msg_error.theta
        vel_l = self.speed_gain*msg_error.x - self.omega_gain*msg_error.theta
        rospy.loginfo("vel_l: {} vel_r:{}".format(vel_l, vel_r))
        self.pwm_motor_r.ChangeDutyCycle(self.vel2cycle(vel_r,'r'))
        self.pwm_motor_l.ChangeDutyCycle(self.vel2cycle(vel_l,'l'))


if __name__ == '__main__':
    rospy.init_node('motors_control')
    try:
        node = GRMI_motors()
        rospy.Timer(rospy.Duration(node.mytime), node.detect)
        rate = rospy.Rate(10)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    GPIO.cleanup()
    print('exiting')
