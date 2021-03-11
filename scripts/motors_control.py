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
        # MODEL
        self.power_gain = rospy.get_param('/motors_control/power_gain')
        self.speed_gain = rospy.get_param('/motors_control/speed_gain')
        self.omega_gain = rospy.get_param('/motors_control/omega_gain')
        self.speed = 0.0
        self.omega = 0.0

        # GPIO Config
        self.pin_motor_r = 12 #Board32
        self.pin_motor_l = 13 #Board33
        self.pwm_frequency = 50
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_motor_r, GPIO.OUT)
        GPIO.setup(self.pin_motor_l, GPIO.OUT)
        self.pwm_motor_r = GPIO.PWM(self.pin_motor_r, self.pwm_frequency)
        self.pwm_motor_l = GPIO.PWM(self.pin_motor_l, self.pwm_frequency)
        self.pwm_motor_r.start(self.vel2cycle(0,'r'))
        self.pwm_motor_l.start(self.vel2cycle(0,'l'))
        rospy.loginfo(" Init Motors")
        time.sleep(1)

        # ROS INFRAESTRUCRE
        self.error_sub = rospy.Subscriber("/error_pose", Pose2D, self.callback)

    def vel2cycle(self,vel,mode):
        slope = 2.5
        if mode=='l':
            slope = -2.5
        offset = 7.5
        if abs(vel)>1:
            vel=abs(vel)/vel
        return int(slope*self.power_gain*vel+offset)


    def callback(self, msg_error):
        vel_r = self.speed_gain*msg_error.x + self.omega_gain*msg_error.theta
        vel_l = self.speed_gain*msg_error.x - self.omega_gain*msg_error.theta
        rospy.loginfo("vel_l: {:.4f} vel_r:{:.4f}".format(vel_l, vel_r))
        self.pwm_motor_r.ChangeDutyCycle(self.vel2cycle(vel_r,'r'))
        self.pwm_motor_l.ChangeDutyCycle(self.vel2cycle(vel_l,'l'))


if __name__ == '__main__':
    rospy.init_node('motors_control')
    try:
        node = GRMI_motors()
        rate = rospy.Rate(10)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    GPIO.cleanup()
    print('exiting')
