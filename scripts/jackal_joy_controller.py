#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 02:02:03 2019
@author: mason
"""

''' import libraries '''
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import Thrust

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def rpy_saturation(angle):
    if angle>np.pi:
        angle=angle-2*np.pi
    if angle<-np.pi:
        angle=angle+2*np.pi
    return angle

''' class '''
class robot():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.vel_pub = rospy.Publisher("/turtlebot3/cmd_vel", Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/turtlebot3/odom', Odometry, self.odom_callback)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

        self.rate = rospy.Rate(30)
        self.hold = 0 #to stop sending input shortly

        self.robot_check = False
        self.joy_check = False

    def odom_callback(self, msg):
        self.robot_check = True

    def joy_callback(self, msg):
        self.joy = msg
        self.joy_check = True
    
def input(rbt):
    k_vel_input = Twist()
    k_vel_input.linear.x= 1.0*(rbt.joy.axes[4])
    k_vel_input.angular.z = 0.7854*(rbt.joy.axes[0])
    rbt.vel_pub.publish(k_vel_input)

##############################################################################################

mav_ctr = robot()
time.sleep(1) #wait 1 second to assure that all data comes in

''' main '''
if __name__ == '__main__':
    while 1:
        try:
            if mav_ctr.joy_check==1:
                input(mav_ctr)
                mav_ctr.rate.sleep()
            else: 
                mav_ctr.rate.sleep()
                pass
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
