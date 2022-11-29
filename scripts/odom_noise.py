#!/usr/bin/env python3
import rospy
from math import *
import numpy as np
from nav_msgs.msg import Odometry

# a callback for Gazebo odometry data
def callback(data):
    std = 0.05
    input_odom = data
    input_odom.pose.pose.position.x += np.random.normal(0,std)
    input_odom.pose.pose.position.y += np.random.normal(0,std)
    
    output_odom = Odometry()
    output_odom = data
    output_odom.pose.pose.position.x = input_odom.pose.pose.position.x
    output_odom.pose.pose.position.y = input_odom.pose.pose.position.y
    pub.publish(output_odom)
    

if __name__ == '__main__':
    rospy.init_node('noisy_odometry', anonymous=True)
    
    robot = rospy.get_param("~robot")
    odom_topic = robot + "/odom"
    out_topic = "/noisy_odom"
    pub = rospy.Publisher(out_topic, Odometry, queue_size=1)
    rospy.Subscriber(odom_topic, Odometry, callback)

    rospy.spin()