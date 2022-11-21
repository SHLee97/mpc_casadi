#!/usr/bin/env python3
import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import math



def scan_cb(msg):
    pc2_msg = lp.projectLaser(msg)
    pc2_msg.header.frame_id = "odom"
    pc_pub.publish(pc2_msg)

if __name__ == '__main__':
    rospy.init_node("laserscan_to_pointcloud", anonymous=True)
    lp = lg.LaserProjection()

    robot = rospy.get_param("~robot")
    scan_topic = robot + "/scan"
    out_topic = robot + "/points"
    pc_pub = rospy.Publisher(out_topic, PointCloud2, queue_size=1)
    rospy.Subscriber(scan_topic, LaserScan, scan_cb, queue_size=1)
    
    rospy.spin()