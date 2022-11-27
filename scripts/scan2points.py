#!/usr/bin/env python3
import sensor_msgs.point_cloud2 as pc2
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, LaserScan
from tf.transformations import quaternion_matrix
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from tf2_ros import TransformStamped

import laser_geometry.laser_geometry as lg
import math

agent_odom = Odometry()
def odom_agent_callback(data):
    global agent_odom
    agent_odom = data

def scan_cb(msg):
    pc2_msg = lp.projectLaser(msg)
    pc2_msg.header.frame_id = "odom"
    rot = quaternion_matrix([agent_odom.pose.pose.orientation.x, agent_odom.pose.pose.orientation.y, 
    agent_odom.pose.pose.orientation.z, agent_odom.pose.pose.orientation.w])    
    rot[0,3]=agent_odom.pose.pose.position.x
    rot[1,3]=agent_odom.pose.pose.position.y
    transform = TransformStamped()
    transform.transform.translation = agent_odom.pose.pose.position
    transform.transform.rotation = agent_odom.pose.pose.orientation
    pc2_msg = do_transform_cloud(pc2_msg, transform)
    pc2_msg.header.frame_id = "odom"
    points_list = []
    for point in pc2.read_points(pc2_msg, skip_nans=True):
        points_list.append([point[0], point[1], point[2], point[3]])
    # print(len(points_list))
    pc_pub.publish(pc2_msg)

if __name__ == '__main__':
    rospy.init_node("laserscan_to_pointcloud", anonymous=True)
    lp = lg.LaserProjection()

    robot = rospy.get_param("~robot")
    scan_topic = robot + "/scan"
    odom_topic = robot + "/odom"
    out_topic = "/points"
    pc_pub = rospy.Publisher(out_topic, PointCloud2, queue_size=1)
    rospy.Subscriber(scan_topic, LaserScan, scan_cb, queue_size=1)
    rospy.Subscriber(odom_topic, Odometry, odom_agent_callback)
    
    rospy.spin()