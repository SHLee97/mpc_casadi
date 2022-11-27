#!/usr/bin/env python3
import numpy as np
import casadi as ca
import math
import time

import rospy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import Marker
from tf.transformations import *
from decomp_ros_msgs.msg import PolyhedronArray

import sys
# sys.path.append("/home/lsh/ee688_ws/src/mpc_casadi/src/")
sys.path.append("/home/dklee98/git/term_ws/src/mpc_casadi/src/")
try:
    from nmpc_controller import NMPCController
except:
    raise

# Odometry callback
agent_odom = Odometry()
def odom_agent_callback(data):
    global agent_odom
    agent_odom = data

# Target callback
target_odom = Odometry()
def odom_target_callback(data):
    global target_odom
    target_odom = data

sfc = PolyhedronArray()
def sfc_callback(data):
    global sfc
    sfc = data
    

def quaternion2Yaw(orientation):
    q0 = orientation.x
    q1 = orientation.y
    q2 = orientation.z
    q3 = orientation.w

    yaw = math.atan2(2.0*(q2*q3 + q0*q1), 1.0 - 2.0*(q1*q1 + q2*q2))
    return yaw

def correct_state(tracjectories):
    error = tracjectories[0,:] - tracjectories[1,:]
    error[2] = error[2] - np.floor((error[2] + np.pi)/(2*np.pi))*2*np.pi
    tracjectories[0,:] = tracjectories[1,:] + error
    return tracjectories

def nmpc_node():
    rospy.init_node("nmpc_node", anonymous=True)
    rate = 10

    # Subscriber
    rospy.Subscriber("/agent/odom", Odometry, odom_agent_callback)
    # rospy.Subscriber("/noisy_odom", Odometry, odom_target_callback)
    rospy.Subscriber("/target/odom", Odometry, odom_target_callback)
    rospy.Subscriber("/polyhedron_array", PolyhedronArray, sfc_callback)

    # Publisher
    pub_vel = rospy.Publisher('/agent/cmd_vel', Twist, queue_size=rate)
    pub_pre_path = rospy.Publisher('/predict_path', Path, queue_size=rate)
    target_pose_pub = rospy.Publisher("/target_pose", Marker, queue_size=1)
    
    r = rospy.Rate(rate)

    print("[INFO] Init Node...")
    while(agent_odom.header.frame_id == "" or target_odom.header.frame_id == ""):
        r.sleep()
        continue
    print("[INFO] NMPC Node is ready!!!")

    print("[INFO] Wait 10s ...")    
    st = time.time()
    while time.time() - st < 10:
        r.sleep()
        continue
    print("[INFO] Start NMPC simulation!!!")

    T = 1/rate
    N = 10      # Predict horizon

    min_vx = rospy.get_param('/agentConstraints/min_vx')
    max_vx = rospy.get_param('/agentConstraints/max_vx')
    min_omega = rospy.get_param('/agentConstraints/min_omega')
    max_omega = rospy.get_param('/agentConstraints/max_omega')

    # Create the current agent position
    px = agent_odom.pose.pose.position.x
    py = agent_odom.pose.pose.position.y
    pq = quaternion2Yaw(agent_odom.pose.pose.orientation)
    agent_pos = np.array([px, py, pq])

    nmpc = NMPCController(agent_pos, min_vx, max_vx, min_omega, max_omega, T, N) 
    t0 = 0
    count = 0
    while not rospy.is_shutdown():
        # Current position
        px = agent_odom.pose.pose.position.x
        py = agent_odom.pose.pose.position.y
        pq = quaternion2Yaw(agent_odom.pose.pose.orientation)
        pos = np.array([px, py, pq])

        # Create the current target position
        px = target_odom.pose.pose.position.x
        py = target_odom.pose.pose.position.y
        pq = quaternion2Yaw(target_odom.pose.pose.orientation)
        target_pos = np.array([px, py, pq])

        next_traj = np.array([pos, target_pos])
        next_traj = correct_state(next_traj)
        # next_traj = desired_trajectory(pos, N, path, count)
        # next_traj = correct_state(nmpc.next_states, next_traj) #yaw check
        st = time.time()
        # vel = nmpc.solve(next_traj)
        vel = nmpc.solve(next_traj, sfc)
        
        print("Processing time: {:.2f}s".format(time.time()-st))
        print(vel)
        # Publish cmd_vel
        vel_msg = Twist()
        vel_msg.linear.x = vel[0]
        vel_msg.angular.z = vel[1]
        pub_vel.publish(vel_msg)

        # Publish the predict path
        predict = nmpc.next_states
        predict_msg = Path()
        predict_msg.header.frame_id = "odom"
        predict_msg.header.stamp = rospy.Time.now()
        for pos in predict:
            pose = PoseStamped()
            pose.header = predict_msg.header
            pose.pose.position.x = pos[0]
            pose.pose.position.y = pos[1]
            quat = quaternion_from_euler(0, 0, pos[2])
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            predict_msg.poses.append(pose)
        pub_pre_path.publish(predict_msg)

        # Publish the target pose
        target_points = Marker()
        target_points.header.frame_id = "odom"
        target_points.ns = "points"
        target_points.id = 1
        target_points.type = Marker.POINTS
        target_points.action = Marker.ADD
        target_points.color = ColorRGBA(1, 1, 0, 1)
        target_points.scale.x = 0.2
        target_points.scale.y = 0.2
        target_points.scale.z = 0.0
        target_points.points.append(Point(next_traj[1,0], next_traj[1,1], 0))
        target_pose_pub.publish(target_points)
        count += 1

        r.sleep()
        t0 += T

if __name__ == '__main__':    
    try:
        nmpc_node()
    except rospy.ROSInterruptException:
        pass
