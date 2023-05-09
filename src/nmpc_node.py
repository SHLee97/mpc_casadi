#!/usr/bin/env python3
import numpy as np
import casadi as ca
import math
import time

import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import *

import sys
sys.path.append("/home/seunghyun/study_ws/src/mpc_casadi/src/")
try:
    from nmpc_controller import NMPCController
except:
    raise

# Odometry callback
odom = Odometry()
def odom_callback(data):
    global odom
    odom = data

# Reference path callback
path = Path()
def path_callback(data):
    global path
    path = data

def quaternion2Yaw(orientation):
    q0 = orientation.x
    q1 = orientation.y
    q2 = orientation.z
    q3 = orientation.w

    yaw = math.atan2(2.0*(q2*q3 + q0*q1), 1.0 - 2.0*(q1*q1 + q2*q2))
    return yaw

# Ref trajectory
def desired_trajectory(pos, N:int, path:Path, count:int):
    # initial state
    x_ = np.zeros((N+1, 3))

    # Process the path
    traj = []
    for i in range(len(path.poses)):
        x = path.poses[i].pose.position.x
        y = path.poses[i].pose.position.y
        q = quaternion2Yaw(path.poses[i].pose.orientation)
        traj.append([x,y,q])
    traj = np.array(traj)
    x_ref = traj[:,0]
    y_ref = traj[:,1]
    q_ref = traj[:,2]

    x_ref_ = x_ref[count:(count+N)]
    y_ref_ = y_ref[count:(count+N)]
    q_ref_ = q_ref[count:(count+N)]
    length = len(x_ref_)

    if length < N:
        x_ex = np.ones(N - length)*x_ref_[-1]
        x_ref_ = np.concatenate((x_ref_, x_ex), axis=None)

        y_ex = np.ones(N - length)*y_ref_[-1]
        y_ref_ = np.concatenate((y_ref_, y_ex), axis=None)

        q_ex = np.ones(N - length)*q_ref_[-1]
        q_ref_ = np.concatenate((q_ref_, q_ex), axis=None)


    x_ = np.array([x_ref_, y_ref_, q_ref_]).T
    x_ = np.concatenate((np.array([pos]), x_), axis=0)
    return x_


# Yaw saturation
def correct_state(states, tracjectories):
    error = tracjectories - states
    error[:,2] = error[:,2] - np.floor((error[:,2] + np.pi)/(2*np.pi))*2*np.pi
    tracjectories = states + error
    return tracjectories

def nmpc_node():
    rospy.init_node("nmpc_node", anonymous=True)
    rate = 10

    # Subscriber
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/path", Path, path_callback)
    
    # Publisher
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=rate)
    pub_pre_path = rospy.Publisher('/predict_path', Path, queue_size=rate)
    pub_path_now = rospy.Publisher('/path_now', Path, queue_size=1)
    pub_odom = rospy.Publisher('/odom_gt', Odometry, queue_size=1)
    
    r = rospy.Rate(rate)

    print("[INFO] Init Node...")
    # while(odom.header.frame_id == "" or path.header.frame_id == ""):
    while(path.header.frame_id == ""):
        r.sleep()
        continue
    print("[INFO] NMPC Node is ready!!!")

    print("[INFO] Wait 5s ...")    
    st = time.time()
    while time.time() - st < 5:
        r.sleep()
        continue
    print("[INFO] Start NMPC simulation!!!")

    T = 1/rate
    N = 20      # Predict horizon

    min_vx = rospy.get_param('/RobotConstraints/min_vx')
    max_vx = rospy.get_param('/RobotConstraints/max_vx')
    min_vy = rospy.get_param('/RobotConstraints/min_vy')
    max_vy = rospy.get_param('/RobotConstraints/max_vy')
    min_omega = rospy.get_param('/RobotConstraints/min_omega')
    max_omega = rospy.get_param('/RobotConstraints/max_omega')

    # Create the current robot position
    # px = odom.pose.pose.position.x
    # py = odom.pose.pose.position.y
    # pq = quaternion2Yaw(odom.pose.pose.orientation)\

    # Use GT: For simple test (rnlcksgdk)
    px = 10
    py = 1
    pq = 0
    pos = np.array([px, py, pq])

    nmpc = NMPCController(pos, min_vx, max_vx, min_vy, max_vy, min_omega, max_omega, T, N) 
    t0 = 0
    count = 0
    while not rospy.is_shutdown():
        if count >= len(path.poses):
            # Publish cmd_vel
            vel_msg = Twist()
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.angular.z = 0.0
            pub_vel.publish(vel_msg)
        else:
            # Current position
            # px = odom.pose.pose.position.x
            # py = odom.pose.pose.position.y
            # pq = quaternion2Yaw(odom.pose.pose.orientation)

            pos = np.array([px, py, pq])

            next_traj = desired_trajectory(pos, N, path, count)
            next_traj = correct_state(nmpc.next_states, next_traj)
            st = time.time()
            vel = nmpc.solve(next_traj)
            print("Processing time: {:.2f}s".format(time.time()-st))
            # Publish cmd_vel
            vel_msg = Twist()
            vel_msg.linear.x = vel[0]
            vel_msg.linear.y = vel[1]
            vel_msg.angular.z = vel[2]
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

            count += 1

            # Use GT: For simple test (rnlcksgdk)
            px = px + vel[0]*T*np.cos(pq) - vel[1]*T*np.sin(pq)
            py = py + vel[0]*T*np.sin(pq) + vel[1]*T*np.cos(pq)
            pq = pq + vel[2]*T

            # publish odom_gt
            odom_msg = Odometry()
            odom_msg.header.frame_id = "odom"
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.pose.pose.position.x = px
            odom_msg.pose.pose.position.y = py
            quat = quaternion_from_euler(0, 0, pq)
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]
            pub_odom.publish(odom_msg)


        r.sleep()
        t0 += T

if __name__ == '__main__':    
    try:
        nmpc_node()
    except rospy.ROSInterruptException:
        pass
