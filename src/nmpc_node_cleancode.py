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
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import sys
# sys.path.append("/home/lsh/study_ws/src/mpc_casadi/src/")
sys.path.append("/home/dklee98/git/term_ws/src/mpc_casadi/src/")
try:
    from nmpc_controller import NMPCController
except:
    raise



class nmpc_node():
    def __init__(self):
        rospy.init_node("nmpc_node", anonymous=True)

        robot_name = rospy.get_param("~robot")
        self.min_vx = rospy.get_param('/RobotConstraints/min_vx')
        self.max_vx = rospy.get_param('/RobotConstraints/max_vx')
        self.min_omega = rospy.get_param('/RobotConstraints/min_omega')
        self.max_omega = rospy.get_param('/RobotConstraints/max_omega')

        # Subscriber
        rospy.Subscriber("/" + robot_name + "/odom", Odometry, self.odom_robot_cb)
        rospy.Subscriber("/noisy_odom", Odometry, self.odom_target_cb)

        # Publisher
        self.pub_vel_cmd = rospy.Publisher("/" + robot_name + "/cmd_vel", Twist, queue_size=1)
        self.pub_predict_path = rospy.Publisher("/predict_path", Path, queue_size=1)
        self.pub_target_pose = rospy.Publisher("/target_pose", Marker, queue_size=1)

        self.robot_odom = Odometry()
        self.target_odom = Odometry()

        self.rate = rospy.Rate(10)
        self.dt = 1/self.rate
        self.N = 10                     # Predict horizon


    def odom_robot_cb(self, msg):
        self.robot_odom = msg
    
    def odom_target_cb(self, msg):
        self.target_odom = msg

    def start_control(self):
        print("[INFO] Init Node...")
        while(robot_odom.header.frame_id == "" or target_odom.header.frame_id == ""):
            return
        print("[INFO] NMPC Node is ready!!!")

        print("[INFO] Wait 10s ...") 
        t0 = time.time()
        while time.time() - t0 < 10:
            return
        print("[INFO] Start NMPC simulation!!!")

        robot_pose_init = get_pose(self.robot_odom)
        nmpc = NMPCController(robot_pose_init, self.min_vx, self.max_vx,
                            self.min_omega, self.max_omega, self.dt, self.N)

        while not rospy.is_shutdown():
            pose_cur = get_pose(self.robot_odom)
            pose_tar = get_pose(self.target_odom)

            next_traj = np.array([pose_cur, pose_tar])
            next_traj = correct_state(next_traj)

            t0 = time.time()
            out_vel = nmpc.solve(next_traj)
            out_predict_path = nmpc.next_states

            if self.verbose:
                print("Processing time: {:.2f}s".format(time.time()-t0))

            # Pub cmd vel
            vel_ = Twist()
            vel_.linear.x = out_vel[0]
            vel_.angular.z = out_vel[1]
            self.pub_vel_cmd.publish(vel_)

            # Pub predict path
            predict_ = Path()
            predict_.header.frame_id = "odom"
            predict_.header.stamp = rospy.Time.now()
            for p in predict:
                pose_ = PoseStamped()
                pose_.header = predict_.header
                pose_.pose.position.x = p[0]
                pose_.pose.position.y = p[1]
                q = quaternion_from_euler(0, 0, p[2])
                pose_.pose.orientation.x = q[0]
                pose_.pose.orientation.y = q[1]
                pose_.pose.orientation.z = q[2]
                pose_.pose.orientation.w = q[3]
                predict_.poses.append(pose)
            self.pub_predict_path.publish(predict_)


    def get_pose(self, input_odom)
        q = [input_odom.pose.pose.orientation.x,
            input_odom.pose.pose.orientation.y,
            input_odom.pose.pose.orientation.z,
            input_odom.pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(q)

        return np.array([input_odom.pose.pose.position.x,
                        input_odom.pose.pose.position.y, yaw])

    def correct_state(traj):
        error = traj[0,:] - traj[1,:]
        error[2] = error[2] - np.floor((error[2] + np.pi) / (2 * np.pi)) * 2 * np.pi
        traj[0,:] = traj[1,:] + error
        return traj

if __name__ == '__main__':   
    nmpc_ = nmpc_node() 
    try:
        nmpc_.start_control()
        nmpc.rate.sleep()
    except rospy.ROSInterruptException:
        pass
