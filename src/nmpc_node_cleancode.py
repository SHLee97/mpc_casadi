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
from decomp_ros_msgs.msg import PolyhedronArray

import sys
import signal
# sys.path.append("/home/lsh/study_ws/src/mpc_casadi/src/")
sys.path.append("/home/dklee98/git/term_ws/src/mpc_casadi/src/")
try:
    from nmpc_controller import NMPCController
except:
    raise

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class nmpc_node():
    def __init__(self):
        rospy.init_node("nmpc_node", anonymous=True)

        robot_name = rospy.get_param("~robot")
        self.min_vx = rospy.get_param('/agentConstraints/min_vx')
        self.max_vx = rospy.get_param('/agentConstraints/max_vx')
        self.min_omega = rospy.get_param('/agentConstraints/min_omega')
        self.max_omega = rospy.get_param('/agentConstraints/max_omega')

        # Subscriber
        rospy.Subscriber("/" + robot_name + "/odom", Odometry, self.odom_agent_cb)
        rospy.Subscriber("/noisy_odom", Odometry, self.odom_target_cb)
        rospy.Subscriber("/polyhedron_array", PolyhedronArray, self.sfc_cb)

        # Publisher
        self.pub_vel_cmd = rospy.Publisher("/" + robot_name + "/cmd_vel", Twist, queue_size=1)
        self.pub_predict_path = rospy.Publisher("/predict_path", Path, queue_size=1)
        self.pub_target_pose = rospy.Publisher("/target_pose", Marker, queue_size=1)

        self.robot_odom = Odometry()
        self.target_odom = Odometry()
        self.sfc = PolyhedronArray()

        self.dt = 1/10
        self.rate = rospy.Rate(1/self.dt)
        self.N = 10                     # Predict horizon

        self.verbose = True

        self.initailize()


    def odom_agent_cb(self, msg):
        self.robot_odom = msg
    
    def odom_target_cb(self, msg):
        self.target_odom = msg

    def sfc_cb(self, msg):
        self.sfc = msg

    def initailize(self):
        print("[INFO] Init Node...")
        while(self.robot_odom.header.frame_id == "" or self.target_odom.header.frame_id == ""):
            continue
        print("[INFO] NMPC Node is ready!!!")

        print("[INFO] Wait 10s ...") 
        t0 = time.time()
        while time.time() - t0 < 1:
            print(time.time() - t0)
            continue

        robot_pose_init = self.get_pose(self.robot_odom)
        self.nmpc = NMPCController(robot_pose_init, self.min_vx, self.max_vx,
                            self.min_omega, self.max_omega, self.dt, self.N)
        print("[INFO] Start NMPC simulation!!!")


    def start_control(self):
        pose_cur = self.get_pose(self.robot_odom)
        pose_tar = self.get_pose(self.target_odom)

        next_traj = np.array([pose_cur, pose_tar])
        next_traj = self.correct_state(next_traj)

        t0 = time.time()
        out_vel = self.nmpc.solve(next_traj, self.sfc)
        out_predict_path = self.nmpc.next_states

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
        for p in out_predict_path:
            pose_ = PoseStamped()
            pose_.header = predict_.header
            pose_.pose.position.x = p[0]
            pose_.pose.position.y = p[1]
            q = quaternion_from_euler(0, 0, p[2])
            pose_.pose.orientation.x = q[0]
            pose_.pose.orientation.y = q[1]
            pose_.pose.orientation.z = q[2]
            pose_.pose.orientation.w = q[3]
            predict_.poses.append(pose_)
        self.pub_predict_path.publish(predict_)


    def get_pose(self, input_odom):
        q = [input_odom.pose.pose.orientation.x,
            input_odom.pose.pose.orientation.y,
            input_odom.pose.pose.orientation.z,
            input_odom.pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(q)

        return np.array([input_odom.pose.pose.position.x,
                        input_odom.pose.pose.position.y, yaw])

    def correct_state(self, traj):
        error = traj[0,:] - traj[1,:]
        error[2] = error[2] - np.floor((error[2] + np.pi) / (2 * np.pi)) * 2 * np.pi
        traj[0,:] = traj[1,:] + error
        return traj

if __name__ == '__main__':   
    nmpc_ = nmpc_node() 
    while 1:
        try:
            nmpc_.start_control()
            nmpc_.rate.sleep()
        except rospy.ROSInterruptException:
            sys.exit(0)
