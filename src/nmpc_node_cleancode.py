#!/usr/bin/env python3
import numpy as np
import casadi as ca
import math
import time

import rospy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from decomp_ros_msgs.msg import PolyhedronArray

import sys
import signal
sys.path.append("/home/lsh/ee688_ws/src/mpc_casadi/src/")
# sys.path.append("/home/dklee98/git/term_ws/src/mpc_casadi/src/")
try:
    from nmpc_controller import NMPCController
except:
    raise

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

## mpc mode
MODE_NMPC = 0
MODE_TUBE = 1


class nmpc_node():
    def __init__(self):
        rospy.init_node("nmpc_node", anonymous=True)

        robot_name = rospy.get_param("~robot")
        self.mode = rospy.get_param("mode", MODE_NMPC)
        self.min_vx = rospy.get_param('/agentConstraints/min_vx')
        self.max_vx = rospy.get_param('/agentConstraints/max_vx')
        self.min_omega = rospy.get_param('/agentConstraints/min_omega')
        self.max_omega = rospy.get_param('/agentConstraints/max_omega')

        # Subscriber
        rospy.Subscriber("/" + robot_name + "/odom", Odometry, self.odom_agent_cb)
        # rospy.Subscriber("/noisy_odom", Odometry, self.odom_target_cb)
        # rospy.Subscriber("/target/odom", Odometry, self.odom_target_cb)
        rospy.Subscriber("/target_detect_odom", Odometry, self.odom_target_cb)
        rospy.Subscriber("/polyhedron_array", PolyhedronArray, self.sfc_cb)

        # Publisher
        self.pub_vel_cmd = rospy.Publisher("/" + robot_name + "/cmd_vel", Twist, queue_size=1)
        self.pub_predict_path = rospy.Publisher("/predict_path", Path, queue_size=1)
        self.pub_circle_path = rospy.Publisher("/target_circle", MarkerArray, queue_size=1)
        # self.pub_target_pose = rospy.Publisher("/target_pose", Marker, queue_size=1)

        self.robot_odom = Odometry()
        self.target_odom = Odometry()
        self.sfc = PolyhedronArray()

        self.dt = 1/10
        self.rate = rospy.Rate(1/self.dt)
        self.N = 10                     # Predict horizon

        ## maintain circle range
        self.d_candidate = []  
        self.radius = 0.5

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
            # print(time.time() - t0)
            continue

        robot_pose_init = self.get_pose(self.robot_odom)

        ## Select MPC mode
        if self.mode == MODE_NMPC:
            self.nmpc = NMPCController(robot_pose_init, self.min_vx, self.max_vx,
                                self.min_omega, self.max_omega, self.dt, self.N)
            print("MPC set controller")
        elif self.mode == MODE_TUBE:
            print("TBC set controller")

        print("[INFO] Start NMPC simulation!!!")


    def start_control(self):
        pose_cur = self.get_pose(self.robot_odom)
        pose_tar = self.get_pose(self.target_odom)

        next_traj = np.array([pose_cur, pose_tar])
        next_traj, error = self.correct_state(next_traj)
        tar_idx = self.target_compute(next_traj, error, self.radius)

        next_traj = np.array([pose_cur, self.d_candidate[tar_idx]])
        next_traj, _ = self.correct_state(next_traj)

        t0 = time.time()
        if self.mode == MODE_NMPC:
            self.nmpc = NMPCController(pose_cur, self.min_vx, self.max_vx,
                                self.min_omega, self.max_omega, self.dt, self.N)
            self.out_vel = self.nmpc.solve(next_traj, self.sfc)
            out_predict_path = self.nmpc.next_states
        elif self.mode == MODE_TUBE:
            print("TBC solve")

        if self.verbose:
            print("Processing time: {:.2f}s".format(time.time()-t0))

        self.visualize(self.out_vel, out_predict_path, tar_idx)
        
        self.d_candidate = []

    def get_pose(self, input_odom):
        q = [input_odom.pose.pose.orientation.x,
            input_odom.pose.pose.orientation.y,
            input_odom.pose.pose.orientation.z,
            input_odom.pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(q)

        return np.array([input_odom.pose.pose.position.x,
                        input_odom.pose.pose.position.y, yaw])
    
    def get_distance(self, p1, p2):
        return np.sqrt(np.power(p2[0] - p1[0], 2) + np.power(p2[1] - p1[1], 2))


    def target_compute(self, traj, error, radius):
        theta = np.arctan2(error[1], error[0])  # atan(dy/dx)
        nx, ny, nyaw = traj[0]  # now agent
        cx, cy, cyaw = traj[1]  # center of target
        M = 10
        for t in range(M):
            self.d_candidate.append([cx + np.cos(2*np.pi*t/M), 
                                    cy + np.sin(2*np.pi*t/M), 
                                    2*np.pi*t/M + np.pi])
        min_d = 1000
        for i, c in enumerate(self.d_candidate):
            d = self.get_distance([nx, ny], [c[0], c[1]])
            if d < min_d:
                min_d = d
                min_idx = i
        yaw_diff = nyaw - theta
        # print(min_d, min_idx)
        return min_idx

    def correct_state(self, traj):
        error = traj[0,:] - traj[1,:]
        error[2] = error[2] - np.floor((error[2] + np.pi) / (2 * np.pi)) * 2 * np.pi
        traj[0,:] = traj[1,:] + error
        return traj, error

    def visualize(self, out_vel, out_predict_path, tar_idx):
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

        # Pub circle
        del_markers = MarkerArray()
        candiates = MarkerArray()
        for i, p in enumerate(self.d_candidate):
            cand = Marker()
            cand.header.frame_id = "odom"
            cand.header.stamp = rospy.Time.now()
            cand.type = Marker.ARROW
            cand.action = Marker.ADD
            cand.id = i
            cand.scale.x = 0.5
            cand.scale.y = 0.1
            cand.scale.z = 0.1
            cand.pose.position.x = p[0]
            cand.pose.position.y = p[1]
            q = quaternion_from_euler(0, 0, p[2])
            cand.pose.orientation.x = q[0]
            cand.pose.orientation.y = q[1]
            cand.pose.orientation.z = q[2]
            cand.pose.orientation.w = q[3]
            if i == tar_idx:
                cand.color = ColorRGBA(1, 1, 1, 1)
            else:
                cand.color = ColorRGBA(1, 1, 0, 1)
            candiates.markers.append(cand)

            dm = Marker()
            dm.header.frame_id = "odom"
            dm.header.stamp = rospy.Time.now()
            dm.action = Marker.DELETE
            dm.id = i
            del_markers.markers.append(dm)
        self.pub_circle_path.publish(del_markers)
        self.pub_circle_path.publish(candiates)

if __name__ == '__main__':   
    nmpc_ = nmpc_node() 
    while 1:
        try:
            nmpc_.start_control()
            nmpc_.rate.sleep()
        except rospy.ROSInterruptException:
            sys.exit(0)
