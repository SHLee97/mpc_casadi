#!/usr/bin/env python3

import casadi as ca
import numpy as np
import math
from decomp_ros_msgs.msg import PolyhedronArray
import time

class NMPCController:
    def __init__(self, robot_pos, min_vx, max_vx, min_omega, max_omega,
                T=0.02, N=30, Q=np.diag([1.0, 1.0, 5.0]), R=np.diag([1.0, 1.0])):
        self.T = T          # time step
        self.N = N          # horizon length

        self.Q = Q          # Weight matrix for states
        self.R = R          # Weight matrix for controls

        # Constraints
        self.min_vx = min_vx
        self.max_vx = max_vx

        self.min_omega = min_omega
        self.max_omega = max_omega

        self.max_dvx = 0.8
        self.max_domega = math.pi/6

        # The history states and controls
        self.next_states = np.ones((self.N+1, 3))*robot_pos
        self.u0 = np.zeros((self.N, 2))

        self.setup_controller()
    
    def setup_controller(self):
        self.opti = ca.Opti()

        # state variable: position and velocity
        self.opt_states = self.opti.variable(self.N+1, 3)
        x = self.opt_states[:,0]
        y = self.opt_states[:,1]
        theta = self.opt_states[:,2]

        # the velocity
        self.opt_controls = self.opti.variable(self.N, 2)
        vx = self.opt_controls[0]
        omega = self.opt_controls[1]

        # create model
        self.opt_x_ref = self.opti.parameter(2, 3)
        f = lambda x_, u_: ca.vertcat(*[
            ca.cos(x_[2])*u_[0],  # dx
            ca.sin(x_[2])*u_[0],  # dy
            u_[1],                                      # dtheta
        ])
        # initial condition
        self.opti.subject_to(self.opt_states[0, :] == self.opt_x_ref[0, :])
        for i in range(self.N):
            x_next = self.opt_states[i, :] + f(self.opt_states[i, :], self.opt_controls[i, :]).T*self.T
            # k1 = f(self.opt_states[i, :], self.opt_controls[i, :]).T*self.T
            # k2 = f(self.opt_states[i, :]+k1*0.5, self.opt_controls[i, :]).T*self.T
            # k3 = f(self.opt_states[i, :]+k2*0.5, self.opt_controls[i, :]).T*self.T
            # k4 = f(self.opt_states[i, :]+k2, self.opt_controls[i, :]).T*self.T
            # x_next = self.opt_states[i, :] + (k1+k4)/6 + (k2+k3)/3
            self.opti.subject_to(self.opt_states[i+1, :] == x_next)
        
        # cost function
        obj = 0
        for i in range(self.N):
            state_error_ = self.opt_states[i, :] - self.opt_x_ref[1, :]
            control_error_ = self.opt_controls[i, :]
            obj = obj + ca.mtimes([state_error_, self.Q, state_error_.T]) \
                        + ca.mtimes([control_error_, self.R, control_error_.T])
        self.opti.minimize(obj)

        # constraint about change of velocity (acceleration)
        for i in range(self.N-1):
            dvel = (self.opt_controls[i+1,:] - self.opt_controls[i,:])/self.T
            self.opti.subject_to(self.opti.bounded(-self.max_dvx, dvel[0], self.max_dvx))
            self.opti.subject_to(self.opti.bounded(-self.max_domega, dvel[1], self.max_domega))

        # boundary and control conditions (velocity)
        self.opti.subject_to(self.opti.bounded(self.min_vx, vx, self.max_vx))
        self.opti.subject_to(self.opti.bounded(self.min_omega, omega, self.max_omega))

        opts_setting = {'ipopt.max_iter':2000,
                        'ipopt.print_level':0,
                        'print_time':0,
                        'ipopt.acceptable_tol':1e-6,
                        'ipopt.acceptable_obj_change_tol':1e-4}

        self.opti.solver('ipopt', opts_setting)
    
    def solve(self, next_trajectories, sfc):

        # next_traj (2,3) : start_pose(now) & target_pose
        self.opti.set_value(self.opt_x_ref, next_trajectories)
        
        ## provide the initial guess of the optimization targets
        self.opti.set_initial(self.opt_states, self.next_states)
        self.opti.set_initial(self.opt_controls, self.u0)

        for i in range(self.N-1):
                for j in range(len(sfc.polyhedrons[0].normals)):
                    if sfc.polyhedrons[0].normals[j].z == 0.0:
                        self.opti.subject_to(self.opt_states[i+1, 0]*sfc.polyhedrons[0].normals[j].x+self.opt_states[i+1, 1]*sfc.polyhedrons[0].normals[j].y<=sfc.polyhedrons[0].points[j].x*sfc.polyhedrons[0].normals[j].x+sfc.polyhedrons[0].points[j].y*sfc.polyhedrons[0].normals[j].y)
        ## solve the problem
        try:
            # print('solve success')
            sol = self.opti.solve()
            self.tmp_ = sol
        except:
            # print('solve failed!!!!!!!!!')
            sol = self.tmp_
            pass
        
        #solve가 안되면 이전 명령으로 움직이게 해봐야되나 solve가 잘 안되네 어렵구만
        ## obtain the control input
        self.u0 = sol.value(self.opt_controls)
        self.next_states = sol.value(self.opt_states)
        return self.u0[0,:]
