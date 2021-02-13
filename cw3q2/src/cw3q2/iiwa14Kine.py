#!/usr/bin/env python

import rospy
from math import pi
import numpy as np
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion


class iiwa14_kinematic(object):

    def __init__(self):
        ##TODO: Fill in the DH parameters based on the xacro file (cw3/iiwa_description/urdf/iiwa14.xacro)
        self.DH_params = np.array([[0.0, -pi/2, 0.2025, 0.0],
                                   [0.0, pi/2, 0.0, 0.0],
                                   [0.0, pi/2, 0.42, 0.0],
                                   [0.0, -pi/2, 0.0, 0.0],
                                   [0.0, -pi/2, 0.4, 0.0],
                                   [0.0, pi/2, 0.0, 0.0],
                                   [0.0, 0.0, 0.126, 0.0]])

        self.current_joint_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.joint_limit_min = [-170 * pi / 180, -120 * pi / 180, -170 * pi / 180, -120 * pi / 180, -170 * pi / 180,
                                -120 * pi / 180, -175 * pi / 180]
        self.joint_limit_max = [170 * pi / 180, 120 * pi / 180, 170 * pi / 180, 120 * pi / 180, 170 * pi / 180,
                                120 * pi / 180, 175 * pi / 180]

        ##The mass of each link.
        self.mass = [4, 4, 3, 2.7, 1.7, 1.8, 0.3]

        ##Moment on inertia of each link, defined at the centre of mass.
        ##Each row is (Ixx, Iyy, Izz) and Ixy = Ixz = Iyz = 0.
        self.Ixyz = np.array([[0.1, 0.09, 0.02],
                              [0.05, 0.018, 0.044],
                              [0.08, 0.075, 0.01],
                              [0.03, 0.01, 0.029],
                              [0.02, 0.018, 0.005],
                              [0.005, 0.0036, 0.0047],
                              [0.001, 0.001, 0.001]])

        ##gravity
        self.g = 9.8

        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback,
                                                queue_size=5)

        self.pose_broadcaster = tf2_ros.TransformBroadcaster()

    def joint_state_callback(self, msg):
        for i in range(0, 7):
            self.current_joint_position[i] = msg.position[i]

        current_pose = self.forward_kine(self.current_joint_position, 7)
        self.broadcast_pose(current_pose)

    def dh_matrix_standard(self, a, alpha, d, theta):
        A = np.zeros((4, 4))

        A[0, 0] = np.cos(theta)
        A[0, 1] = -np.sin(theta) * np.cos(alpha)
        A[0, 2] = np.sin(theta) * np.sin(alpha)
        A[0, 3] = a * np.cos(theta)

        A[1, 0] = np.sin(theta)
        A[1, 1] = np.cos(theta) * np.cos(alpha)
        A[1, 2] = -np.cos(theta) * np.sin(alpha)
        A[1, 3] = a * np.sin(theta)

        A[2, 1] = np.sin(alpha)
        A[2, 2] = np.cos(alpha)
        A[2, 3] = d

        A[3, 3] = 1.0

        return A

    def broadcast_pose(self, pose):

        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'iiwa_link_0'
        transform.child_frame_id = 'iiwa_ee'

        transform.transform.translation.x = pose[0, 3]
        transform.transform.translation.y = pose[1, 3]
        transform.transform.translation.z = pose[2, 3]
        transform.transform.rotation = self.rotmat2q(pose)

        self.pose_broadcaster.sendTransform(transform)

    ##Useful Transformation function
    def T_translation(self, t):
        T = np.identity(4)
        for i in range(0, 3):
            T[i, 3] = t[i]
        return T

    ##Useful Transformation function
    def T_rotationZ(self, theta):
        T = np.identity(4)
        T[0, 0] = np.cos(theta)
        T[0, 1] = -np.sin(theta)
        T[1, 0] = np.sin(theta)
        T[1, 1] = np.cos(theta)
        return T

    ##Useful Transformation function
    def T_rotationX(self, theta):
        T = np.identity(4)
        T[1, 1] = np.cos(theta)
        T[1, 2] = -np.sin(theta)
        T[2, 1] = np.sin(theta)
        T[2, 2] = np.cos(theta)
        return T

    ##Useful Transformation function
    def T_rotationY(self, theta):
        T = np.identity(4)
        T[0, 0] = np.cos(theta)
        T[0, 2] = np.sin(theta)
        T[2, 0] = -np.sin(theta)
        T[2, 2] = np.cos(theta)
        return T


    def forward_kine(self, joint, frame):
        T = np.identity(4)
        ##Add offset from the iiwa platform.
        T[2, 3] = 0.1575
        ##TODO: Fill in this function to complete Q2.
	for i in range(frame):
            A = self.dh_matrix_standard(self.DH_params[i][0], self.DH_params[i][1], self.DH_params[i][2],
                                            joint[i] + self.DH_params[i][3])

            T = T.dot(A)

        return T


    def forward_kine_cm(self, joint, frame):
        ##TODO: Fill in this function to complete Q2.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## "frame" is an integer indicating the frame you wish to calculate.
        ## The output is a numpy 4*4 matrix describing the transformation from the 'iiwa_link_0' frame to the centre of mass of the specified link.
	T = self.forward_kine(joint, frame - 1)
        Rot = self.T_rotationZ(joint[frame - 1] + self.DH_params[frame - 1][3])

	Tcom = np.dot(np.dot(T,R),Trans)
        return Tcom  

    def get_jacobian(self, joint):
        ##TODO: Fill in this function to complete Q2.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## The output is a numpy 6*7 matrix describing the Jacobian matrix defining at each frame.
	J = np.zeros((6,7))
	A = np.zeros((7,4,4))
	for i in range(len(joint)):
	    A[i] = self.dh_matrix_standard(self.DH_params[i][0], self.DH_params[i][1], self.DH_params[i][2],
                                           joint[i] + self.DH_params[i][3])
	T = np.zeros((7,4,4))
	Ti = np.identity(4)
	Ti[2,3] = 0.1575
	for i in range(len(joint)):
	    Ti = np.dot(Ti, A[i])
	    T[i] = Ti
	
	    
	for i in range(len(joint)):
	    J[:3, i] = np.cross(T[i][:3, 2], T[-1][:3, -1]-T[i][:3, -1])
	    J[3:,i] = T[i][:3, 2]
	
        return J

    def get_jacobian_cm(self, joint, frame):
        ##TODO: Fill in this function to complete Q2.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## "frame" is an integer indicating the frame you wish to calculate.
        ## The output is a numpy 6*7 matrix describing the Jacobian matrix defining at the centre of mass of the specified link.
	J = np.zeros((6,7))
	A = np.zeros((7,4,4))
	for i in range(frame):
	    A[i] = self.dh_matrix_standard(self.DH_params[i][0], self.DH_params[i][1], self.DH_params[i][2],
                                           joint[i] + self.DH_params[i][3])
	T = np.zeros((7,4,4))
	Ti = np.identity(4)
	Ti[2,3] = 0.1575
	for i in range(frame):
	    Ti = np.dot(Ti, A[i])
	    T[i] = Ti
	
	Te = self.forward_kine_cm(joint,frame)
	for i in range(frame):
	    J[:3, i] = np.cross(T[i][:3, 2], Te[:3, -1]- T[i][:3, -1])
	    J[3:, i] = T[i][:3, 2]
        return J 


    def inverse_kine_ite(self, desired_pose, current_joint):
        ##TODO: Fill in this function to complete Q2.
        ## "desired_pose" is a numpy 4*4 matrix describing the transformation of the manipulator.
        ## "current_joint" is an array of double consisting of the joint value (works as a starting point for the optimisation).
        ## The output is numpy vector containing an optimised joint configuration for the desired pose.
	iteration = 1000
        a = 0.1
        joint = current_joint
        Xe = self.Te2x(desired_pose)
        for i in range(iteration):
            J = self.get_jacobian(joint)
            x = self.Te2x(self.joint2Te(joint))
            joint += (a* np.dot(np.linalg.pinv(J), (Xe-x).reshape(-1, 1))).flatten()
            error = np.sqrt(np.sum((Xe - x)**2))
            if error < 0.01:
                break
	conf = joint - 2*np.pi*(joint/(2*np.pi)).astype(int)
	return conf
        


    def inverse_kine_closed_form(self, desired_pose):
        ##TODO: Fill in this function to complete Q2.
        ## "desired_pose" is a numpy 4*4 matrix describing the transformation of the manipulator.
        ## The output is a numpy matrix consisting of the joint value for the desired pose.
        ## You may need to re-structure the input of this function.
	
        raise NotImplementedError() #Remove this line, once implemented everything
  


    def getB(self, joint):
        ##TODO: Fill in this function to complete Q2.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## The output is a numpy 7*7 matrix.
	b = []
	for i in range(len(joint)):
	    T = self.forward_kine_cm(joint, i + 1)
            J = self.get_jacobian_cm(joint, i + 1)
            j = np.dot(np.dot(T, np.diag(self.Ixyz[i])), np.transpose(T[:3, :3]))
            b.append(self.mass[i] * np.dot(np.transpose(J[:3, :]), J[:3, :]) + np.dot(np.dot(np.transpose(J[3:, :]), j), J[3:, :]))
	B = np.sum(np.array(b), axis = 0)
        return B

    def getC(self, joint, vel):
        ##TODO: Fill in this function to complete Q2.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## "vel" is a numpy array of double consisting of the joint velocity.
        ## The output is a numpy 7*7 matrix.
        B = self.getB(joint)
	inc = np.diag(np.zeros(len(joint)) + 0.00001)
	B1 = np.array([self.getB(joint + inc[i]) for i in range(len(joint))])
        dB = (B1 - B) / 0.00001
	H = dB * vel.reshape((-1, 1, 1), axis=0) - 0.5 * np.sum(dB * vel, axis=2)
	C = np.sum(H) 
        return C



    def getG(self, joint):
        ##TODO: Fill in this function to complete Q2.
        ## "joint" is a numpy array of double consisting of the joint value.
        ## The output is a numpy array 7*1.
        g = np.array([0, 0, -9.81])
        w = []
	inc = np.diag(np.zeros(len(joint)) + 0.00001)
        for i in range(len(joint)):
            p = self.forward_kine_cm(joint, i + 1)[:3, -1]
            p1 = np.array([self.forward_kine_cm(joint + inc[j], i + 1)[:3, -1] for j in range(len(joint))])
	    dp = (p1-p)/0.00001
	    w.append(self.mass[i]*np.dot(dp,g))
	G = np.sum(np.array(w),axis = 0)
        return G
