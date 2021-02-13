#!/usr/bin/env python

import rospkg
from math import pi
import rosbag
import rospy
import tf2_kdl
import tf2_ros

from cw3q2.iiwa14Kine import iiwa14_kinematic

if __name__ == '__main__':
    rospy.init_node('cw3q1_node')
    iiwa14 = iiwa14_kinematic()
    
    p0 = [0, 0, 0, 0, 0, 0, 0]
    p1 = [0.768506, 0.121759, -0.939627, 1.99111, -0.200346, -1.35671, -0.790215]
    v1 = [-0.033846, 0.0564997, 0.295151, 0.271778, 0.259246, 0.0124612, -0.0292725]

    p2 = [-1.58425, -0.272908, 0.796272, 2.37887, 0.0842769, -0.0436095, 1.05386]
    v2 = [-0.327998, -0.166912, 0.147049, -0.339939, -0.0167218, -0.272989, 0.0179721]

    p3 = [-2.13286, 0.266082, 0.942719, 2.37668, -2.04084, -0.598984, -0.832711]
    v3 = [0.203643, 0.130777, 0.0819627, 0.181411, 0.033077, -0.00593176, 0.0985884]

    #JOINT1
    T = iiwa14.forward_kine(p1, 6)
    T_cm = iiwa14.forward_kine_cm(p1, 6)

    jacobian_cm = iiwa14.get_jacobian_cm(p1,6)
    IK_ite = iiwa14.inverse_kine_ite(T, p0)
    T_ite = iiwa14.forward_kine(IK_ite, 6)
    B = iiwa14.getB(p1)
    C = iiwa14.getC(p1, v1)
    G = iiwa14.getG(p1)
    print("///// JOINT 1 /////")
    print("Position: ")
    print(p1)
    print("Velocity: ")
    print(v1)
    print("T_cm is: ")
    print(T_cm)

    print("jacobian_cm: ")
    print(jacobian_cm)
    print("The iterative form IK is: ")
    print(IK_ite)
    print("B is: ")
    print(B)
    print("C is: ")
    print(C)
    print("G is: ")
    print(G)
    print("\n")


    
