#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

def create_ht_matrix(alpha, a, d, q):
    cq = cos(q)
    sq = sin(q)
    calpha = cos(alpha)
    salpha = sin(alpha)
    return Matrix([[          cq,            -sq,          0,              a ],
                   [ sq * calpha,    cq * calpha,    -salpha,    -salpha * d ],
                   [ sq * salpha,    cq * salpha,     calpha,     calpha * d ],
                   [           0,              0,          0,              1 ]])

def rotation_z(angle):
    ca = cos(angle)
    sa = sin(angle)
    return Matrix([[  ca,   -sa,     0 ],
                   [  sa,    ca,     0 ],
                   [   0,     0,     1 ]])

def rotation_y(angle):
    ca = cos(angle)
    sa = sin(angle)
    return Matrix([[  ca,     0,    sa ],
                   [   0,     1,     0 ],
                   [ -sa,     0,    ca ]])

def rotation_x(angle):
    ca = cos(angle)
    sa = sin(angle)
    return Matrix([[   1,     0,     0 ],
        	   [   0,    ca,   -sa ],
        	   [   0,    sa,    ca ]])

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

	# Create Modified DH parameters
        d01  = 0.75
        d34  = 1.5
        d6ee = 0.303
        a12  = 0.35
        a23  = 1.25
        a34  = -0.054

	# Define Modified DH Transformation matrix
        dh_table = {alpha0:     0,      a0:   0,      d1:  d01,      q1: q1,
                    alpha1: -pi/2,      a1: a12,      d2:    0,      q2: q2-pi/2,
                    alpha2:     0,      a2: a23,      d3:    0,      q3: q3,
                    alpha3: -pi/2,      a3: a34,      d4:  d34,      q4: q4,
                    alpha4:  pi/2,      a4:   0,      d5:    0,      q5: q5,
                    alpha5: -pi/2,      a5:   0,      d6:    0,      q6: q6,
                    alpha6:     0,      a6:   0,      d7: d6ee,      q7: 0}

	# Create individual transformation matrices
	T0_1 = create_ht_matrix(alpha0, a0, d1, q1).subs(dh_table)
        T1_2 = create_ht_matrix(alpha1, a1, d2, q2).subs(dh_table)
        T2_3 = create_ht_matrix(alpha2, a2, d3, q3).subs(dh_table)
        T3_4 = create_ht_matrix(alpha3, a3, d4, q4).subs(dh_table)
        T4_5 = create_ht_matrix(alpha4, a4, d5, q5).subs(dh_table)
        T5_6 = create_ht_matrix(alpha5, a5, d6, q6).subs(dh_table)
        T6_EE = create_ht_matrix(alpha6, a6, d7, q7).subs(dh_table)

        R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]

	# Extract rotation matrices from the transformation matrices
        T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

        # Correction to resove gripper link differences between DH and URDF convention
        T0_EE_corr = T0_EE * Matrix([[0,0,1.0,0],[0,-1.0,0,0],[1.0,0,0,0],[0,0,0,1.0]])

        # Precalculate some constants to not recalculate them in a loop
        phi_angle = atan2(-a34, d34)
        side_a = sqrt(d34**2 + a34**2)
        side_c = a23
        side_a_sq = side_a**2
        side_c_sq = side_c**2
        side_a_mul_side_c_dbl = side_a * side_c * 2
        half_pi = pi/2

        r, p, y = symbols('r p y')
        R_x = rotation_x(r)
        R_y = rotation_y(p)
        R_z = rotation_z(y)

        R_EE = R_z * R_y * R_x
        R_corr = rotation_z(pi) * rotation_y(-pi/2)
        R_EE_corr = R_EE * R_corr

        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
            R_EE_ev = R_EE_corr.subs({'r': roll, 'p': pitch, 'y': yaw})

            # Gripper position matrix
            EE = Matrix([[px], [py], [pz]])
            # Wrist position matrix
            WC = EE - d6ee * R_EE_ev[:, 2]
            wcx, wcy, wcz = (WC[0], WC[1], WC[2])

	    # Calculate joint angles using Geometric IK method
	    theta1 = atan2(wcy, wcx)

            # Solving theta2 and theta3 using triangle with sides a, b, c
            wcxy_comp = sqrt(wcx**2 + wcy**2) - a12
            wcz_comp = wcz - d01
            side_b_sq = wcxy_comp**2 + (wcz_comp)**2
            side_b = sqrt(side_b_sq)
            side_b_dbl = side_b * 2

            angle_a = acos((side_b_sq + side_c_sq - side_a_sq) / (side_c * side_b_dbl))
            angle_b = acos((side_a_sq + side_c_sq - side_b_sq) / (side_a_mul_side_c_dbl))
            angle_c = acos((side_b_sq + side_a_sq - side_c_sq) / (side_a * side_b_dbl))

            theta2 = half_pi - angle_a - atan2(wcz_comp, wcxy_comp)
            theta3 = half_pi - angle_b - phi_angle

            # Calculate end effector angles
            R0_3_ev = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
            R3_6 = R0_3_ev.transpose() * R_EE_ev

            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0, 2]**2 + R3_6[2, 2]**2), R3_6[1, 2])
            theta6 = atan2(-R3_6[1,1],R3_6[1,0])

            print "DEBUG: theta1:", theta1, "theta2:", theta2, "theta3:", theta3, "theta4:", theta4, "theta5:", theta5, "theta6:", theta6

            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
