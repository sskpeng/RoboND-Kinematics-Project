#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:   
        ### Your FK code here
        joint_trajectory_list = []
        # Create symbols
        print "start cal"
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')        
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, = symbols('alpha0:7')
        r, p, y = symbols('r p y')
        #   
        # Create Modified DH parameters
        s = {alpha0:      0, a0:      0, d1:  0.75,  q1:       q1,
        alpha1: -pi/2., a1:   0.35, d2:     0,  q2:-pi/2.+q2,
        alpha2:      0, a2:   1.25, d3:     0,  q3:       q3,
        alpha3: -pi/2., a3: -0.054, d4:  1.50,  q4:       q4,
        alpha4:  pi/2., a4:      0, d5:     0,  q5:       q5,
        alpha5: -pi/2., a5:      0, d6:     0,  q6:       q6,
        alpha6:      0, a6:      0, d7: 0.303,  q7:    0}

        # Create individual transformation matrices
        T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(s)
        T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(s)
        T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(s)
        T0_3 = T0_1 * T1_2 * T2_3
        R0_3 = T0_3[0:3, 0:3]

        
        # Create rotation matrices
        ROT_x = Matrix([[       1,            0,         0],
                        [       0,       cos(r),   -sin(r)],
                        [       0,       sin(r),    cos(r)]]) # roll

        ROT_y = Matrix([[  cos(p),            0,    sin(p)],
                        [       0,            1,         0],
                        [ -sin(p),            0,    cos(p)]]) # pitch

        ROT_z = Matrix([[  cos(y),      -sin(y),         0],
                        [  sin(y),       cos(y),         0],
                        [       0,            0,         1]]) # yaw
              
        ROT_EE = ROT_z * ROT_y * ROT_x
        ROT_corr = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
        ROT_EE = ROT_EE * ROT_corr
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
            
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([req.poses[x].orientation.x, req.poses[x].orientation.y,req.poses[x].orientation.z, req.poses[x].orientation.w])
            
            # Find EE rotation matrix    
            ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
    
            EE = Matrix([[px], [py], [pz]])
            # ee_adj = Matrix([[0.303], [0], [0]])
            WC = EE - 0.303 * ROT_EE[:,2] # d7 = 0.303
        
            # Calculate joint angles using Geometric IK method
#            theta1 = atan2(WC[1], WC[0])

#            side_a = 1.501 # d4.subs(s)
#            side_b = sqrt((sqrt(WC[0]**2 + WC[1]**2) - 0.35)**2 + (WC[2] - 0.75)**2)
#            side_c = 1.25 # a2.subs(s)
#            
#            # angle_a = acos((side_b**2 + side_c**2 - side_a**2)/(2 * side_b * side_c))
#            c_a = (side_b**2 + side_c**2 - side_a**2)/(2 * side_b * side_c)
#            angle_a = atan2(sqrt(1 - c_a**2), c_a)
#            # angle_b = acos((side_a**2 + side_c**2 - side_b**2)/(2 * side_a * side_c))
#            c_b = (side_a**2 + side_c**2 - side_b**2)/(2 * side_a * side_c)
#            angle_b = atan2(sqrt(1 - c_b**2), c_b)
#            # angle_c = acos((side_a**2 + side_b**2 - side_c**2)/(2 * side_a * side_b))
#            c_c = (side_a**2 + side_b**2 - side_c**2)/(2 * side_a * side_b)
#            angle_c = atan2(sqrt(1 - c_c**2), c_c)
#            
#            theta2 = pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0]**2 + WC[1]**2) - 0.35)
#            theta3 = pi/2 - (angle_b + 0.036)
            a_3 = s[a3]
            d_4 = s[d4]
            d_1 = s[d1]
            a_1 = s[a1]
            a_2 = s[a2]
            d_7 = s[d7]

            theta1 = (atan2(WC[1], WC[0])).evalf()
        
            s1 = sqrt(WC[0]**2 + WC[1]**2) - a_1
            s2 = WC[2] - d_1
            s3 = sqrt(s2**2 + s1**2)
            s4 = sqrt(a_3**2 + d_4**2)
            beta1 = atan2(s2, s1)
    
            D2 = (a_2**2 + s3**2 - s4**2) / (2 * a_2 * s3)
            beta2 = atan2(sqrt(1 - D2**2), D2)
            
            D3 = (a_2**2 + s4**2 - s3**2) / (2 * a_2 * s4)
            beta3 = atan2(sqrt(1 - D3**2), D3)
            
            beta4 = atan2(-a_3, d_4)
    
            theta2 = ((pi / 2) - beta2 - beta1).evalf()
            theta3 = ((pi / 2) - beta4 - beta3).evalf()
            # Eular angles from rotation matrix
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R0_3_inv = R0_3.transpose()
            R3_6 = R0_3_inv * ROT_EE

            r13 = R3_6[0, 2]
            r33 = R3_6[2, 2]
            r23 = R3_6[1, 2]
            r21 = R3_6[1, 0]
            r22 = R3_6[1, 1]
            r12 = R3_6[0, 1]
            r32 = R3_6[2, 1]
        
            theta5 = (atan2(sqrt(r13**2 + r33**2), r23)).evalf()
            theta4 = (atan2(r33, -r13)).evalf()
            theta6 = (atan2(-r22, r21)).evalf()
            
#            if x == 0:
#                theta4_p = theta4
#            
#            while (theta4 - theta4_p > pi):
#                theta4 = theta4 - 2 * pi
#            while (theta4 - theta4_p < -pi):
#                theta4 = theta4 + 2 * pi 
#            
#            theta4_p = theta4
            
            ###
        
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)
        
        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)
    #            
# Define Modified DH Transformation matrix
def TF_Matrix(alpha, a, d, q):
    TF = Matrix([[      cos(q),       -sin(q),    0,      a],
       [ sin(q)*cos(alpha), cos(q)*cos(alpha),  -sin(alpha),  -sin(alpha)*d],
       [ sin(q)*sin(alpha), cos(q)*sin(alpha),   cos(alpha),   cos(alpha)*d],
       [     0,     0,    0,      1]])
    return TF

def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()


