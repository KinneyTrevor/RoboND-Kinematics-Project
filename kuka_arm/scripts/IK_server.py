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
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:q8')
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:d8')

            
            # Joint angle symbols
            # Video started these at 0 for some reason so if shit breaks later try that
            a1, a2, a3, a4, a5, a6, a7 = symbols('a1:a8')
            alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, alpha7 = symbols('alpha1:alpha8')

      
            # Modified DH params
            s = {alpha1:        0, a1:           0, d1:   0.75,
                alpha2:     -pi/2, a2:        0.35, d2:      0,    q2: q2 - pi/2,
                alpha3:         0, a3:        1.25, d3:      0,
                alpha4:     -pi/2, a4:      -0.054, d4:   1.50,
                alpha5:      pi/2, a5:           0, d5:      0,
                alpha6:     -pi/2, a6:           0, d6:      0,
                alpha7:         0, a7:           0, d7:  0.303,             q7: 0}


            
            # Define Modified DH Transformation matrix



            # Create individual transformation matrices

            # Transformation Matrix for 0 to 1
            T0_1 = Matrix([[                 cos(q1),               -sin(q1),            0,               a1],
                            [    sin(q1)*cos(alpha1),    cos(q1)*cos(alpha1), -sin(alpha1),  -sin(alpha1)*d1],
                            [    sin(q1)*sin(alpha1),    cos(q1)*sin(alpha1),  cos(alpha1),   cos(alpha1)*d1],
                            [                      0,                      0,            0,                 1,]])
            T0_1 = T0_1.subs(s)

            # Transformation Matrix for 1 to 2
            T1_2 = Matrix([[                 cos(q2),               -sin(q2),            0,               a2],
                            [    sin(q2)*cos(alpha2),    cos(q2)*cos(alpha2), -sin(alpha2),  -sin(alpha2)*d2],
                            [    sin(q2)*sin(alpha2),    cos(q2)*sin(alpha2),  cos(alpha2),   cos(alpha2)*d2],
                            [                      0,                      0,            0,                 1,]])   
            T1_2 = T1_2.subs(s)
            
            # Transformation Matrix for 2 to 3        
            T2_3 = Matrix([[                 cos(q3),               -sin(q3),            0,               a3],
                            [    sin(q3)*cos(alpha3),    cos(q3)*cos(alpha3), -sin(alpha3),  -sin(alpha3)*d3],
                            [    sin(q3)*sin(alpha3),    cos(q3)*sin(alpha3),  cos(alpha3),   cos(alpha3)*d3],
                            [                      0,                      0,            0,                 1,]])
            T2_3 = T2_3.subs(s)

            # Transformation Matrix for 3 to 4
            T3_4 = Matrix([[                 cos(q4),               -sin(q4),            0,               a4],
                            [    sin(q4)*cos(alpha4),    cos(q4)*cos(alpha4), -sin(alpha4),  -sin(alpha4)*d4],
                            [    sin(q4)*sin(alpha4),    cos(q4)*sin(alpha4),  cos(alpha4),   cos(alpha4)*d4],
                            [                      0,                      0,            0,                 1,]]) 
            T3_4 = T3_4.subs(s)

            # Transfortmation Matrix for 4 to 5
            T4_5 = Matrix([[                 cos(q5),               -sin(q5),            0,               a5],
                            [    sin(q5)*cos(alpha5),    cos(q5)*cos(alpha5), -sin(alpha5),  -sin(alpha5)*d5],
                            [    sin(q5)*sin(alpha5),    cos(q5)*sin(alpha5),  cos(alpha5),   cos(alpha5)*d5],
                            [                      0,                      0,            0,                 1,]])
            T4_5 = T4_5.subs(s)

            # Transformation Matrix for 5 to 6
            T5_6 = Matrix([[                 cos(q6),               -sin(q6),            0,               a6],
                            [    sin(q6)*cos(alpha6),    cos(q6)*cos(alpha6), -sin(alpha6),  -sin(alpha6)*d6],
                            [    sin(q6)*sin(alpha6),    cos(q6)*sin(alpha6),  cos(alpha6),   cos(alpha6)*d6],
                            [                      0,                      0,            0,                 1,]])
            T5_6 = T5_6.subs(s)

            # Transformation Matrix for 6 to 7
            T6_7 = Matrix([[                 cos(q7),               -sin(q7),            0,               a7],
                            [    sin(q7)*cos(alpha7),    cos(q7)*cos(alpha7), -sin(alpha7),  -sin(alpha7)*d7],
                            [    sin(q7)*sin(alpha7),    cos(q7)*sin(alpha7),  cos(alpha7),   cos(alpha7)*d7],
                            [                      0,                      0,            0,                 1,]])
            T6_7 = T6_7.subs(s)

            # Z Axix Rotation Matrix
            Z_Rot = Matrix([[       cos(np.pi),     -sin(np.pi),            0,           0],
                            [       sin(np.pi),      cos(np.pi),            0,           0],
                            [                0,               0,            1,           0],
                            [                0,               0,            0,           1]])                                 
            
            # Y Axis Rotation Matrix
            Y_Rot = Matrix([[    cos(-np.pi/2),               0,    sin(-np.pi/2),           0],
                            [                0,               1,                0,           0],
                            [   -sin(-np.pi/2),               0,    cos(-np.pi/2),           0],
                            [                0,               0,                0,           1]])
            R_corr = simplify(Z_Rot * Y_Rot)

            T0_2 = simplify(T0_1 * T1_2)
            T0_3 = simplify(T0_2 * T2_3)
            T0_4 = simplify(T0_3 * T3_4)
            T0_5 = simplify(T0_4 * T4_5)
            T0_6 = simplify(T0_5 * T5_6)
            # 7 is end effector
            T0_7 = simplify(T0_7 * T6_7)

            T_total = simplify(T0_7 * R_corr)

            # Extract end-effector position and orientation from request
        # px,py,pz = end-effector position
        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x 
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])


            r, p, y = symbols('r p y')

            # Roll
            ROT_X_EE = Matrix([
                [1,         0,            0],
                [0,     cos(r),     -sin(r)],
                [0,     sin(r),      cos(r)]
                                ])

            # Pitch
            ROT_Y_EE = Matrix([
                [ cos(p),    0,      sin(p)],
                [      0,    1,           0],
                [-sin(p),    0,      cos(p)]
                                ])

            # Yaw dude
            ROT_Z_EE = Matrix([
                [cos(y),    -sin(y),    0],
                [sin(y),     cos(y),    0],
                [     0,          0,    1]
                                ])

            # Multiply the rotation matrics
            ROT_EE = ROT_Z_EE * ROT_Y_EE * ROT_X_EE

            # Account for error, same as R_Corr above
            Rot_err = ROT_Z_EE.subs(y, radians(180)) * ROT_Y_EE.subs(p, radians(90))

            # Multiply
            ROT_EE = ROT_EE * Rot_err

            #Plug in them values
            ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

            # End Effector
            EE = Matrix([
                [px],
                [py],
                [pz]
                ])

            # Wrist Center
            Wr_C = EE - (0.303) * ROT_EE[:,2]

            # Calculate joint angles using Geometric IK method
            
            # Trig stuff for solving for theta2 and theta3
            side1 = 1.50097
            side2 = sqrt(pow((sqrt(Wr_C[0] * Wr_C[0] * Wr_C[1]) - 0.35), 2) + pow((Wr_C[2] - 0.75), 2))
            side3 = 1.25

            angle1 = acos((side2**2 + side3**2 - side1**2) / (2 * side2 * side3))
            angle2 = acos((side1**2 + side3**2 - side2**2) / (2 * side1 * side3))
            angle3 = acos((side1**2 + side2**2 - side3**2) / (2 * side1 * side2))

            
            theta1 = atan2(Wr_C[1],Wr_C[0])
            theta2 = pi/2 - angle1 - atan2(Wr_C[2] - 0.75, sqrt(Wr_C[0] * Wr_C[0] + Wr_C[1] * Wr_C[1]) - 0.35)
            theta3 = pi/2 - (angle2 + 0.36)

            # get first three trans matrics
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})

            # Last 3
            R3_6 = R0_3.inv("LU") * ROT_EE

            # Final Three thetas
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])


            # This is all fake news but I spent so much time getting it wrong I felt like I should leave it

            # L1 = 1.25 # Link lenght between joint 2 and 3
            # L2 = 1.50097 # Link length between joint 3 and wrist center
            # D1 = 0.35 # Distance between x axis and link 2
            # D2 = 0.75 # Distance between z axis and link 2
            # a1 = 0.09 # Link lenght between joint 1 and join 2
            # a2 = L1+L2 # This is probably wrong


            # #Set points to be on same plane
            # R_2 = R_C - D1
            # Z_2 = pz - D2

            # D2 = (px**2 + pz**2 - a1**2 - a2**2)/(2*a1*a2)
            # theta2 = atan2(D2, (1-D**2)**(1/2))

            # # uhh
            # D3 = (R_2**2 + Z_2**2 - L_1**2 - L_2**2)/(2 * L_1 * L_2)
            # theta3 = atan2((-sqrt(1-D3**2)),D3)




        


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
