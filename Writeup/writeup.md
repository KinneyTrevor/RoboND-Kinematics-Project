## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)



## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

# DH Param table and kuka schematic drawing

[image1]: ./misc_images/kuka_drawing.jpg

I obtained this table through following allong with the lessons and drawing the schematic on my own. The paramaters were assigned using the denavit hartenberg notation.

# Transformation matrices in Python - explanation is below the table.

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
            T0_7 = simplify(T0_7 * T6_7)

            T_total = simplify(T0_7 * R_corr)

In order to aquire the transformation matrices you need to transform points for each individual join, resulting in 7 transformation. Lastly you need to rotate about the Z axis once and the Y axis once in order to ensure your gripper is on the same plane of coordinates. 

# Inverse Kinematics
[image2]: ./misc_images/theta_equations.jpg

In the above image you can see how I solved for the inverse kinematic equations for theta1, theta2, and theta3. You have to apply a your standard trig functions as well as some distance equations in order to solve for the theta values. It was easiest to draw the robot arm in a 2D format and solve it as if it were a triangle, this can be found in the top right of the above image.
