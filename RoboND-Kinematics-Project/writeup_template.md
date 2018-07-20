## Project: Kinematics Pick & Place

1. Setup ROS Workspace by creating /catkin_ws workspace.

2. Cloning the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  

3. Experiment with the forward_kinematics environment and get familiar with the robot.

4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).

5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
Check out DH table here:

  ### DH Parameters
        DH_Table = {alpha0:     0,  a0:      0, d1:  0.75, q1:         q1,
                    alpha1: -pi/2,  a1:   0.35, d2:     0, q2: -pi/2 + q2,
                    alpha2:     0,  a2:   1.25, d3:     0, q3:         q3,
                    alpha3: -pi/2,  a3: -0.054, d4:   1.5, q4:         q4,
                    alpha4:  pi/2,  a4:      0, d5:     0, q5:         q5,
                    alpha5: -pi/2,  a5:      0, d6:     0, q6:         q6,
                    alpha6:     0,  a6:      0, d7: 0.303, q7:          0}

# URDF file can be found in the attached PDF file.

Here the individual transformation matrices about each joint:


        TF =  Matrix([[           cos(q),           -sin(q),           0,             a]
                      [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d]
                      [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d]
                      [                0,                 0,           0,             1]])

    T0_1 = TF_Matrix(alpha0, a0, d1, q1)
    T1_2 = TF_Matrix(alpha1, a1, d2, q2)
    T2_3 = TF_Matrix(alpha2, a2, d3, q3)
    T3_4 = TF_Matrix(alpha3, a3, d4, q4)
    T4_5 = TF_Matrix(alpha4, a4, d5, q5)
    T5_6 = TF_Matrix(alpha5, a5, d6, q6)
    T6_EE = TF_Matrix(alpha6, a6, d7, q7)

The generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose is:

    T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE



Then, here we have each rotation matrix:

    r, p, y = symbols('r p y')

    ROT_x = Matrix([[       1,      0,       0]
                    [       0, cos(r), -sin(r)]
                    [       0, sin(r),  cos(r)]]) # ROW


    ROT_y = Matrix([[  cos(p),      0,  sin(p)]
                    [       0,      1,       0]
                    [ -sin(p),      0,  cos(p)]]) #PITCH

    ROT_z = Matrix([[  cos(y), -sin(y),      0]
                    [  sin(y),  cos(y),      0]
                    [       0,       0,      1]]) # YAW
                   	
############    IK code start HERE   ####################

Calculate joint angles using Geometric IK method
 Calculating positions of the wrist center

            ROT_EE = R_z * R_y * R_x

 Compensate for rotation discrepancy between DH parameters and Gazebo
   
            Rot_correction = R_z.subs(y,pi)*R_y.subs(p,-pi/2)

            ROT_EE = ROT_EE * Rot_correction
            ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

            end_effector_pos = Matrix([px, py, pz])
            wrist_center = end_effector_pos - (0.303) * ROT_EE[:,2]

            Wx, Wy, Wz = wrist_center[0], wrist_center[1], wrist_center[2]

SS triangle for theta2 and theta3
         
            side_a = 1.50
            side_b = sqrt(pow(sqrt(Wx**2 + Wy**2) - 0.35,2) + pow((Wz - 0.75),2))
            side_c = 1.25

            angle_a = acos((side_b**2 + side_c**2 - side_a**2)/(2*side_b*side_c))
            angle_b = acos((side_a**2 + side_c**2 - side_b**2)/(2*side_a*side_c))
            angle_c = acos((side_a**2 + side_b**2 + side_c**2)/(2*side_a*side_b))

Finding the first three joint angles using trigonometry

            theta1 = atan2(Wy, Wx)
            theta2 = pi/2 - angle_a - atan2((Wz - 0.75), sqrt(Wx**2 + Wy**2) - 0.35)
            theta3 = pi/2 - angle_b + 0.036

Finding the last three joint angles 4, 5, 6

            R0_3 = T0_1[0:3,0:3]*T1_2[0:3,0:3]*T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
           
            R3_6 = R0_3.inv('LU') * ROT_EE

Euler angles from rotation matrix

            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
  
  ############   IK code finish HERE     #########################
