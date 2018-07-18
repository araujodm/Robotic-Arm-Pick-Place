## Project: Kinematics Pick & Place

1. Setup your ROS Workspace.

It was done by creating the /catkin_ws. As mentioned during the course, catkin is the official build system of ROS.


2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  

https://github.com/araujodm/RoboND-Kinematics-Project/blob/patch-1/misc_images/ROS%20Workspace.png



3. Experiment with the forward_kinematics environment and get familiar with the robot.

https://github.com/araujodm/RoboND-Kinematics-Project/blob/patch-1/misc_images/Forward%20Kinematics.png



4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).

https://github.com/araujodm/RoboND-Kinematics-Project/blob/patch-1/misc_images/Kuka%20arm%20demo%20RViz.png
https://github.com/araujodm/RoboND-Kinematics-Project/blob/patch-1/misc_images/kuka%20arm%20demo%20Gazebo.png


5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).

Check out DH table here:
https://github.com/araujodm/RoboND-Kinematics-Project/blob/patch-1/misc_images/DH_Table.pdf

Here a screenshot from the urdf file:
https://github.com/araujodm/RoboND-Kinematics-Project/blob/patch-1/misc_images/kuka_arm_xacro_urdf.png

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

Before define each rotation matrix, I have reviewed the basic concept by watching some classes from www.khanacademy.org.
See some examples on these screenshots:
https://github.com/araujodm/RoboND-Kinematics-Project/blob/patch-1/misc_images/Rotation%20in%20R2.png
https://github.com/araujodm/RoboND-Kinematics-Project/blob/patch-1/misc_images/Rotation%20in%20R3.png

Now, it's clear to me how to get the new vector after its rotation according given theta angle. (Forward Kinematics)

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
                    
                    
                    
Considering there is a distance of 0.303m between WC and EE (End-Effector):

    ROT_EE = ROT_z * ROT_y * ROT_x
    
    ROT_Error = ROT_z.subs(y,radians(180)*ROT_y.subs(p,radians(-90)))

    ROT_EE = ROT_EE * ROT_Error       ===>>>> product = translation

    ROT_EE =  ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

    EE = Matrix ([[px],
                  [py],
                  [pz]])

    WC = EE - 0.303 * ROT_EE[:,2]     ====>>>          WC = ([[px], -  ( 0.303 * ROT_ EE[:,2])  => EE transladed by 0.303m  
                                                              [py],
                                                              [pz]])
                                                              
                                                              

    

6. Fill in th_e `IK_server.py` with your Inverse Kinematics code. 

Here is my code:
https://github.com/araujodm/RoboND-Kinematics-Project/blob/patch-1/IK_server.py

And comments below:

	theta1 = atan2(WC[1], WC[0])   =====>>> calculating theta between two vectors BUT I'M NOT SURE ABOUT WC MATRIX.

        #SSS triangule for theta2 and theta3

>>> I'M NOT SURE ABOUT WHICH SIDE IS A, B OR C ON THE SERIAL MANIPULATOR. COULD YOU GIVE ME A SHORT EXPLANATION SHOWING HOW DID YOU GET EACH?

        side_a = 1.501
        side_b = sqrt(pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35), 2) + pow[(WC[2]-0.75), 2])
        side_c = 1.25

Laws of cosine:

        angle_a = acos((side_b*side_b + side_c*side_c - side_a*side_a) / (2 * side_b * side_c))
        angle_b = acos((side_a*side_a + side_c*side_c - side_b*side_b) / (2 * side_a * side_c))
        angle_c = acos((side_a*side_a + side_b*side_b - side_c*side_c) / (2 * side_a * side_b))

        theta2 = pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0]*WC[0] + WC[1]*WC[1] - 0.35))
        theta3 = pi/2 - (angle_b + 0.036)  #0.036 accounts for sag in link 4 of -0.054


        R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
        R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
        R3_6 = R0_3.inv("LU") * ROT_EE

        # EULER ANGLES from rotation matrix

        theta4 = atan2(R3_6[2,2], -R3_6[0, 2])
        theta5 = atan2(sqrt(R3_6[0, 2]*R3_6[0, 2] + R3_6[2, 2]* R3_6[2, 2]), R3_6[1.25])
        theta6 = atan2( -R3_6[1, 1], R3_6[1, 0])


