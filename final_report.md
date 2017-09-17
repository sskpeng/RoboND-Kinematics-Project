## Project: Kinematics Pick & Place


[//]: # (Image References)

[image1]: ./writeup_pic/pic01.png
[image2]: ./writeup_pic/pic02.png
[image3]: ./writeup_pic/pic03.png
[image4]: ./writeup_pic/pic04.png

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The joint coordinates and their translation relations between adjacent coordinates are defined according to the course material and shown on the picture below.

![alt text][image1]

1. alpha<sub>i-1</sub> is rotation offset about X<sub>i-1</sub>.
2. a<sub>i-1</sub> is translation offset between Z<sub>i-1</sub> and Z<sub>i</sub>.
3. d<sub>i</sub> is translation offset between X<sub>i-1</sub> and X<sub>i</sub>.
4. theta<sub>i</sub> is rotation offset about Z<sub>i</sub> axis.

The D-H parameters of the robot are collected on the tablet below.

Links | alpha(i-1) | a(i-1) | d(i
) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q<sub>1</sub>
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q<sub>2</sub>
2->3 | 0 | 1.25 | 0 | q<sub>3</sub>
3->4 | - pi/2 | -0.054 | 1.50 | q<sub>4</sub>
4->5 |   pi/2 | 0 | 0 | q<sub>5</sub>
5->6 | - pi/2 | 0 | 0 | q<sub>6</sub>
6->EE | 0 | 0 | 0.303 | 0

These values can be found from the robot urdf.xaro file. Note that, in the joints section, the origin location of each joint is described with respect to the previous (parent) joint. It is desired to combine offsets on two adjacent joints but along the same axis into one offset to reduce the none zero itenitiy on the D-H table. For example, for joint 1 and joint 2, the Z offsets are combined (0.33 + 0.42 = 0.75) and placed on just joint 2

![alt text][image3]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base\_link and gripper\_link using only end-effector(gripper) pose.

The individual transformation matrix in each joint can be formulated as a function code below, and the transformation matrix from base\_link to gripper\_link is simply multipling all the individual matrics together.

![alt text][image4]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

A desired end effecotr is consisted of position and orientation. Thanks to the mechancial structure design, the inverse orientation kinematic problem can be achieved by the 3 joints at the wrist unit (J4 to J6) , and the inverse position kinematic problem with 6 joint variables for the desired end effector (EE) can be simplified to find the wrist center (WC) position with 3 joint variables (J1 to J3). The following formula is used to compute the desired WC position based on the desired EE position and orientation.

```sh
POS_WC = POS_EE - ROT_EE * (joint_6 length)
```
##### Inverse position
The inverse position question is to find J1, J2, and J3 values for the desired WC position. Joint 1 value can be found by X and Y values of WC.

```
theta1 = atan2(WC[1], WC[0]) # WC[0] = WC_x, WC[1] = WC_y, WC[2] = WC_z
```
For joint 2 and joint 3, the cos rule _cosC = (a<sup>2</sup> + b<sup>2</sup> - c<sup>2</sup>) / 2ab_ is used. Refer to the picture below. 

![alt text][image2]
Note that angle between a<sub>2</sub> and d<sub>4</sub> is a right angle.

In the triangle AA\_BB\_CC, all the three length are known values and the three inner angles can be calcated. Then joint 2 and joint 3 can be found.

```
theta2 = pi/2 - angle_a - angle_T
theta3 = pi/2 - (angle_b + angle_S)
```
##### Inverser orientation
With J1 to J3, ROT0\_3 and ROT3\_6 can be calculated. Then J4 to J6 can be found in the matrix.

```
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


In the IK_server.py, the handle_calcuate_IK function is called to calculate the inverse problem.
In the handle_calculate_IK function, first, the D-H table and transformation matrices are formed. Then a for loop is used to compute the joint values for each requested pos (`req.pos`).



