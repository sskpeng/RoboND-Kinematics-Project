from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''


TESTPOS = [[2.043105518,-0.000134841,1.946017037,-6.83E-05,-8.62E-05,-6.60E-05],
[2.04676184,0.011441642,1.939982637,0.000577693,0.009061308,0.00544537],
[2.050324864,0.02299681,1.933939356,0.001217882,0.018196436,0.010632979],
[2.057089095,0.045747845,1.921990568,0.002442539,0.036149227,0.019927391],
[2.063498319,0.068408759,1.910037059,0.00357839,0.053909178,0.027968241],
[2.075205668,0.113161316,1.886347259,0.005408747,0.088251959,0.040275436],
[2.085588822,0.157543414,1.862866269,0.00643093,0.120716517,0.047829994],
[2.096775308,0.212458935,1.834062507,0.006234755,0.157512985,0.050955411],
[2.106086614,0.266910674,1.806061988,0.004189404,0.188919382,0.047719935],
[2.113518315,0.320264508,1.779474118,0.000409138,0.213395692,0.03924927],
[2.119291418,0.373465901,1.754061593,-0.00482053,0.230446331,0.026712708],
[2.122748589,0.416040824,1.734676233,-0.009662516,0.238201722,0.014724812],
[2.125188244,0.458760938,1.716180186,-0.014666186,0.240419952,0.001896117],
[2.126025405,0.480186005,1.707287383,-0.017084811,0.23940103,-0.004545671],
[2.126607057,0.501692845,1.698625814,-0.019373591,0.236953774,-0.01086094],
[2.126800714,0.512454503,1.694393175,-0.020447306,0.235199363,-0.013925989],
[2.126929816,0.523242279,1.690218131,-0.021467365,0.233090863,-0.016917271],
[2.126966456,0.54950764,1.680352217,-0.023641228,0.226525793,-0.023723435],
[2.126613353,0.575964198,1.670818229,-0.025369886,0.217931524,-0.029823364],
[2.125860538,0.602638072,1.661609715,-0.026571545,0.207355948,-0.035032579],
[2.124696021,0.629554438,1.652717575,-0.027176533,0.194862058,-0.039176458],
[2.123077485,0.657100821,1.644027178,-0.027099222,0.180342383,-0.042082181],
[2.121005578,0.684943298,1.635633518,-0.026313227,0.164032063,-0.043573617],
[2.118461877,0.713103084,1.627519258,-0.024794476,0.146035434,-0.043508547],
[2.115426149,0.741599178,1.619664599,-0.022536357,0.126466973,-0.041758289],
[2.112290803,0.767260897,1.612875998,-0.019904738,0.107837617,-0.038676325],
[2.108731945,0.793211662,1.60625813,-0.016715032,0.088154202,-0.034095617],
[2.10473229,0.819459388,1.599792639,-0.012995704,0.067512494,-0.027949746],
[2.100273818,0.846010004,1.593459962,-0.008784407,0.046011751,-0.020178454],
[2.097869444,0.859376925,1.590344557,-0.006505046,0.035006849,-0.015663607],
[2.095343861,0.872820334,1.587254287,-0.004121374,0.023827222,-0.010724848],
[2.092694685,0.886340354,1.584186435,-0.001640259,0.012485959,-0.005356091],
[2.089919511,0.899936969,1.581138233,0.000931018,0.000996302,0.000448508]]


def test_code(POS):
    ## Set up code
    ## Do not modify!

    # x = 0
    
    
    ########################################################################################
    ## 

    ## Insert IK code here!
    theta1 = 0
    theta2 = 0
    theta3 = 0
    theta4 = 0
    theta5 = 0
    theta6 = 0
    ### Your FK code here
        # Create symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6, = symbols('alpha0:7')
    #
    #   
    # Create Modified DH parameters
    s = {alpha0:      0, a0:      0, d1:  0.75,  q1:       q1,
         alpha1: -pi/2., a1:   0.35, d2:     0,  q2:-pi/2.+q2,
         alpha2:      0, a2:   1.25, d3:     0,  q3:       q3,
         alpha3: -pi/2., a3: -0.054, d4:  1.50,  q4:       q4,
         alpha4:  pi/2., a4:      0, d5:     0,  q5:       q5,
         alpha5: -pi/2., a5:      0, d6:     0,  q6:       q6,
         alpha6:      0, a6:      0, d7: 0.303,  q7:        0}
    #
    #
    # Create individual transformation matrices
    def TF_Matrix(alpha, a, d, q):
        TF = Matrix([[          cos(q),           -sin(q),            0,              a],
                   [ sin(q)*cos(alpha), cos(q)*cos(alpha),  -sin(alpha),  -sin(alpha)*d],
                   [ sin(q)*sin(alpha), cos(q)*sin(alpha),   cos(alpha),   cos(alpha)*d],
                   [                 0,                 0,            0,              1]])
        return TF
    T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(s)
    T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(s)
    T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(s)
    T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(s)
    T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(s)
    T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(s)
    T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(s)

    T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

    # Define RPY rotation matices
    px, py, pz, roll, pitch, yaw = symbols('px, py, pz, r, p, y')

    ROT_x = Matrix([[       1,            0,         0],
                    [       0,       cos(roll),   -sin(roll)],
                    [       0,       sin(roll),    cos(roll)]]) # roll

    ROT_y = Matrix([[  cos(pitch),            0,    sin(pitch)],
                    [       0,            1,         0],
                    [ -sin(pitch),            0,    cos(pitch)]]) # pitch

    ROT_z = Matrix([[  cos(yaw),      -sin(yaw),         0],
                    [  sin(yaw),       cos(yaw),         0],
                    [       0,            0,         1]]) # yaw
                    
    ROT_EE = ROT_z * ROT_y * ROT_x
    ROT_corr = ROT_EE * ROT_z.subs(yaw, radians(180)) * ROT_y.subs(pitch, radians(-90))

    EE = Matrix([[px], [py], [pz]])    
    WC = EE - 0.303 * ROT_EE[:,2] # d7 = 0.303
    
    theta1 = atan2(WC[1], WC[0])

    side_a = 1.501 # d4.subs(s)
    side_b = simplify(sqrt((sqrt(WC[0]**2 + WC[1]**2) - 0.35)**2 + (WC[2] - 0.75)**2))
    side_c = 1.25 # a2.subs(s)

    #angle_a = simplify(acos((side_b**2 + side_c**2 - side_a**2)/(2 * side_b * side_c)))
    #angle_b = simplify(acos((side_a**2 + side_c**2 - side_b**2)/(2 * side_a * side_c)))
    #angle_c = simplify(acos((side_a**2 + side_b**2 - side_c**2)/(2 * side_a * side_b)))

    #theta2 = simplify(pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0]**2 + WC[1]**2) - 0.35))
    #theta3 = simplify(pi/2 - (angle_b + 0.036))
    
    T0_4 = T0_1 * T1_2 * T2_3* T3_4
    R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
    # R0_3_inv = simplify(R0_3.inv("LU"))
    
    # R3_6 = simplify(R0_3.inv("LU") * ROT_EE)

    # Extract end-effector position and orientation from request
    # px, py, pz = end-effector position
    # roll, pitch, yaw = end-effectro orientation
    for x in range(len(POS)):
        start_time = time()
        pos = {px: POS[x][0], py: POS[x][1], pz: POS[x][2],
            roll: POS[x][3], pitch: POS[x][4], yaw: POS[x][5]}

        print "No. ", x, ", POS =", px, py, pz, roll, pitch, yaw

        #EE = Matrix([[px], [py], [pz]])    
        #WC = EE - 0.303 * ROT_EE[:,2] # d7 = 0.303
        EE = EE.subs(pos)
        WC = WC.subs(pos)
        #
        # Calculate joint angles using Geometric IK method
        # for JT1, JT2, JT3
        side_b = side_b.subs(pos) # (sqrt((sqrt(WC[0]**2 + WC[1]**2) - 0.35)**2 + (WC[2] - 0.75)**2))

        angle_a = (acos((side_b**2 + side_c**2 - side_a**2)/(2 * side_b * side_c)))
        angle_b = (acos((side_a**2 + side_c**2 - side_b**2)/(2 * side_a * side_c)))
        angle_c = (acos((side_a**2 + side_b**2 - side_c**2)/(2 * side_a * side_b)))

        theta1 = theta1.subs(pos)
        theta2 = (pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0]**2 + WC[1]**2) - 0.35))
        theta3 = (pi/2 - (angle_b + 0.036))
    
    # for JT4, JT5, JT6
        R0_3 = R0_3.evalf(subs = {q1: theta1, q2: theta2, q3: theta3})
        R0_3_inv = simplify(R0_3.inv("LU"))
        R3_6 = R0_3_inv * ROT_EE.subs(pos)

        r13 = R3_6[0, 2]
        r33 = R3_6[2, 2]
        r23 = R3_6[1, 2]
        r21 = R3_6[1, 0]
        r22 = R3_6[1, 1]
        r12 = R3_6[0, 1]
        r32 = R3_6[2, 1]
        
        theta5 = (atan2(sqrt(r13**2 + r33**2), r23)).evalf()
        
        if (sin(theta5) < 0):
            # print("BELOW!!!")
            theta4 = (atan2(-r33, r13)).evalf()
            theta6 = (atan2(r22, -r21)).evalf()
        elif (theta5 == 0):
            # print("EQUAL!!!")
            theta4 = 0
            theta6 = (atan2(-r12, -r32)).evalf()
        else:   
            # print("ELSE!!!!!")
            theta4 = (atan2(r33, -r13)).evalf()
            theta6 = (atan2(-r22, r21)).evalf()
            
        while (theta4 > pi):
            theta4 = theta4 - 2*pi
        while (theta4 < -pi):
            theta4 = 2*pi + theta4           
        while (theta5 > pi):
            theta5 = theta5 - 2*pi
        while (theta5 < -pi):
            theta5 = 2*pi + theta5           
        while (theta6 > pi):
            theta6 = theta6 - 2*pi
        while (theta6 < -pi):
            theta6 = 2*pi + theta6

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
        FK = T0_EE.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
        FK3 = T0_4.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    # your_wc = [1,1,1] # <--- Load your calculated WC values in this array
    # your_ee = [1,1,1] # <--- Load your calculated end effector value from your forward kinematics
        rqst_wc = [WC[0], WC[1], WC[2]]
        your_wc = [FK3[0,3], FK3[1,3], FK3[2,3]]
    # print "WC =", WC
    # print "FK3 =", FK3[0,3], FK3[1,3], FK3[2,3]
    # print "theta =", theta4, ", ", theta5, ", ", theta6
    #print 'WC=',WC
    #print 'T03=',FK3[0,3], FK3[1,3], FK3[2,3]
        your_ee = [FK[0,3], FK[1,3], FK[2,3]]
    ########################################################################################

    ## Error analysis
        print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

#    # Find WC error
#    if not(sum(your_wc)==3):
#        wc_x_e = abs(your_wc[0]-rqst_wc[0])
#        wc_y_e = abs(your_wc[1]-rqst_wc[1])
#        wc_z_e = abs(your_wc[2]-rqst_wc[2])
#        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
#        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
#        print ("Wrist error for y position is: %04.8f" % wc_y_e)
#        print ("Wrist error for z position is: %04.8f" % wc_z_e)
#        print ("Overall wrist offset is: %04.8f units" % wc_offset)


    # Find FK EE error
        if not(sum(your_ee)==3):
            ee_x_e = abs(your_ee[0]-POS[x][0])
            ee_y_e = abs(your_ee[1]-POS[x][1])
            ee_z_e = abs(your_ee[2]-POS[x][2])
            ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
            print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
            print ("End effector error for y position is: %04.8f" % ee_y_e)
            print ("End effector error for z position is: %04.8f" % ee_z_e)
            print ("Overall end effector offset is: %04.8f units \n" % ee_offset)

if __name__ == "__main__":
    # Change test case number for different scenarios
    test_code(TESTPOS)