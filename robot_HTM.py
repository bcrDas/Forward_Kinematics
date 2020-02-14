#For 5-DOF articulated manipulator

import numpy as np


def forward_kinematics():

    #Joint angles in degrees
    angle_1 = 0
    angle_2 = 0
    angle_3 = 0
    angle_4 = 90
    angle_5 = 0


    #Joint angles in radians
    angle_1 = (angle_1/180)*np.pi
    angle_2 = (angle_2/180)*np.pi
    angle_3 = (angle_3/180)*np.pi
    angle_4 = (angle_4/180)*np.pi
    angle_5 = (angle_5/180)*np.pi


    #Link lengths(in mm.)
    link_1 = 500
    link_2 = 400
    link_3 = 300
    link_4 = 200
    link_5 = 50


    # Rotation matrix
    I_0 = [[1,0,0],[0,0,-1],[0,1,0]]
    R0_1_temp = [[np.cos(angle_1),-np.sin(angle_1),0],[np.sin(angle_1),np.cos(angle_1),0],[0,0,1]]
    R0_1 = np.dot(R0_1_temp,I_0)

    I_1 = [[1,0,0],[0,1,0],[0,0,1]]
    R1_2_temp = [[np.cos(angle_2),-np.sin(angle_1),0],[np.sin(angle_2),np.cos(angle_2),0],[0,0,1]]
    R1_2 = np.dot(I_1,R1_2_temp) # or R1_2 = np.dot(R0_1_temp,I_0)

    I_2 = [[1,0,0],[0,1,0],[0,0,1]]
    R2_3_temp = [[np.cos(angle_3),-np.sin(angle_3),0],[np.sin(angle_3),np.cos(angle_3),0],[0,0,1]]
    R2_3 = np.dot(I_2,R2_3_temp)

    I_3 = [[0,0,1],[1,0,0],[0,1,0]]
    R3_4_temp = [[np.cos(angle_4),-np.sin(angle_4),0],[np.sin(angle_4),np.cos(angle_4),0],[0,0,1]]
    R3_4 = np.dot(R3_4_temp,I_3)

    I_4 = [[1,0,0],[0,1,0],[0,0,1]]
    R4_5_temp = [[np.cos(angle_5),-np.sin(angle_5),0],[np.sin(angle_5),np.cos(angle_5),0],[0,0,1]]
    R4_5 = np.dot(I_4,R4_5_temp)


    # Translation matrix
    D0_1 = [[0],[0],[link_1]]
    D1_2 = [[link_2*np.cos(angle_2)],[link_2*np.sin(angle_2)],[0]]
    D2_3 = [[link_3*np.cos(angle_3)],[link_3*np.sin(angle_3)],[0]]
    D3_4 = [[0],[0],[0]] # According to convention [[0],[0],[0]] *** Doubt
    D4_5 = [[0],[0],[link_5]]


    # Homogeneous transformation matrix
    H0_1_temp = np.concatenate((R0_1,D0_1),1)
    H0_1 = np.concatenate((H0_1_temp,[[0,0,0,1]]),0)

    H1_2_temp = np.concatenate((R1_2,D1_2),1)
    H1_2 = np.concatenate((H1_2_temp,[[0,0,0,1]]),0)

    H2_3_temp = np.concatenate((R2_3,D2_3),1)
    H2_3 = np.concatenate((H2_3_temp,[[0,0,0,1]]),0)

    H3_4_temp = np.concatenate((R3_4,D3_4),1)
    H3_4 = np.concatenate((H3_4_temp,[[0,0,0,1]]),0)

    H4_5_temp = np.concatenate((R4_5,D4_5),1)
    H4_5 = np.concatenate((H4_5_temp,[[0,0,0,1]]),0)

    H0_2 = np.dot(H0_1,H1_2)
    H0_3 = np.dot(H0_2,H2_3)
    H0_4 = np.dot(H0_3,H3_4)
    H0_5 = np.dot(H0_4,H4_5)

    print("\n The orientation and position of the end-effector in terms of HTM with respect to base is : \n")
    print(np.matrix(H0_5))
    print("\n")






if __name__ == '__main__':

    forward_kinematics()


