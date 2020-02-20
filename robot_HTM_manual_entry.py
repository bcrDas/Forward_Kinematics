#For 5-DOF articulated manipulator

import numpy as np


def enter_angles():

        print("Enter the 1st Joint angle : ")
        angle_1 = int(input())
        print("\n")

        print("Enter the 2nd Joint angle : ")
        angle_2 = int(input())
        print("\n")

        print("Enter the 3rd Joint angle : ")
        angle_3 = int(input())
        print("\n")

        print("Enter the 4th Joint angle : ")
        angle_4 = int(input())
        print("\n")

        print("Enter the 5th Joint angle : ")
        angle_5 = int(input())
        print("\n")

        return angle_1,angle_2,angle_3,angle_4,angle_5


def enter_link_lenghts():
    print("Enter the 1st link length : ")
    link_1 = int(input())
    print("\n")

    print("Enter the 1st link length : ")
    link_2 = int(input())
    print("\n")

    print("Enter the 1st link length : ")
    link_3 = int(input())
    print("\n")

    print("Enter the 1st link length : ")
    link_4 = int(input())
    print("\n")

    print("Enter the 1st link length : ")
    link_5 = int(input())
    print("\n")

    return link_1,link_2,link_3,link_4,link_5


def forward_kinematics(angle_1,angle_2,angle_3,angle_4,angle_5,link_1,link_2,link_3,link_4,link_5,angle_flag,link_flag):

    if (angle_flag != 1):
        print("Angle is not manually defined.")

         #Joint angles in degrees
        angle_1 = 0
        angle_2 = 0
        angle_3 = 0
        angle_4 = 0
        angle_5 = 90

    elif (link_flag != 1):
        print("Link length is not manually defined.")

        #Link lengths(in mm.)
        link_1 = 500
        link_2 = 400
        link_3 = 300
        link_4 = 200
        link_5 = 50



    #Joint angles in radians
    angle_1 = (angle_1/180)*np.pi
    angle_2 = (angle_2/180)*np.pi
    angle_3 = (angle_3/180)*np.pi
    angle_4 = (angle_4/180)*np.pi
    angle_5 = (angle_5/180)*np.pi



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

    print("\n")
    print("Enter your choice(for manually enter angle(press 1) and link length(press 2)/ for both press 0/for using default joint angle and link length press 9) : ")
    choice = int(input())
    print("\n")

    angle_flag = 0
    link_flag = 0


    if(choice == 1):
        angle_1,angle_2,angle_3,angle_4,angle_5 = enter_angles()
        angle_flag = 1
        link_1 = 0
        link_2 = 0
        link_3 = 0
        link_4 = 0
        link_5 = 0
        forward_kinematics(angle_1,angle_2,angle_3,angle_4,angle_5,link_1,link_2,link_3,link_4,link_5,angle_flag,link_flag) 

    elif(choice == 2):
        link_1,link_2,link_3,link_4,link_5 = enter_link_lenghts()
        link_flag = 1
        angle_1 = 0
        angle_2 = 0
        angle_3 = 0
        angle_4 = 0
        angle_5 = 0
        forward_kinematics(angle_1,angle_2,angle_3,angle_4,angle_5,link_1,link_2,link_3,link_4,link_5,angle_flag,link_flag)
        
    elif(choice == 0):
        angle_1,angle_2,angle_3,angle_4,angle_5 = enter_angles() 
        link_1,link_2,link_3,link_4,link_5 = enter_link_lenghts()
        angle_flag = 1
        link_flag = 1
        forward_kinematics(angle_1,angle_2,angle_3,angle_4,angle_5,link_1,link_2,link_3,link_4,link_5,angle_flag,link_flag)

    elif(choice == 9):
        angle_1 = 0
        angle_2 = 0
        angle_3 = 0
        angle_4 = 0
        angle_5 = 0

        link_1 = 0
        link_2 = 0
        link_3 = 0
        link_4 = 0
        link_5 = 0

        forward_kinematics(angle_1,angle_2,angle_3,angle_4,angle_5,link_1,link_2,link_3,link_4,link_5,angle_flag,link_flag)

    else:
        print("Invalid choice!!!")
        print("\n")

    #forward_kinematics(angle_1,angle_2,angle_3,angle_4,angle_5,link_1,link_2,link_3,link_4,link_5,angle_flag,link_flag)


