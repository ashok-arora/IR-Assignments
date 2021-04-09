import math
import numpy as np
from math import cos, sin, atan2, degrees, radians


def input_fk():
    dh_table = np.zeros((6, 4))
    for i in range(6):
        parameters = np.array(list(map(int, input(
            "Enter parameters (a, alpha, d, theta) of link " + str(i) + ": ").strip().split(','))))
        dh_table[i] = parameters
    return dh_table


def solve_fk(dh_table):

    def T(dh_table, i):
        a_i1 = dh_table[i][0]
        alpha_i1 = radians(dh_table[i][1])
        d_i = dh_table[i][2]
        theta_i = radians(dh_table[i][3])

        return np.array([[cos(theta_i), -sin(theta_i), 0, a_i1],
                         [cos(alpha_i1)*sin(theta_i), cos(alpha_i1) *
                          cos(theta_i), -sin(alpha_i1), -d_i*sin(alpha_i1)],
                         [sin(alpha_i1)*sin(theta_i), sin(alpha_i1) *
                          cos(theta_i), cos(alpha_i1), d_i*cos(alpha_i1)],
                         [0, 0, 0, 1]])

    T_final = T(dh_table, 0)
    for i in range(1, dh_table.shape[0]):
        T_final = T_final @ T(dh_table, i)
    end_effector_position = T_final @ np.array([0, 0, 0, 1])

    print('\nEnd effector position: ({:.3f}, {:.3f}, {:.3f})'.format(
        end_effector_position[0], end_effector_position[1], end_effector_position[2]))


def input_ik():
    inp = list(
        map(int, input("Enter robot features (a2, a3, d3, d4): ").strip().split(',')))
    T = np.zeros((4, 4))
    print('Enter 4x4 Transformation Matrix: ')
    for i in range(4):
        T[i] = np.array(list(map(float, input().split(','))))

    return (T, inp[0], inp[1], inp[2], inp[3])


def solve_ik(T, a2, a3, d3, d4):
    px = T[0, 3]
    py = T[1, 3]
    pz = T[2, 3]

    # calculating theta1
    theta1 = []
    theta1.append(degrees(atan2(py, px) -
                          atan2(d3, (px**2+py**2-d3**2)**0.5)))
    theta1.append(degrees(atan2(py, px) -
                          atan2(d3, -((px**2+py**2-d3**2)**0.5))))

    K = (px**2+py**2+pz**2-a2**2-a3**2-d3**2-d4**2)/(2*a2)

    # calculating theta3
    theta3 = []
    theta3.append(degrees(atan2(a3, d4) -
                          atan2(K, (a3**2+d4**2-K**2)**0.5)))
    theta3.append(degrees(atan2(a3, d4) -
                          atan2(K, -((a3**2+d4**2-K**2)**0.5))))

    # calculating theta2
    theta2 = []

    s1 = sin(radians(theta1[0]))
    c1 = cos(radians(theta1[0]))
    s3 = sin(radians(theta3[0]))
    c3 = cos(radians(theta3[0]))
    theta23 = degrees(atan2(
        ((-a3-(a2*c3))*pz)-(((c1*px)+(s1*py))*(d4-(a2*s3))), (((a2*s3)-d4)*pz)+((a3+(a2*c3))*((c1*px)+(s1*py)))))
    theta2.append(theta23-theta3[0])
    s1 = sin(radians(theta1[1]))
    c1 = cos(radians(theta1[1]))
    s3 = sin(radians(theta3[0]))
    c3 = cos(radians(theta3[0]))
    theta23 = degrees(atan2(
        ((-a3-a2*c3)*pz)-((c1*px+s1*py)*(d4-a2*s3)), ((a2*s3-d4)*pz)-((a3+a2*c3)*(c1*px+s1*py))))
    theta2.append(theta23-theta3[0])
    s1 = sin(radians(theta1[0]))
    c1 = cos(radians(theta1[0]))
    s3 = sin(radians(theta3[1]))
    c3 = cos(radians(theta3[1]))
    theta23 = degrees(atan2(
        ((-a3-a2*c3)*pz)-((c1*px+s1*py)*(d4-a2*s3)), ((a2*s3-d4)*pz)-((a3+a2*c3)*(c1*px+s1*py))))
    theta2.append(theta23-theta3[1])
    s1 = sin(radians(theta1[1]))
    c1 = cos(radians(theta1[1]))
    s3 = sin(radians(theta3[1]))
    c3 = cos(radians(theta3[1]))
    theta23 = degrees(atan2(
        ((-a3-a2*c3)*pz)-((c1*px+s1*py)*(d4-a2*s3)), ((a2*s3-d4)*pz)-((a3+a2*c3)*(c1*px+s1*py))))
    theta2.append(theta23-theta3[1])

    r13 = T[0, 2]
    r23 = T[1, 2]
    r33 = T[2, 2]

    # calculating theta4
    theta4 = []

    s1 = sin(radians(theta1[0]))
    c1 = cos(radians(theta1[0]))
    s23 = sin(radians(theta2[0]+theta3[0]))
    c23 = cos(radians(theta2[0]+theta3[0]))
    theta4.append(degrees(atan2((-r13*s1)+(r23*c1),
                                (-r13*c1*c23)-(r23*s1*c23)+(r33*s23))))
    s1 = sin(radians(theta1[1]))
    c1 = cos(radians(theta1[1]))
    s23 = sin(radians(theta2[1]+theta3[0]))
    c23 = cos(radians(theta2[1]+theta3[0]))
    theta4.append(degrees(atan2((-r13*s1)+(r23*c1),
                                (-r13*c1*c23)-(r23*s1*c23)+(r33*s23))))
    s1 = sin(radians(theta1[0]))
    c1 = cos(radians(theta1[0]))
    s23 = sin(radians(theta2[2]+theta3[1]))
    c23 = cos(radians(theta2[2]+theta3[1]))
    theta4.append(degrees(atan2((-r13*s1)+(r23*c1),
                                (-r13*c1*c23)-(r23*s1*c23)+(r33*s23))))
    s1 = sin(radians(theta1[1]))
    c1 = cos(radians(theta1[1]))
    s23 = sin(radians(theta2[3]+theta3[1]))
    c23 = cos(radians(theta2[3]+theta3[1]))
    theta4.append(degrees(atan2((-r13*s1)+(r23*c1),
                                (-r13*c1*c23)-(r23*s1*c23)+(r33*s23))))

    # calculating theta5
    theta5 = []

    s1 = sin(radians(theta1[0]))
    c1 = cos(radians(theta1[0]))
    s23 = sin(radians(theta2[0]+theta3[0]))
    c23 = cos(radians(theta2[0]+theta3[0]))
    s4 = sin(radians(theta4[0]))
    c4 = cos(radians(theta4[0]))
    theta5.append(degrees(atan2((-r13*(c1*c23*c4+s1*s4)) -
                                (r23*(s1*c23*c4))+(r33*s23*c4), (-r13*c1*s23)-(r23*s1*s23)-(r33*c23))))

    s1 = sin(radians(theta1[1]))
    c1 = cos(radians(theta1[1]))
    s23 = sin(radians(theta2[1]+theta3[0]))
    c23 = cos(radians(theta2[1]+theta3[0]))
    s4 = sin(radians(theta4[1]))
    c4 = cos(radians(theta4[1]))
    theta5.append(degrees(atan2((-r13*(c1*c23*c4+s1*s4)) -
                                (r23*(s1*c23*c4))+(r33*s23*c4), (-r13*c1*s23)-(r23*s1*s23)-(r33*c23))))

    s1 = sin(radians(theta1[0]))
    c1 = cos(radians(theta1[0]))
    s23 = sin(radians(theta2[2]+theta3[1]))
    c23 = cos(radians(theta2[2]+theta3[1]))
    s4 = sin(radians(theta4[2]))
    c4 = cos(radians(theta4[2]))
    theta5.append(degrees(atan2((-r13*(c1*c23*c4+s1*s4)) -
                                (r23*(s1*c23*c4))+(r33*s23*c4), (-r13*c1*s23)-(r23*s1*s23)-(r33*c23))))

    s1 = sin(radians(theta1[1]))
    c1 = cos(radians(theta1[1]))
    s23 = sin(radians(theta2[3]+theta3[1]))
    c23 = cos(radians(theta2[3]+theta3[1]))
    s4 = sin(radians(theta4[3]))
    c4 = cos(radians(theta4[3]))
    theta5.append(degrees(atan2((-r13*(c1*c23*c4+s1*s4)) -
                                (r23*(s1*c23*c4))+(r33*s23*c4), (-r13*c1*s23)-(r23*s1*s23)-(r33*c23))))

    r11 = T[0, 0]
    r21 = T[1, 0]
    r31 = T[2, 0]

    # calculating theta6
    theta6 = []

    s1 = sin(radians(theta1[0]))
    c1 = cos(radians(theta1[0]))
    s23 = sin(radians(theta2[0]+theta3[0]))
    c23 = cos(radians(theta2[0]+theta3[0]))
    s4 = sin(radians(theta4[0]))
    c4 = cos(radians(theta4[0]))
    s5 = sin(radians(theta5[0]))
    c5 = cos(radians(theta5[0]))
    theta6.append(degrees(atan2(
        (-r11*(c1*c23*s4-s1*c4)-(r21*(s1*c23*s4+c1*c4))+r31*s23*s4), (r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5))+(r21*((s1*c23*c4-c1*s4)*c5-s1*s23*s5))-(r31*(s23*c4*c5+c23*s5)))))
    s1 = sin(radians(theta1[1]))
    c1 = cos(radians(theta1[1]))
    s23 = sin(radians(theta2[1]+theta3[0]))
    c23 = cos(radians(theta2[1]+theta3[0]))
    s4 = sin(radians(theta4[1]))
    c4 = cos(radians(theta4[1]))
    s5 = sin(radians(theta5[1]))
    c5 = cos(radians(theta5[1]))
    theta6.append(degrees(atan2(
        (-r11*(c1*c23*s4-s1*c4)-(r21*(s1*c23*s4+c1*c4))+r31*s23*s4), (r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5))+(r21*((s1*c23*c4-c1*s4)*c5-s1*s23*s5))-(r31*(s23*c4*c5+c23*s5)))))
    s1 = sin(radians(theta1[0]))
    c1 = cos(radians(theta1[0]))
    s23 = sin(radians(theta2[2]+theta3[1]))
    c23 = cos(radians(theta2[2]+theta3[1]))
    s4 = sin(radians(theta4[2]))
    c4 = cos(radians(theta4[2]))
    s5 = sin(radians(theta5[2]))
    c5 = cos(radians(theta5[2]))
    theta6.append(degrees(atan2(
        (-r11*(c1*c23*s4-s1*c4)-(r21*(s1*c23*s4+c1*c4))+r31*s23*s4), (r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5))+(r21*((s1*c23*c4-c1*s4)*c5-s1*s23*s5))-(r31*(s23*c4*c5+c23*s5)))))
    s1 = sin(radians(theta1[1]))
    c1 = cos(radians(theta1[1]))
    s23 = sin(radians(theta2[3]+theta3[1]))
    c23 = cos(radians(theta2[3]+theta3[1]))
    s4 = sin(radians(theta4[3]))
    c4 = cos(radians(theta4[3]))
    s5 = sin(radians(theta5[3]))
    c5 = cos(radians(theta5[3]))
    theta6.append(degrees(atan2(
        (-r11*(c1*c23*s4-s1*c4)-(r21*(s1*c23*s4+c1*c4))+r31*s23*s4), (r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5))+(r21*((s1*c23*c4-c1*s4)*c5-s1*s23*s5))-(r31*(s23*c4*c5+c23*s5)))))

    # creating a list of joint angles calculated above
    joint_angles = []

    joint_angles.append(
        (theta1[0], theta2[0], theta3[0], theta4[0], theta5[0], theta6[0]))
    joint_angles.append(
        (theta1[1], theta2[1], theta3[0], theta4[1], theta5[1], theta6[1]))
    joint_angles.append(
        (theta1[0], theta2[2], theta3[1], theta4[2], theta5[2], theta6[2]))
    joint_angles.append(
        (theta1[1], theta2[3], theta3[1], theta4[3], theta5[3], theta6[3]))
    joint_angles.append(
        (theta1[0], theta2[0], theta3[0], theta4[0]+180, -theta5[0], theta6[0]+180))
    joint_angles.append(
        (theta1[1], theta2[1], theta3[0], theta4[1]+180, -theta5[1], theta6[1]+180))
    joint_angles.append(
        (theta1[0], theta2[2], theta3[1], theta4[2]+180, -theta5[2], theta6[2]+180))
    joint_angles.append(
        (theta1[1], theta2[3], theta3[1], theta4[3]+180, -theta5[3], theta6[3]+180))

    print("\nPossible joint angles are: ")
    for i in range(8):
        print('theta1: {:.3f}, theta2: {:.3f}, theta3: {:.3f}, theta4: {:.3f}, theta5: {:.3f}, theta6: {:.3f}'.format(
            joint_angles[i][0], joint_angles[i][1], joint_angles[i][2], joint_angles[i][3], joint_angles[i][4], joint_angles[i][5]))
        print()


def main():

    print("Program to calculate forward and inverse kinematics of PUMA 560 robot.\n")
    choice = input("Enter 1 for forward kinematics or 2 for inverse kinematics. Enter your choice: ")

    if choice == '1': 
        dh_table = input_fk()
        solve_fk(dh_table)

    elif choice == '2':
        T, a2, a3, d3, d4 = input_ik()
        solve_ik(T, a2, a3, d3, d4)

    else:
        print("Incorrect choice, exiting.")

if __name__ == '__main__':
    main()
