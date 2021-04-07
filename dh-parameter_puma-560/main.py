import math
import numpy as np
from math import cos
from math import sin

def solve(dh_table):

    def T(i, j):
        theta_i = math.radians(dh_table[i][1])
        a_j = dh_table[j][2]
        alpha_j = math.radians(dh_table[j][3])
        d_i = dh_table[i][0]
        return np.array([[cos(theta_i), -sin(theta_i), 0, a_j],
                        [cos(alpha_j) * sin(theta_i), cos(alpha_j) * cos(theta_i), -sin(alpha_j), -d_i * sin(alpha_j)],
                        [sin(alpha_j) * sin(theta_i), sin(alpha_j) * cos(theta_i), cos(alpha_j), d_i * cos(alpha_j)],
                        [0, 0, 0, 1]])

    def final_transformation_matrix():
        mat = T(1, 0)
        for i in range(2, 6):
            mat = np.matmul(mat, T(i, i - 1))
        return mat

    point_coords = np.array([0, 0, 0, 1])
    final = np.round(np.matmul(final_transformation_matrix(), point_coords), 3) + 0
    print("\nPosition of end effector is:", final)

def main():
    dh_table = np.zeros((6, 4))
    for i in range(6):
        parameters = np.array(list(map(int, input("Enter parameters of link " + str(i) + ": ").strip().split(','))))
        dh_table[i] = parameters
    solve(dh_table)

if __name__ == '__main__':
    main()