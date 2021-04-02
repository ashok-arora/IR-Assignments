import numpy as np
import math
from operator import add, sub

def euclidean(lst1, lst2):
    lst = list(map(sub, lst2, lst1))
    lst = [i ** 2 for i in lst]
    return math.sqrt(sum(lst))

def scalarMul(scalar, list):
    lst = [scalar * i for i in list]
    return lst

def solve(inital_position, goal, tol = 0.01):
    link_lengths = []
    for i in range(inital_position.shape[0] - 1):
        link_lengths.append(euclidean(inital_position[i + 1], inital_position[i]))

    diff = euclidean(goal, inital_position[0])
    itr = 0
    if (diff > sum(link_lengths)):
        print("goal unreachable")
        for i in range(inital_position.shape[0] - 1):
            r = euclidean(goal, inital_position[i])
            lamb = link_lengths[i] / r
            inital_position[i + 1] = list(map(add, ((1 - lamb) * inital_position[i]), scalarMul(lamb, goal)))
    else:
        b = np.copy(inital_position[0])
        curr_tol = euclidean(goal, inital_position[inital_position.shape[0]-1])
        while (curr_tol > tol):
            inital_position[inital_position.shape[0]-1] = goal
            for i in range(inital_position.shape[0] - 2, -1, -1):
                r = euclidean(inital_position[i + 1], inital_position[i])
                lamb = link_lengths[i] / r
                inital_position[i] = list(
                    map(add, (1 - lamb) * inital_position[i + 1], scalarMul(lamb, inital_position[i])))
            inital_position[0] = b
            for i in range(inital_position.shape[0] - 1):
                r = euclidean(inital_position[i + 1], inital_position[i])
                lamb = link_lengths[i] / r
                inital_position[i + 1] = list(
                    map(add, (1 - lamb) * inital_position[i], scalarMul(lamb, inital_position[i + 1])))

            curr_tol = euclidean(goal, inital_position[inital_position.shape[0]-1])
            itr += 1
    return (itr, inital_position)
