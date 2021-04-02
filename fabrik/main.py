import fabrik
import numpy as np
from matplotlib import pyplot as plt 

if __name__ == "__main__":
    print("\nThis program can calculate position for n-link manipulator.\n")
    link = int(input("Enter number of links in the robot: "))
    inital_position = np.zeros(shape=(link+1, 2))
    for i in range(link+1):
        inital_position[i][0], inital_position[i][1] = list(map(float, input("Enter coordinate {}: ".format(i + 1)).split(',')))

    goal = list(map(float, input("Enter coordinates of goal: ").split(',')))

    plt.plot(*zip(*inital_position), 'b--', label="Initial Position")
    plt.plot(goal[0], goal[1], 'r*', label="Goal Point")

    iteration, final_position = fabrik.solve(inital_position, goal, tol=0.01)
    plt.plot(*zip(*final_position), 'g-', label="Final Position")
    plt.title("Number of iterations to reach goal state: {}".format(iteration))
    plt.grid(True)
    plt.legend()

    print("\nInitial position: \n", inital_position)
    print("\nFinal position: \n", final_position)
    print("\nGoal point:", goal)
    print("\nNumber of iterations to reach goal state: ", iteration, "\n")
    plt.show()

