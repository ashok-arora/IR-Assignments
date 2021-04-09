<h1 align="center">Introduction to Robotics</h1>

<h2 align="center">BCCS-9402</h2>

__Course Instructor__: Dr. Vijay Bhaskar Semwal 

__Submitted by__: Ashok Arora (2019BCS-075)

__Problem Statement__: Write the code to calculate forward kinematics for PUMA 560 robot using DH table.

__Sample input/output (forward kinematics)__:
```
Program to calculate forward and inverse kinematics of PUMA 560 robot.

Enter 1 for forward kinematics or 2 for inverse kinematics. Enter your choice: 1
Enter parameters (a, alpha, d, theta) of link 0: 0, 0, 0, 10
Enter parameters (a, alpha, d, theta) of link 1: 0, -90, 0, 15
Enter parameters (a, alpha, d, theta) of link 2: 3, 0, 3, 20
Enter parameters (a, alpha, d, theta) of link 3: 3, -90, 3, 25
Enter parameters (a, alpha, d, theta) of link 4: 0, 90, 0, 30
Enter parameters (a, alpha, d, theta) of link 5: 0, -90, 0, 35

End effector position: (3.058, 3.586, -4.955)
```

__Sample input/output (inverse kinematics)__:
```
Program to calculate forward and inverse kinematics of PUMA 560 robot.

Enter 1 for forward kinematics or 2 for inverse kinematics. Enter your choice: 2
Enter robot features (a2, a3, d3, d4): 3, 3, 3, 3
Enter 4x4 Transformation Matrix: 
0.23409161, -0.38798654, -0.89144129,  3.05834348
-0.7910126, -0.60910276,  0.05738376,  3.58554831
-0.56524347, 0.69170823, -0.44948808, -4.95464258
0,           0,           0,           1

Possible joint angles are: 
theta1: 10.000, theta2: 15.000, theta3: 20.000, theta4: 25.000, theta5: 28.801, theta6: 35.006

theta1: -90.926, theta2: 29.337, theta3: 20.000, theta4: -109.328, theta5: 70.989, theta6: 104.001

theta1: 10.000, theta2: 82.302, theta3: -110.000, theta4: 12.199, theta5: 90.316, theta6: 57.058

theta1: -90.926, theta2: 150.663, theta3: -110.000, theta4: -106.264, theta5: 68.330, theta6: 95.194

theta1: 10.000, theta2: 15.000, theta3: 20.000, theta4: 205.000, theta5: -28.801, theta6: 215.006

theta1: -90.926, theta2: 29.337, theta3: 20.000, theta4: 70.672, theta5: -70.989, theta6: 284.001

theta1: 10.000, theta2: 82.302, theta3: -110.000, theta4: 192.199, theta5: -90.316, theta6: 237.058

theta1: -90.926, theta2: 150.663, theta3: -110.000, theta4: 73.736, theta5: -68.330, theta6: 275.194
```

### Note: 

The sample input is stored in `input_fk.txt` and `input_ik.txt` files for forward and inverse kinematics respectively.

A way to run the program without manually supplying the inputs every time:

```
/usr/bin/python3.9 /path/to/main.py < input_fk.txt
```

or 

```
/usr/bin/python3.9 /path/to/main.py < input_ik.txt
```


