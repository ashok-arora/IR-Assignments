import numpy as np
import math

def Input():
    print('\nThis program can be used to calculate the spatial position with respect to frame {A/B} \nwhen the postion in frame {B/A} and translations and/or rotations are known.')

    xRotation = int(input('\nEnter rotation in X axis (in degrees): '))
    xRotation = math.pi/180*xRotation
    yRotation = int(input('Enter rotation in Y axis (in degrees): '))
    yRotation = math.pi/180*yRotation
    zRotation = int(input('Enter rotation in Z axis (in degrees): '))
    zRotation = math.pi/180*zRotation

    xTranslation = int(input('Enter translation along X axis: '))
    yTranslation = int(input('Enter translation along Y axis: '))
    zTranslation = int(input('Enter translation along Z axis: '))

    frameKnown = ''
    while frameKnown != 'a' and frameKnown != 'b':
        frameKnown = input('Point is known in frame? (A/B): ').lower()
        if frameKnown != 'a' and frameKnown != 'b':
            print('Two frames are \'A\' or \'B\'')

    initialCoordinates = np.array(list(
        map(int, input('\nEnter coordinates of the point in the frame: ').split())))

    return initialCoordinates, transformationMatrix(xRotation, yRotation, zRotation, xTranslation, yTranslation, zTranslation, frameKnown)


def transformationMatrix(xRotation, yRotation, zRotation, xTranslation, yTranslation, zTranslation, frameKnown):
    rx = np.array([[1, 0, 0],
                   [0, math.cos(xRotation), -math.sin(xRotation)],
                   [0, math.sin(xRotation), math.cos(xRotation)]])

    ry = np.array([[math.cos(yRotation), 0, math.sin(yRotation)],
                   [0, 1, 0],
                   [-math.sin(yRotation), 0, math.cos(yRotation)]])

    rz = np.array([[math.cos(zRotation), -math.sin(zRotation), 0],
                   [math.sin(zRotation), math.cos(zRotation), 0],
                   [0, 0, 1]])

    r = rz @ ry @ rx

    # if point is known is frame 'A' we need inverse transformation matrix
    if frameKnown == 'a':
        # displacement matrix representing displacement about 3 axes
        d = np.array([[xTranslation], [yTranslation], [zTranslation]])
        temp = -r.T@d
        temp = np.vstack((temp, [1]))

        transformationMatrix = np.hstack(
            (np.vstack((r.T, np.array([0, 0, 0]))), temp))

        return transformationMatrix

    elif frameKnown == 'b':
        # displacement matrix representing displacement about 3 axes
        d = np.array([[xTranslation], [yTranslation], [zTranslation], [1]])

        transformationMatrix = np.hstack(
            (np.vstack((r, np.array([0, 0, 0]))), d))

        return transformationMatrix


def transform(initialCoordinates, transformationMatrix):
    # 1 is appending to the coordinates for matrix multiplication
    initialCoordinates = np.hstack((initialCoordinates, 1))
    finalCoordinates = transformationMatrix@initialCoordinates

    # removing '1' in the last dimension
    return finalCoordinates[:3]


def Print(finalCoordinates):
    print('\nPosition of the point in other frame is: ({}, {}, {})'.format(round(finalCoordinates[0], 3), round(finalCoordinates[1], 3), round(finalCoordinates[2], 3)))


if __name__ == '__main__':
    initialCoordinates, transformationMatrix = Input()
    finalCoordinates = transform(initialCoordinates, transformationMatrix)
    Print(finalCoordinates)
