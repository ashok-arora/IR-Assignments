import cv2
import numpy as np
import matplotlib.pyplot as plt

def image_to_coord(filename):
    image = cv2.imread(filename+'.png')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Set threshold level
    threshold_level = 50

    # Find coordinates of all pixels below threshold
    coords = np.column_stack(np.where(gray < threshold_level))

    for c in coords:
        # c = float(c)
        c[0], c[1] = c[1], c[0]

    print(np.shape(coords))
    print(coords[0][0], coords[0][1])
    print(coords[:10])
    print(type(coords[0][0]))
    np.savetxt(filename+'_xy-coord.csv', coords, delimiter=",", fmt='%i')

    plt.plot(*zip(*coords), 'ko')

    ax = plt.gca()
    ax.set(xlim=(-50, 300), ylim=(0, 300))
    ax.set_ylim(ax.get_ylim()[::-1])

    plt.show()
image_to_coord('./iiitm')