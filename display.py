# BCIT-HEXAPOD ROBOTICS CLUB 
# Author(s): Hassan Islam 
# Date : 2024-04-24
# DESC : Animates position of end affector

from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
import Coordinate2JointAngle as c2a

'''EXAMPLE IMPLEMENTAION

angles1 = leg.setPosition([1.2071,  1.2071,  1.2071], 6) 
angles2 = leg.setPosition([0,  1,  2], 3) 
angles3 = leg.setPosition([0,  -1,  -2], 7) 

angles = np.vstack((angles1,angles2,angles3))

d.display(angles)

'''

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
xdata, ydata, zdata = [], [], []
ln, = ax.plot([], [], [], lw=2)

def init():
    ln.set_data([], [])
    ln.set_3d_properties([])
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.set_zlim(-3, 3)
    return ln,

def update(frame, points):
    x = points[frame][0]
    y = points[frame][2]
    z = points[frame][1]
    ln.set_data([0, x], [0, y])  # Set new data from origin to current point
    ln.set_3d_properties([0, z])  # Set new data from origin to current point
    return ln,

def display(angles):

    points = []
    for angle in angles:
        points.append(c2a.func(angle*np.pi/180,[1,1,1],[0,0,0]).T) 

    # i couldnt figure out how to pause the animation so im just holding it at the last frame 
    for i in range(100):
        points.append(points[-1])

    ani = animation.FuncAnimation(fig, update, frames=len(points), fargs=(points,), init_func=init, blit=False)
    plt.show()
