#####
#
#   Plots the result of performing a rotation on a 3-dimensional frame
#   Traces out the path that the rotation took from the space frame to the rotated frame
#
#####


import quaternion
import numpy as np
import functions as f
from matplotlib import animation
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import proj3d
from matplotlib.patches import FancyArrowPatch

sin, cos = np.sin, np.cos
deg, rad = np.degrees, np.radians
q, conj = np.quaternion, np.conjugate
pi, exp, sqrt = np.pi, np.exp, np.sqrt
asin, acos, atan = np.arcsin, np.arccos, np.arctan2


# S Frame unit vectors and matrix
sFrame_x = np.array( [1, 0, 0] )
sFrame_y = np.array( [0, 1, 0] )
sFrame_z = np.array( [0, 0, 1] )
sFrameMatrix = np.matrix([  [ 1, 0, 0 ],
                            [ 0, 1, 0 ],
                            [ 0, 0, 1 ]   ])

# Vector arrow properties
W_arrow_properties = dict(mutation_scale = 20,  arrowstyle = '-|>',  lw=2,  shrinkA = 0,  shrinkB = 0)
Rsb_arrow_properties = dict(mutation_scale = 20,  arrowstyle = '-|>',  lw=2, color = 'm',  shrinkA = 0,  shrinkB = 0)
sframe_arrow_properties = dict(mutation_scale = 20,  arrowstyle = '-|>',  color = 'k',  lw=1.5,  shrinkA = 0,  shrinkB = 0)

# Class needed to enable the drawing of arrows in three dimensions
class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        super().__init__((0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs
    def do_3d_projection(self, renderer=None):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.axes.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        return np.min(zs)

# Function for saving animation as .gif (save is type bool)
def save(save):
    if save:
        f = r"c://Users/Matt/Desktop/animation.gif"
        writergif = animation.PillowWriter(fps=30)
        anim.save(f, writer=writergif)
    else:
        return

# Sets the properties of the axes
def setAxes():
    ax.set_xlim3d([-1.1, 1.1])
    ax.set_xlabel('X')
    ax.set_ylim3d([-1.1, 1.1])
    ax.set_ylabel('Y')
    ax.set_zlim3d([-1.1, 1.1])
    ax.set_zlabel('Z')
    ax.view_init(24, 41)

# Draws the space frame at the origin with unit vectors
def drawSpaceFrame():
    a = Arrow3D([0, 1], [0, 0], [0, 0], **sframe_arrow_properties, alpha=0.9, label = 's frame')
    b = Arrow3D([0, 0], [0, 1], [0, 0], **sframe_arrow_properties, alpha=0.9)
    c = Arrow3D([0, 0], [0, 0], [0, 1], **sframe_arrow_properties, alpha=0.9)
    ax.add_artist(a)
    ax.add_artist(b)
    ax.add_artist(c)

# Draws a given rotation frame with unit vectors
def drawFrame(matrix):
    a = Arrow3D([0, matrix.item(0)], [0, matrix.item(3)], [0, matrix.item(6)], **Rsb_arrow_properties, alpha=0.8, label = 'R')
    b = Arrow3D([0, matrix.item(1)], [0, matrix.item(4)], [0, matrix.item(7)], **Rsb_arrow_properties, alpha=0.8)
    c = Arrow3D([0, matrix.item(2)], [0, matrix.item(5)], [0, matrix.item(8)], **Rsb_arrow_properties, alpha=0.8)
    ax.add_artist(a)
    ax.add_artist(b)
    ax.add_artist(c)

# Generates the data array that is used to build the animation for the xHat direction
def genX(theta, R):
    Theta = rad(theta)
    Q1 = q( 0, 1, 0, 0 )
    i = 0
    while i < theta:
        Q2 = q( cos(Theta/2*i/theta), wHat[0]*sin(Theta/2*i/theta), wHat[1]*sin(Theta/2*i/theta), wHat[2]*sin(Theta/2*i/theta) )
        Q2c = conj(Q2)
        Q3 = Q2*Q1*Q2c
        yield np.array( [Q3.x, Q3.y, Q3.z] )
        i += 1

# Generates the data array that is used to build the animation for the yHat direction
def genY(theta, R):
    Theta = rad(theta)
    Q1 = q( 0, 0, 1, 0 )
    i = 0
    while i < theta:
        Q2 = q( cos(Theta/2*i/theta), wHat[0]*sin(Theta/2*i/theta), wHat[1]*sin(Theta/2*i/theta), wHat[2]*sin(Theta/2*i/theta) )
        Q2c = conj(Q2)
        Q3 = Q2*Q1*Q2c
        yield np.array( [Q3.x, Q3.y, Q3.z] )
        i += 1

# Generates the data array that is used to build the animation for the zHat direction
def genZ(theta, R):
    Theta = rad(theta)
    Q1 = q( 0, 0, 0, 1 )
    i = 0
    while i < theta:
        Q2 = q( cos(Theta/2*i/theta), wHat[0]*sin(Theta/2*i/theta), wHat[1]*sin(Theta/2*i/theta), wHat[2]*sin(Theta/2*i/theta) )
        Q2c = conj(Q2)
        Q3 = Q2*Q1*Q2c
        yield np.array( [Q3.x, Q3.y, Q3.z] )
        i += 1

# Called from the animation object, actually builds the animation through time
def update(num, data1, line1, data2, line2, data3, line3):
    line1.set_data(data1[:2, :num])
    line1.set_3d_properties(data1[2, :num])
    line2.set_data(data2[:2, :num])
    line2.set_3d_properties(data2[2, :num])
    line1.set_color("green")
    line2.set_color("red")
    line1.set_linestyle('dashed')
    line2.set_linestyle('dashed')
    line3.set_data(data3[:2, :num])
    line3.set_3d_properties(data3[2, :num])
    line3.set_color("blue")
    line3.set_linestyle('dashed')
    line1.set_alpha(0.5)
    line2.set_alpha(0.5)
    line3.set_alpha(0.5)
    # ax.view_init(elev=20., azim=num)

# Draws a given vector with a color and label, assumes the vector tail begins at the origin of the space frame
def drawVector(vector, color, label):
    unit_vector = f.unitVector(vector)
    if label == 'none':
        a = Arrow3D([0, unit_vector.item(0)], [0, unit_vector.item(1)], [0, unit_vector.item(2)], **W_arrow_properties, color = color, alpha=0.75)
        ax.add_artist(a)
    else:
        a = Arrow3D([0, unit_vector.item(0)], [0, unit_vector.item(1)], [0, unit_vector.item(2)], **W_arrow_properties, color = color, alpha=0.75, label = label)
        ax.add_artist(a)


# ------------------------------------------------------------------------------------------------------------------------------------------------- #


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
setAxes()
drawSpaceFrame()


Rsb = f.rotate2(2, ['x', 'z'], [60, 30])
wHatAndTheta = f.matrixLog(Rsb)
wHat = wHatAndTheta[0]
theta = wHatAndTheta[1]


drawFrame(Rsb)
drawVector(wHat, 'red', 'wHat')


data1 = np.array(  list( genY(theta, Rsb) )  ).T
data2 = np.array(  list( genZ(theta, Rsb) )  ).T
data3 = np.array(  list( genX(theta, Rsb) )  ).T
line1, = ax.plot(  data1[0, 0:1] ,  data1[1, 0:1] ,  zs = data1[2, 0:1]  )
line2, = ax.plot(  data2[0, 0:1] ,  data2[1, 0:1] ,  zs = data2[2, 0:1]  )
line3, = ax.plot(  data3[0, 0:1] ,  data3[1, 0:1] ,  zs = data3[2, 0:1]  )
anim = animation.FuncAnimation(fig, update, int(theta+100), fargs=(data1, line1, data2, line2, data3, line3), interval=360/theta, blit=False)


plt.axis('off')
save(False)
plt.show()

