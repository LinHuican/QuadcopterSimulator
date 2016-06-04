import numpy 
import math
import random
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
import matplotlib.pyplot as plt


def getConvMatrix(theta):
    return numpy.array([[ 1,                0,               - math.sin(theta[1])],
                         [ 0,  math.cos(theta[0]), math.cos(theta[1])*math.sin(theta[0])],
                         [ 0, -math.sin(theta[0]), math.cos(theta[1])*math.cos(theta[0])]])

# Compute thrust given current inputs and thrust coefficient.
def thrust( inputs, k):
    # Inputs are values for ${\omega_i}^2$
    T = numpy.array([[0], [0],[ k * sum(inputs)]])
    return T

# Compute torques, given current inputs, length, drag coefficient, and thrust coefficient.
def torques( inputs, L, b, k):
    # Inputs are values for ${\omega_i}^2$
    tau = numpy.array([[ L * k * (inputs[0] - inputs[2])],
                       [L * k * (inputs[1] - inputs[3])],
                       [b * (inputs[0] - inputs[1] + inputs[2] - inputs[3])]])
    
    return tau

def acceleration( inputs, angles, xdot, m, g, k, kd):
    gravity = [[0], [0], [-g]]
    R = rotation(angles)
    T = R.dot(thrust(inputs, k))
#     print ("R: ", R)
#     print ("thrust:", thrust(inputs, k))
    print ("T: ", T)
    Fd = -kd * xdot
    a = gravity + 1 / m * T + Fd
    return a

def angular_acceleration(inputs, omega, Inertia, L, b, k):
    tau = torques(inputs, L, b, k)
#    temp = numpy.cross(numpy.matrix.transpose(omega),numpy.matrix.transpose(Inertia.dot(omega)))
    temp =  numpy.cross(omega,Inertia.dot(omega), axisa=0, axisb=0, axisc=0)
    omegaddot = numpy.linalg.inv(Inertia).dot(tau - temp)
    return omegaddot


def rotation(theta):
    R_x = numpy.array([[               1,               0,                0],
                       [               0, math.cos(theta[0]), -math.sin(theta[0])],
                       [               0, math.sin(theta[0]),  math.cos(theta[0])]])
    
    R_y = numpy.array([[ math.cos(theta[1]),               0,  math.sin(theta[1])],
                       [               0,               1,               0],
                       [-math.sin(theta[1]),               0,  math.cos(theta[1])]])
    
    R_z = numpy.array([[ math.cos(theta[2]), -math.sin(theta[2]),               0],
                       [ math.sin(theta[2]),  math.cos(theta[2]),               0],
                       [               0,               0,                1]])
    
    R = R_x.dot(R_y).dot(R_z)
    print ("R: ", R)
    return R
    

def thetadot2omega(thetadot, theta):    
    return getConvMatrix(theta).dot(thetadot)

def omega2thetadot(omega, theta):
    return numpy.linalg.inv(getConvMatrix(theta)).dot(omega)


def input(t):
    # this omega
    return t*numpy.array([1.0, 1.0, 1.0, 1.0])


plt.ion()
# Simulation times, in seconds.
start_time = 0  #s
end_time = 10   #s
dt = 0.005      
times = numpy.arange(start_time, end_time, dt)

m = 0.1 #kg
g = 9.81 #m/s^2
k = 1.0 # Some gain factor for motors? 
kd = 1.0

L = 14 #cm
b = 0.5 #is our drag coefficient

I_xx = 1.1
I_yy = 1.2
I_zz = 1.3
I = numpy.array([[I_xx,    0,    0],
                 [   0, I_yy,    0],
                 [   0,    0, I_zz]])

# Number of points in the simulation.
N = len(times)

# Initial simulation state.
x = numpy.array([[0], [0], [1]])
xdot = numpy.zeros((3, 1))
theta = numpy.zeros((3, 1))

# Simulate some disturbance in the angular velocity.
# The magnitude of the deviation is in radians / second.
deviation = 100
thetadot = numpy.radians(2 * deviation * numpy.random.rand(3, 1) - deviation)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


# Step through the simulation, updating the state.
for t in times:
    # Take input from our controller.
    i = input(t);
    
    theta = theta + numpy.ones((3, 1))/100.0

    omega = thetadot2omega(thetadot, theta)
    
    # Compute linear and angular accelerations.
    a = acceleration(i, theta, xdot, m, g, k, kd)
    omegadot = angular_acceleration(i, omega, I, L, b, k)
    
    omega = omega + dt * omegadot
    thetadot = omega2thetadot(omega, theta) 
    theta = theta + dt * thetadot
    xdot = xdot + dt * a
    x = x + dt * xdot
    
#    print("x: ", x)
#    print ("a: ", a)
    ax.cla()
    ax.set_ylim([-5,5])
    ax.set_xlim([-5,5])
    ax.set_zlim([ 0,5])
    xpos, ypos, zpos = x[0][0], x[1][0], x[2][0]
    ax.plot( [xpos, xpos + a[0]], [ypos, ypos+a[1]] , [zpos, zpos+a[2]],"g-",linewidth=2)
    plt.draw()
    

