from __future__ import print_function
import numpy 
import math
import Rotation
import NBPlot
import time
import Controller
import random

rotation = Rotation.Rotation()

# Constants
Noise = 0.03
m = 0.1 #kg
g = 9.81 #m/s^2
k = 3e-6 # Some gain factor for motors? 
kd = 0.25
L = 0.14 #m
b = 1e-7 #is our drag coefficient

I_xx = 5e-3
I_yy = 5e-3
I_zz = 10e-3
I = numpy.array([[I_xx,    0,    0],
                 [   0, I_yy,    0],
                 [   0,    0, I_zz]])


# Orientation:
#        0
#        |
#    1 - o - 3
#        |
#        2
#

def controlInput(state, thetadot):
    Kd = 6.0
    Kp = 4.0
    controller = Controller.Controller()
    return controller.pd_controller(state, thetadot, Kd, Kp)

def controlPosition(x, setpoint):
    Kp = 6.0
    controller = Controller.PositionController()
    return controller.p_controller(x, setpoint, Kp)

        
def getThetaDot2OmegaConvMatrix(theta):
    return numpy.array([[ 1,                   0,                  - math.sin(theta[1])],
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
    R = rotation.rotate(angles)
    T = R.dot(thrust(inputs, k))
    Fd = -kd * xdot
    a = gravity + 1 / m * T + Fd
    return a

def angular_acceleration(inputs, omega, Inertia, L, b, k):
    tau = torques(inputs, L, b, k)
    temp =  numpy.cross(omega,Inertia.dot(omega), axisa=0, axisb=0, axisc=0)
    omegaddot = numpy.linalg.inv(Inertia).dot(tau - temp)
    return omegaddot

def thetadot2omega(thetadot, theta):    
    return getThetaDot2OmegaConvMatrix(theta).dot(thetadot)

def omega2thetadot(omega, theta):
    return numpy.linalg.inv(getThetaDot2OmegaConvMatrix(theta)).dot(omega)


def main():
    
    dt = 0.01   # period between attitude controller calls

    # x,y setpoint
    setpoint = numpy.array([[0], [0], [0]])
    
    
    # Initial simulation state.
    x = numpy.zeros((3, 1))
    xdot = numpy.zeros((3, 1))
    theta = numpy.zeros((3, 1))
    a = numpy.zeros((3, 1))
    a_pos_controller = numpy.zeros((3, 1))
    
    # Simulate some disturbance in the angular velocity.
    # The magnitude of the deviation is in radians / second.
    deviation = 100
    thetadot = numpy.radians(2 * deviation * numpy.random.rand(3, 1) - deviation)
    #thetadot = numpy.zeros((3,1))
    
    controller_params = {'dt': dt, 'I' : I, 'k': k, 'L': L, 'b' : b, 'm' : m, 'g' : g}
    
    
    data = {}   #data to send to plotting process
    pl = NBPlot.NBPlot()
    data["x"]= x
    data["a"]= a
    data["theta"] = theta
    data["t"] = 0
    data["error_controller"]= numpy.zeros((3,1))
    pl.plot(data)
    time.sleep(2)   #wait till plot is shown
    
    # Step through the simulation, updating the state.
    start_time = time.time()  #s
    prev_time = start_time
    t = 0
    end_time = 15.0   #s
    timeouts = 0
    total_passes = 0
    while t <= end_time:
        # Take input from our Controller.
        [control, controller_params, error_controller] = controlInput(controller_params, thetadot)
        print("error_controller: ", error_controller)
#         print("control: ", control)
        
        noise = numpy.random.rand(3,1) * Noise-Noise/2.0
        print("noise: ", noise)
        omega = thetadot2omega(thetadot, theta) + noise
#        print("omega: ", omega)
         
        # Compute linear and angular accelerations.
        a = acceleration(control, theta, xdot, m, g, k, kd)
        print("a: ", a)
        omegadot = angular_acceleration(control, omega, I, L, b, k)
         
        omega = omega + dt * omegadot
        thetadot = omega2thetadot(omega, theta) 
        theta = theta + dt * thetadot
        
        a += a_pos_controller
        xdot = xdot + dt * a
        x = x + dt * xdot
        
#        xpos, ypos, zpos = x[0][0], x[1][0], x[2][0]
        
#         print("a: ", a)
#        print("theta: ", theta)

        if total_passes%20 == 0:
            print("output: ", controlPosition(x, setpoint))
            a_pos_controller_temp = controlPosition(x, setpoint)
            a_pos_controller = rotation.inv_rotate(theta).dot(a_pos_controller_temp)
            print ("a_pos_controller: ", a_pos_controller)
    
        if total_passes%10 == 0:
            print ("plot new data")
            data["x"]= x
            data["a"]= a
            data["theta"] = theta
            data["t"] = t
            data["error_controller"]=error_controller
            pl.plot(data)
            
        time.sleep(dt)
#        print (time.time(), "Controller")
        print ("t: ", t)
        
        t=time.time() - start_time
        real_dt = time.time() - prev_time
#        print ("real_dt: ", real_dt)
        prev_time = time.time()
        
        total_passes+=1
        
        if t>1.0:
            if real_dt > dt*2.0 or real_dt < dt*0.9:
                timeouts+=1
    
    print('timeouts: ', timeouts, "/", total_passes, " = ", timeouts/total_passes)
    #wait till plotting has finished
    pl.plot_process.join()
    pl.plot(None, finished=True)
    

if __name__ == '__main__':
    main()
    
# --------------------------------------------------------------------------------------------    
    
    
 
 #   ax_xzplane.plot([xpos, xpos + a[0]],[zpos, zpos+a[2]], "r")
 #   ax_xzplane.plot([xpos, xpos + zBody_current[0]],[zpos, zpos+zBody_current[2]],"b")
    #ax_xzplane.plot([xpos, xpos + xBody_current[0]],[zpos, zpos+xBody_current[2]], "g")    
    
    
#    ax_xyplane.plot([xpos, xpos + xBody_current[0]],[ypos, ypos+xBody_current[1]])
#    ax_xyplane.plot([xpos, xpos + yBody_current[0]],[ypos, ypos+yBody_current[1]])  
    
# def rotate(theta):
#     R_x = numpy.array([[               1,               0,                0],
#                        [               0, math.cos(theta[0]), -math.sin(theta[0])],
#                        [               0, math.sin(theta[0]),  math.cos(theta[0])]])
#     
#     R_y = numpy.array([[ math.cos(theta[1]),               0,  math.sin(theta[1])],
#                        [               0,               1,               0],
#                        [-math.sin(theta[1]),               0,  math.cos(theta[1])]])
#     
#     R_z = numpy.array([[ math.cos(theta[2]), -math.sin(theta[2]),               0],
#                        [ math.sin(theta[2]),  math.cos(theta[2]),               0],
#                        [               0,               0,                1]])
#     
#     R = R_x.dot(R_y).dot(R_z)
#     print ("R: ", R)
#     return R

    #a1 = Arrow3D([xpos, xpos + xBody[0]],[ypos, ypos+xBody[1]], [zpos, zpos+xBody[2]], mutation_scale=20, lw=3, arrowstyle="-|>", color="r")
    #a2 = Arrow3D([xpos, xpos + yBody[0]],[ypos, ypos+yBody[1]], [zpos, zpos+yBody[2]], mutation_scale=20, lw=3, arrowstyle="-|>", color="b")
    #ax.add_artist(a1)
    #ax.add_artist(a2)
    
#    x_data.append(xpos)
#    y_data.append(ypos) 
    #ax_xyplane.plot(xpos, ypos, "go")    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

