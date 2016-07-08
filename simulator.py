from __future__ import print_function
import numpy 
import Rotation
import NBPlot
import time
import Controller
import math
import csv


rotation = Rotation.Rotation()

# Model Constants
Noise_omega = 0.02
Noise_position = 0.1
m = 0.1 #kg
g = 9.81 #m/s^2
k = 3e-6 # Some gain factor for motors? 
kd = 0.25
L = 0.14 #m
b = 1e-7 #is our drag coefficient

#Inertia
I_xx = 5e-3
I_yy = 5e-3
I_zz = 10e-3
I = numpy.array([[I_xx,    0,    0],
                 [   0, I_yy,    0],
                 [   0,    0, I_zz]])

# time
end_time = 30.0   # simulation runtime in s
dt = 0.01   # time span between attitude controller calls

csv_file='mycsvfile.csv'

# x,y setpoint
setpoint = numpy.array([[1.0], [0.0], [0.0]])


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
    Kp_xy = 1.3
    Kp_alt = 0.01
    Ki = 0.0
    controller_xy = Controller.PositionController()
    controller_z = Controller.AltitudeController()
    output_z = controller_z.pi_controller(x, setpoint, Kp_alt, Ki)
    return controller_xy.p_controller(x, setpoint, Kp_xy) + output_z

        
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
    gravity = numpy.array([[0], [0], [-g]])
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
    # Initial simulation state.
    x = numpy.zeros((3, 1))
    xdot = numpy.zeros((3, 1))
    theta = numpy.zeros((3, 1))
    a = numpy.zeros((3, 1))
    a_controller_body = numpy.zeros((3, 1))
    x_err = numpy.zeros((3, 1))
    total_pid_error = numpy.zeros((3, 1))
    
    # Simulate some disturbance in the angular velocity.
    # The magnitude of the deviation is in radians / second.
    deviation = 0.0
    thetadot = numpy.radians(2 * deviation * numpy.random.rand(3, 1) - deviation)
    #thetadot = numpy.zeros((3,1))
    
    controller_params = {'dt': dt, 'I' : I, 'k': k, 'L': L, 'b' : b, 'm' : m, 'g' : g}
    
    data = {}   #data to send to plotting process
    pl = NBPlot.NBPlot()
    data["x"]= x
    data["x_err"]= x_err
    data["a"]= a
    data["theta"] = theta
    data["t"] = 0
    data["error_controller"]= numpy.zeros((3,1))
    data["total_pid_error"]= total_pid_error
    pl.plot(data)
    time.sleep(2)   #wait till plot is shown
    
    # Step through the simulation, updating the state.
    start_time = time.time()  #s
    
    # some counters
    t = 0
    timeouts = 0
    total_loop_passes = 0
    
    # save values
    meassured_values_row = {'time': t, 'x' : x[0][0], 'y': x[1][0], 'z': x[2][0],\
                        'x_err':"{:.5f}".format(x_err[0][0]), 'y_err': "{:.5f}".format(x_err[1][0]),\
                        'z_err':"{:.5f}".format(x_err[2][0]), 'ax': "{:.5f}".format(a[0][0]),\
                        'ay': "{:.5f}".format(a[1][0]),'az': "{:.5f}".format(a[2][0])}
    meassured_values = []
    
    
    
    while t <= end_time:
        start_loop_time = time.time()
        print("start_loop_time: ", start_loop_time)
        # Take input from our Controller.
        thetadot_noisy = thetadot + numpy.random.rand(3,1) * Noise_omega-Noise_omega/2.0
        [control, controller_params, error_controller] = controlInput(controller_params, thetadot_noisy)
        
        #noise = numpy.random.rand(3,1) * Noise_omega-Noise_omega/2.0
        omega = thetadot2omega(thetadot, theta)# + noise
         
        # Compute linear and angular accelerations.
        a = acceleration(control, theta, xdot, m, g, k, kd)
        omegadot = angular_acceleration(control, omega, I, L, b, k)
         
        omega = omega + dt * omegadot
        thetadot = omega2thetadot(omega, theta) 
        theta = theta + dt * thetadot
       
        # determine action,  20 * dt = 20*0.01 = 0.2 i.e. 5Hz position control
        if total_loop_passes%20 == 0:
            x_noisy = x + numpy.random.rand(3,1) * Noise_position-Noise_position/2.0
            a_pos_controller_intertial = controlPosition(x_noisy, setpoint)
            a_controller_body = rotation.inv_rotate(theta).dot(a_pos_controller_intertial)
            print ("a_controller_body: ", a_controller_body)
        
        # action
        a += a_controller_body
        print("a: ", a)
        
        #new position
        xdot = xdot + dt * a
        x = x + dt * xdot 
        
        # error after executing control
        x_err = x-setpoint
        
        total_pid_error = total_pid_error + numpy.square(x_err)
    
        # plot data (might be delayed)
        if total_loop_passes%10 == 0:
#            print ("plot new data")
            data["x"]= x
            data["x_err"] = x_err
            data["a"]= a
            data["theta"] = theta
            data["t"] = t
            data["error_controller"]=error_controller
            data["total_pid_error"]=total_pid_error
            pl.plot(data)
        
#         meassured_values_row['time'], meassured_values_row['x'], meassured_values_row['y'], meassured_values_row['z'] = t, x[0][0], x[1][0], x[2][0]
        meassured_values_row = {'time': t, 'x' : x[0][0], 'y': x[1][0], 'z': x[2][0],\
                            'x_err':"{:.5f}".format(x_err[0][0]), 'y_err': "{:.5f}".format(x_err[1][0]),\
                            'z_err':"{:.5f}".format(x_err[2][0]), 'ax': "{:.5f}".format(a[0][0]),\
                            'ay': "{:.5f}".format(a[1][0]),'az': "{:.5f}".format(a[2][0])}

        meassured_values.append(meassured_values_row)
    
        t=time.time() - start_time
        total_loop_passes+=1
        
        sleep_time = dt - (time.time()-start_loop_time)
        print("sleep_time: ", sleep_time)    
        time.sleep(max(0, sleep_time))
        
        if t>1.0 and sleep_time < 0:
            timeouts+=1 # attitude controller not called within dt
            
    with open(csv_file, 'w') as f:  # Just use 'w' mode in 3.x
        w = csv.DictWriter(f, meassured_values[0].keys(), delimiter=';')
        w.writeheader()
        w.writerows(meassured_values)
    f.close()
    
    data["x"]= x
    data["x_err"] = x_err
    data["a"]= a
    data["theta"] = theta
    data["t"] = t
    data["error_controller"]=error_controller
    data["total_pid_error"]=total_pid_error/total_loop_passes
    pl.plot(data)
    
    print('timeouts: ', timeouts, "/", total_loop_passes, " = ", timeouts/total_loop_passes)
    #wait till plotting has finished
    pl.plot_process.join()
    pl.plot(None, finished=True)
    

if __name__ == '__main__':
    main()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

