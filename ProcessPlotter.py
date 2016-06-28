from __future__ import print_function
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
import matplotlib.pyplot as plt
import Rotation
import numpy
import collections

enable3d = True

Number_of_datapoints = 100

class ProcessPlotter(object):
    def __init__(self):
        self.errors_controller = collections.deque(Number_of_datapoints*[numpy.zeros(3)], Number_of_datapoints)
        self.time_stamps = collections.deque(Number_of_datapoints*[0], Number_of_datapoints)
        pass

    def terminate(self):
        plt.close('all')

    def call_back(self):
        while 1:
            # wait a second for data
            if not self.pipe.poll(1):
                print("Timeout - no data to poll from pipe")
                break

            command = self.pipe.recv()
            if command is None:
                self.terminate()
                print("No command received - terminate")
                return False

            else:
#                    print("command: ", command)
                xBody = numpy.array([[1], [0], [0]])
                yBody = numpy.array([[0], [1], [0]])
                zBody = numpy.array([[0], [0], [1]])
                
                x = command['x']
                a = command['a']
                theta = command['theta']
                t = command['t']
                self.errors_controller.append(command['error_controller'])
                #temp = command['error_controller']
                
                self.time_stamps.append(t)
                
                xpos, ypos, zpos = x[0][0], x[1][0], x[2][0]
 
                if enable3d:
                    self.ax3d.cla()
                    self.ax3d.set_ylim([-10, 10])
                    self.ax3d.set_xlim([-10, 10])
                    self.ax3d.set_zlim([ 0, 5])
                    self.ax3d.plot( [xpos], [ypos], [zpos], "bo")
                    self.ax3d.plot( [xpos, xpos + a[0]], [ypos, ypos+a[1]] , [zpos, zpos+a[2]],"g-",linewidth=2)
                    self.ax3d.set_title( "x: " + str(xpos) + "   y: " + str(ypos) + "   z: " + str(zpos) + "  t: " + str(t) )
                    self.ax3d.set_xlabel("X")
                    self.ax3d.set_ylabel("Y")
                    self.ax3d.set_zlabel("Z")
                    plt.draw()
                
                rotation = Rotation.Rotation()
                R = rotation.rotate(theta)
                xBody_current = R.dot(xBody)
                yBody_current = R.dot(yBody)
                zBody_current = R.dot(zBody)
            
                self.ax_xyplane.set_title("XY - plane: " +str(xpos) + ", " + str(ypos) +"    t: " + str(t))
                self.line1_1.set_xdata([xpos, xpos + xBody_current[0]])
                self.line1_1.set_ydata([ypos, ypos + xBody_current[1]])
                self.line1_2.set_xdata([xpos, xpos + yBody_current[0]])
                self.line1_2.set_ydata([ypos, ypos + yBody_current[1]])
            
                self.line2_1.set_xdata([xpos, xpos + 4*a[0]])
                self.line2_1.set_ydata([zpos, zpos + 4*a[2]])
                self.line2_2.set_xdata([xpos, xpos + zBody_current[0]])
                self.line2_2.set_ydata([zpos, zpos + zBody_current[2]])
                
                # plot error ins orientation
                self.line3_1.set_xdata(list(self.time_stamps))
                self.line3_2.set_xdata(list(self.time_stamps))
                self.line3_3.set_xdata(list(self.time_stamps))
                self.line3_1.set_ydata(numpy.array(self.errors_controller)[:,0])
                self.line3_2.set_ydata(numpy.array(self.errors_controller)[:,1])
                self.line3_3.set_ydata(numpy.array(self.errors_controller)[:,2])
                
            #    ax_xzplane.plot(xpos, zpos, "go")
                #
                self.fig.canvas.draw()
                
        self.fig.canvas.draw()
        return True


    def __call__(self, pipe):
        print('starting plotter...')

        self.pipe = pipe
        #self.fig, self.ax = plt.subplots()

        xpos, ypos, zpos = 0,0,0
        t = 0
        error_controller = numpy.zeros(3)

        self.fig = plt.figure(figsize=(10,16))
        self.ax_xyplane = self.fig.add_subplot(311)
        self.ax_xzplane = self.fig.add_subplot(312)
        self.ax_err_controller_plane = self.fig.add_subplot(313)
        plt.tight_layout(pad=4.0, w_pad=2.0, h_pad=2.0)
        
        if enable3d:
            plt.ion()
            self.fig3d = plt.figure(figsize=(10,16))
            self.ax3d = self.fig3d.add_subplot(111, projection='3d')
            plt.figure(2)
         
        self.line1_1, = self.ax_xyplane.plot(xpos, ypos, "b") # vector in x-direction 
        self.line1_2, = self.ax_xyplane.plot(xpos, ypos, "g") # vector in y-direction 
        self.line2_1, = self.ax_xzplane.plot(xpos, ypos, "r") # force in z-direction
        self.line2_2, = self.ax_xzplane.plot(xpos, ypos, "b") # vector in z-direction
        self.line3_1, = self.ax_err_controller_plane.plot(error_controller, "or") 
        self.line3_2, = self.ax_err_controller_plane.plot(error_controller, "ob") 
        self.line3_3, = self.ax_err_controller_plane.plot(error_controller, "og") 
        #self.line3_2, = self.ax_err_controller_plane.plot(t, error_controller,"b") 
         
        self.ax_xyplane.set_ylim([-5,5])
        self.ax_xyplane.set_xlim([-8,8])
        self.ax_xyplane.set_title("XY - plane")
        self.ax_xyplane.set_xlabel("X")
        self.ax_xyplane.set_ylabel("Y")
        
        self.ax_xzplane.set_ylim([-5,5])
        self.ax_xzplane.set_xlim([-8,8])
        self.ax_xzplane.set_title("XZ - plane")
        self.ax_xzplane.set_xlabel("X")
        self.ax_xzplane.set_ylabel("Z")
        
        self.ax_err_controller_plane.set_ylim([-5,5])
        self.ax_err_controller_plane.set_xlim([0, 15])
        self.ax_err_controller_plane.set_title("error - plane")
        self.ax_err_controller_plane.set_xlabel("t")
        self.ax_err_controller_plane.set_ylabel("error")
        
        timer = self.fig.canvas.new_timer(interval=100)
        timer.add_callback(self.call_back)
        timer.start()

        print('...done')
        plt.show()