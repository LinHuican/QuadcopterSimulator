
class Pid():
    def __init__(self):
        self.p = 1.0
        self.setpoint_x = None
        self.setpoint_y = None
    
    def setpoint(self, x , y):
        self.setpoint_x = x
        self.setpoint_y = y
        
#     def input(self, x, y):
#         inp = numpy.zeros((4))
#         inp[:] = 290
#         inp[0] += 0.1
#         inp[2] += 0.1
#         inp = inp**2;
#         return inp