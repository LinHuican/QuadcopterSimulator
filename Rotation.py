import numpy
import math

class Rotation:
    
    def inv_rotate(self, theta):
        return numpy.linalg.inv(self.rotate(theta))
    
    def rotate(self, theta):
        #R = _rotationZ(theta[0]).dot(_rotationX(theta[1]).dot(_rotationZ(theta[2])))
        R = self._rotationZ(theta[2]).dot(self._rotationY(theta[1]).dot(self._rotationX(theta[0])))
        return R
        
    def _rotationX(self, angle):
        R_x = numpy.array([[               1,               0,                0],
                           [               0, math.cos(angle), -math.sin(angle)],
                           [               0, math.sin(angle),  math.cos(angle)]])
        return R_x
    
    def _rotationY(self, angle):
        R_y = numpy.array([[ math.cos(angle),               0,  math.sin(angle)],
                       [               0,               1,               0],
                       [-math.sin(angle),               0,  math.cos(angle)]])
        return R_y
        
    def _rotationZ(self, angle):
        R_z = numpy.array([[ math.cos(angle), -math.sin(angle),               0],
                           [ math.sin(angle),  math.cos(angle),               0],
                           [               0,               0,                1]])
        return R_z