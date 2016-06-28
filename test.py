import unittest
import Rotation
import numpy

class TestRl(unittest.TestCase):
    def setUp(self):
        pass
    
    def test_rotation_about_xyz(self):
        rotation = Rotation.Rotation()
        
        vector1=numpy.array([[1],[0],[0]])
        vector2=numpy.array([[0],[1],[0]])
        vector3=numpy.array([[0],[0],[1]])
        
        angle=numpy.array([[numpy.pi/2],[0],[0]])   # gamma, beta, alpha        about x
        vector1_x_1 = rotation.rotate(angle).dot(vector1)
        vector2_x_2 = rotation.rotate(angle).dot(vector2)
        vector3_x_3 = rotation.rotate(angle).dot(vector3)
        
#         print (vector1_x_1)   # 1 0 0
#         print (vector2_x_2)   # 0 0 1
#         print (vector3_x_3)   # 0 -1 0
        numpy.testing.assert_almost_equal(vector1_x_1, numpy.array([[1], [0], [0]]), decimal=15) 
        numpy.testing.assert_almost_equal(vector2_x_2, numpy.array([[0], [0], [1]]), decimal=15) 
        numpy.testing.assert_almost_equal(vector3_x_3, numpy.array([[0], [-1], [0]]), decimal=15) 
        
        angle=numpy.array([[0],[numpy.pi/2],[0]])    # gamma, beta, alpha       about y
        vector1_y_1 = rotation.rotate(angle).dot(vector1)
        vector2_y_2 = rotation.rotate(angle).dot(vector2)
        vector3_y_3 = rotation.rotate(angle).dot(vector3)
        
#         print (vector1_y_1)   # 0 0 -1
#         print (vector2_y_2)   # 0 1 0
#         print (vector3_y_3)   # 1 0 0
        
        numpy.testing.assert_almost_equal(vector1_y_1, numpy.array([[0], [0], [-1]]), decimal=15) 
        numpy.testing.assert_almost_equal(vector2_y_2, numpy.array([[0], [1], [0]]), decimal=15) 
        numpy.testing.assert_almost_equal(vector3_y_3, numpy.array([[1], [0], [0]]), decimal=15) 
        
        angle=numpy.array([[0],[0],[numpy.pi/2]])    # gamma, beta, alpha       about z
        vector1_z_1 = rotation.rotate(angle).dot(vector1)
        vector2_z_2 = rotation.rotate(angle).dot(vector2)
        vector3_z_3 = rotation.rotate(angle).dot(vector3)
        
#         print (vector1_z_1)   # 0 1 0
#         print (vector2_z_2)   # -1 0 0
#         print (vector3_z_3)   # 0 0 1
        
        numpy.testing.assert_almost_equal(vector1_z_1, numpy.array([[0], [1], [0]]), decimal=15) 
        numpy.testing.assert_almost_equal(vector2_z_2, numpy.array([[-1], [0], [0]]), decimal=15) 
        numpy.testing.assert_almost_equal(vector3_z_3, numpy.array([[0], [0], [1]]), decimal=15) 



    def test_inv_rotation_about_xyz(self):
        rotation = Rotation.Rotation()
        
        vector1_body = numpy.array([[1],[0],[0]])
        vector2_body = numpy.array([[0],[1],[0]])
        vector3_body = numpy.array([[0],[0],[1]])
        
        angle=numpy.array([[numpy.pi/2],[0],[0]])   # gamma, beta, alpha        about x
        vector1_x_1_inertial = rotation.inv_rotate(angle).dot(vector1_body)
        vector2_x_2_inertial = rotation.inv_rotate(angle).dot(vector2_body)
        vector3_x_3_inertial = rotation.inv_rotate(angle).dot(vector3_body)
        
        numpy.testing.assert_almost_equal(vector1_x_1_inertial, numpy.array([[1], [0], [0]]), decimal=15)
        numpy.testing.assert_almost_equal(vector2_x_2_inertial, numpy.array([[0], [0], [-1]]), decimal=15)
        numpy.testing.assert_almost_equal(vector3_x_3_inertial, numpy.array([[0], [1], [0]]), decimal=15) 
