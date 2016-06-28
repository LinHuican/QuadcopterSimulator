import numpy
import math

class Controller():
    
    # Implement a PD controller. See simulate(controller).
    def pd_controller(self, state, thetadot, Kd, Kp, ):
        # Initialize integral to zero when it doesn't exist.
        if not 'integral' in state:
            state['integral'] = numpy.zeros((3, 1))
    
        # Compute total thrust.
        total = state['m'] * state['g'] / state['k'] / (math.cos(state['integral'][0] * math.cos(state['integral'][1])))
    
        # Compute PD error and inputs.
        err = Kd * thetadot + Kp * state['integral']
        controller_output = self.err2inputs(state, err, total)
    
        # Update controller state.
        state['integral'] = state['integral'] + numpy.multiply(state['dt'], thetadot)
        
        return (controller_output, state, err)
        
        
    # Given desired torques, desired total thrust, and physical parameters,
    # solve for required system inputs.
    def err2inputs(self, state, err, total):
        e1 = float(err[0])
        e2 = float(err[1])
        e3 = float(err[2])
        Ix = state['I'][0, 0]
        Iy = state['I'][1, 1]
        Iz = state['I'][2, 2]
        k = state['k']
        L = state['L']
        b = state['b']
    
        inputs = numpy.zeros((4))
        inputs[0] = total/4 -(2 * b * e1 * Ix + e3 * Iz * k * L)/(4 * b * k * L)
        inputs[1] = total/4 + e3 * Iz/(4 * b) - (e2 * Iy)/(2 * k * L)
        inputs[2] = total/4 -(-2 * b * e1 * Ix + e3 * Iz * k * L)/(4 * b * k * L)
        inputs[3] = total/4 + e3 * Iz/(4 * b) + (e2 * Iy)/(2 * k * L)
        
        return inputs
    
    
class PositionController():
    def p_controller(self, x, setpoint, Kp):
        
        error = numpy.zeros((3,1))
        error[0] = setpoint[0] - x[0]
        error[1] = setpoint[1] - x[1]
        output = Kp * error
        return output