import numpy as np

"""

@param array; position; positions of the ball during the simulation
@param array; velocity; velocities of the ball during the simulation
@param int; v; current control applied to the motor
@return int; v; eventually modified control to apply on the motor
 """
def idiot_proofing(position, velocity, v):
    if (abs(position[-1]) > 31.5):
        p1 = 10
        v_max = p1 / (abs(position[-1]))
        
        if (abs(velocity[-1]) > v_max):
            p2 = -100
         
            if(abs(v) < 0.04):
                v -= p2*np.deg2rad(np.tan((position[-1]-18*np.sign(position[-1]))*np.pi/80))
                
            else:
                v = -2*v
                

    if(abs(v) > np.deg2rad(11)):
        v = np.deg2rad(11*np.sign(v))
      
    return v
