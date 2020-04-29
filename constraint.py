import numpy as np

"""
Manage the control of the ball as it comes near to the edges
@param array; position; positions of the ball during the simulation
@param array; velocity; velocities of the ball during the simulation
@param int; v; current control applied to the motor
@return int; v; eventually modified control to apply on the motor
 """
def idiot_proofing(position, velocity, v):
    if (abs(position[-1]) > 31.5):           # position[-1] the current position of the ball & 31.5 chosen experimentally
        p1 = 10                              # p1 computed experimentally
        v_max = p1 / (abs(position[-1]))     # computed speed threshold
        
        if (abs(velocity[-1]) > v_max):      # velocity[-1] the current velocity of the ball
            p2 = -100                        # p2 computed experimentally
         
            if(abs(v) < 0.04):               # 0.04 chosen experimentally
                v -= p2*np.deg2rad(np.tan((position[-1]-18*np.sign(position[-1]))*np.pi/80))
                # if the current control is smaller than the chosen limit, we decrease it by a value based on some experimentally
                # chosen parameters and the tangent function applied on the current position
            else:
                v = -2*v                     # else we set a control in the opposite way of the current one
                

    if(abs(v) > np.deg2rad(11)):             # if the control is bigger than a limit near to the maximum applicable by the
        v = np.deg2rad(11*np.sign(v))        # motor, we replace it by this limit
      
    return v
