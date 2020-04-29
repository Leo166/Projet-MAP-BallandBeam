import numpy as np
from scipy import *
from dynamical_system import *
from simulator import Simulation
from scipy.optimize import minimize

"""
Optimize the parameters in the dynamical system which have to be put to [0, 0, 0, 0] 
(the function is use by minimize from Python !)
@param array; x; parameters to find
@param object ManualControllerFile; controller;  controller use in for the dynamical system
@return mean squared error for these parameters
 """


"""
Optimize the parameters in the dynamical system which have to be put to [0, 0, 0, 0] 
(the function is use by minimize from Python !)
@param array; x; parameters to find
@param object ManualControllerFile; controller;  controller use in for the dynamical system
@return mean squared error for these parameters
 """



"""
Optimize the parameters in the dynamical system which have to be put to [0, 0, 0, 0] 
(the function is use by minimize from Python !)
@param array; x; parameters to find
@param object ManualControllerFile; controller;  controller use in for the dynamical system
@return mean squared error for these parameters
 """
def opt_param(x, controller, t):
    time_simulation = t
    bc = [-38.5, 38.5]
    system = DynamicalSystem(controller, bc, False)
    def solve_equation(y, t, p, k, s, f):
        u = controller.last_u
        [x1, x2] = y
        dx1dt = x2
        if len(controller.control) >= 2:
            derivative_control = controller.get_derivative()
        else:
            derivative_control = 0
        dx2dt = (-(eq[1] + k) * x2 - eq[2] * x1 + (eq[3] + s) * np.sin(u) + f + m * x1 * derivative_control ** 2 + eq[4]) / (eq[0] + p)
        return [dx1dt, dx2dt]

    def get_solution(ic, arg):
        controller.last_u = controller.get_control(True)
        f = odeint(solve_equation, [ic[0], ic[1]], [0, dt], args=arg)
        pos = f[:, 0][-1]
        vel = f[:, 1][-1]
        if pos <= bc[0]:
            pos = bc[0]
            vel = 0
        elif pos >= bc[1]:
            pos = bc[1]
            vel = 0

        # Correction on the speed at low speed at low variation angle
        if abs(vel) <= 0.4:
            if controller.count >= 1:
                if abs(controller.control[controller.count] - controller.last_u) <= np.deg2rad(0.5):
                    pos = ic[0]
                    vel = 0

        system.add_data([projection(controller.last_u, pos, False), vel])
        return np.array([pos, vel])

    def find_best_param(x):
        t = 0
        xt = [controller.position[0], 0]
        system.add_data(xt)
        while t < time_simulation:  # bad
            xt = get_solution(xt, (x[0], x[1], x[2], x[3]))
            t += dt
        pp = np.mean(abs(controller.position[0:len(system.position)] - system.position))
        system.position = []
        system.velocity = []
        system.current_ctrl = 0
        controller.count = 0
        controller.last_u = 0
        return pp

    return find_best_param(x)


#trajectoire

