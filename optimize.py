import numpy as np
from scipy import *
from utils import *
from controller import *
from dynamical_system import *
from scipy.optimize import minimize

def opt_param(x, controller):
    time_simulation = 15
    bc = [-100, 38.5]
    system = DynamicalSystem(controller, bc)
    def solve_equation(y, t, p, k, s):
        u = controller.last_u
        [x1, x2] = y
        dx1dt = x2
        if len(controller.control) >= 2:
            derivative_control = controller.get_derivative()
        else:
            derivative_control = 0
        dx2dt = (-(eq[1] + p) * x2 - eq[2] * x1 + eq[3] * np.sin(u) + s + m * x1 * derivative_control ** 2 + eq[4]) / (
                    eq[0] + k)
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
        system.add_data([projection(controller.last_u, pos, False), vel])
        return np.array([pos, vel])

    def find_best_param(x):
        t = 0
        xt = [controller.position[0], 0]
        system.add_data(xt)
        while t < time_simulation:  # bad
            xt = get_solution(xt, (x[0], x[1], x[2]))
            t += dt
        # print(len(system.position))
        # print(len(controller.position))
        # print(np.array(system.position))
        pp = np.mean(abs(controller.position[0:len(system.position)] - system.position))
        # print(system.position)
        system.position = []
        system.velocity = []
        system.current_ctrl = 0
        controller.count = 0
        controller.last_u = 0
        print("ok")
        return pp
    return find_best_param(x)
