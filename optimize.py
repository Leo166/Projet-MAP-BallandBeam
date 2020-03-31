import numpy as np
from scipy import *
from utils import *

from scipy.optimize import minimize

def opt_param(x):
    def solve_equation(y, t, p, k):
        contr_simul = system.control
        u = contr_simul.last_u
        [x1, x2] = y
        dx1dt = x2
        if len(contr_simul.control) >= 2:
            derivative_control = contr_simul.get_derivative()
        else:
            derivative_control = 0
        dx2dt = (-(eq[1] + p) * x2 - eq[2] * x1 + eq[3] * np.sin(u) + m * x1 * derivative_control ** 2 + eq[4]) / (
                    eq[0] + k)
        return [dx1dt, dx2dt]

    def get_solution(ic, arg):
        contr_simul = system.control
        contr_simul.last_u = contr_simul.get_control(True)
        f = odeint(solve_equation, [ic[0], ic[1]], [0, dt], args=arg)
        pos = f[:, 0][-1]
        vel = f[:, 1][-1]
        if pos <= bc[0]:
            pos = bc[0]
            vel = 0
        elif pos >= bc[1]:
            pos = bc[1]
            vel = 0
        system.add_data([projection(contr_simul.last_u, pos, False), vel])
        return np.array([pos, vel])

    ctrl = [controller, controller1]

    def find_best_param(x):
        pp = []
        for cont in ctrl:
            system.control = cont
            t = 0
            xt = [cont.position[0], 0]
            system.add_data(xt)
            while t < time_simulation:  # bad
                xt = get_solution(xt, (x[0], x[1]))
                t += dt
            # print(len(system.position))
            # print(len(controller.position))
            pp.append(np.mean(abs(cont.position[0:len(system.position)] - system.position)))
            # print(system.position)
            system.position = []
            system.velocity = []
            system.current_ctrl = 0
            cont.count = 0
            cont.last_u = 0
        print("ok")
        return max(pp)
