import numpy as np
from scipy import *
from utils import *
from controller import *
from dynamical_system import *
from simulator import *
from scipy.optimize import minimize

def opt_param(x, controller, t):
    time_simulation = t
    bc = [-38.5, 38.5]
    system = DynamicalSystem(controller, bc)
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

        if abs(vel) <= 0.4:
            if controller.count >= 1:
                if abs(controller.control[controller.count] - controller.last_u) <= np.deg2rad(5):
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
        return pp
    return find_best_param(x)

"""--------------------------"""
"""--------------------------"""


def opt_trajectory(ic, speed_limit):
    def find_best_trajectory(k):
        bc = [-38.15, 38.15]
        controller = PIDController([k[0], k[1], k[2]], dt)
        system = DynamicalSystem(controller, bc)
        simulation = Simulation(system, controller, set_point, ic, bc)
        position = simulation.start_simulation()
        return scalar
    res = minimize(find_best_trajectory, np.array([0, 0, 0, 0]), method='BFGS', tol=1e-6)
    return np.array(res.x)

# [0, 0] -- carré -- BFGS, Powell -- [0.52422432 0.08500503] -- 5.0794
# [0, 0] -- carré -- Nelder-Mead -- [0.08980634 0.27940342] -- 13.1126
