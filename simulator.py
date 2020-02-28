import numpy as np
from scipy import *
from scipy.integrate import odeint
from matplotlib import pyplot as plt
from data import *
from utils import *
from controller import *

class Simulation:
    def __init__(self, time, system, controller):
        self.time = time
        self.system = system
        self.controller = controller
        self.ic = 0
        self.bc = 0

    def start_simulation(self):

class DynamicalSystem:
    def __init__(self, setpoint, control, bc):
        self.setpoint = setpoint
        self.control = control
        self.bc = bc

    def solve_equation(self, y, t):
        ctrl = self.control
        u = ctrl.get_control()
        [x1, x2] = y
        dx1dt = x2
        if len(ctrl) >= 2:
            deriv_control = ctrl.get_derivative()
        else:
            deriv_control = 0
        dx2dt = (-eq[1]*x2 - eq[2]*x1 + eq[3]*np.sin(u) + m*x1*deriv_control**2 + eq[4])/eq[0]
        return [dx1dt, dx2dt]

    def get_solution(self):
        bc = self.bc
        setpoint = self.setpoint
        f = odeint(self.solve_equation, [setpoint[0], setpoint[1]], [0, dt])
        pos = f[:, 0][-1]
        vit = f[:, 1][-1]
        if pos <= bc[0]:
            pos = bc[0]
            vit = 0
        elif pos >= bc[1]:
            pos = bc[1]
            vit = 0
        position.append(pos)
        vitesse.append(vit)
        return np.array([pos, vit])

simulation = Simulation(15,)