import numpy as np
from scipy.integrate import odeint
from utils import *

"""Caractéristique système"""
dt = 50*10**-3      # frequency of mesure [s]
g = 9.81*10**2           # acceleration [m/s^2]
m = 0.0559          # mass [kg]
r = 15*10**-1       # ball radius [m]
J = (2*m*r**2)/5    # inertial moment
eta = 10**-5        # dynamic viscosity [kg/ms]
vs = 1.414*10   # ball volume [m^3]
ro = 997/(10**6)            # density of water [kg/m^3]
param = [0.5, 0]
# param = [0.05392127, 0.31127394] #ok
# param = [0.60276305, 0.14232733]
# param = [0.15394507, 0.27942107]
eq = [m + J/r**2 + param[1], 6*r*eta*np.pi + param[0], 0, -m*g + ro*vs*g, 0]   #equa_diff 5y" + 4y' + 3y = 2u + const 0.4
# eq = [5, 4, 3, 2, 0]

class DynamicalSystem:
    def __init__(self, control, bc):
        self.control = control
        self.bc = bc
        self.current_ctrl = 0
        self.position = []
        self.velocity = []

    def add_data(self, data):
        self.position.append(data[0])
        self.velocity.append(data[1])

    def solve_equation(self, y, t):
        u = self.current_ctrl
        [x1, x2] = y
        dx1dt = x2
        if len(self.control.control) >= 2:
            derivative_control = self.control.get_derivative()
        else:
            derivative_control = 0
        dx2dt = (-eq[1]*x2 - eq[2]*x1 + eq[3]*np.sin(u) + m*x1*derivative_control**2 + eq[4])/eq[0]
        return [dx1dt, dx2dt]

    def get_solution(self, ic):
        bc = self.bc
        self.current_ctrl = self.control.get_control(True)
        f = odeint(self.solve_equation, [ic[0], ic[1]], [0, dt])
        pos = f[:, 0][-1]
        vel = f[:, 1][-1]
        if pos <= bc[0]:
            pos = bc[0]
            vel = 0
        elif pos >= bc[1]:
            pos = bc[1]
            vel = 0
        # pos = projection(self.current_ctrl, pos, False)
        self.add_data([pos, vel])
        return np.array([pos, vel])

