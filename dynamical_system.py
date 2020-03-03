import numpy as np
from scipy.integrate import odeint

"""Caractéristique système"""
dt = 50*10**-3      # frequency of mesure [s]
g = 9.81            # acceleration [m/s^2]
m = 0.0559          # mass [kg]
r = 15*10**-3       # ball radius [m]
J = (2*m*r**2)/5    # inertial moment
eta = 10**-3        # dynamic viscosity [kg/ms]
vs = 1.414*10**-5   # ball volume [m^3]
ro = 997            # density of water [kg/m^3]

eq = [m + J/r**2, 6*r*eta*np.pi + 0.4, 0, -m*g + ro*vs*g, 0]   #equa_diff 5y" + 4y' + 3y = 2u + const 0.4

class DynamicalSystem:
    def __init__(self, control, bc):
        self.control = control
        self.bc = bc
        self.last_ctrl = 0
        self.position = []
        self.velocity = []

    def add_data(self, data):
        self.position.append(data[0])
        self.velocity.append(data[1])

    def solve_equation(self, y, t):
        u = self.last_ctrl
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
        self.last_ctrl = self.control.get_control(True)
        f = odeint(self.solve_equation, [ic[0], ic[1]], [0, dt])
        pos = f[:, 0][-1]
        vel = f[:, 1][-1]
        if pos <= bc[0]:
            pos = bc[0]
            vel = 0
        elif pos >= bc[1]:
            pos = bc[1]
            vel = 0
        self.add_data([pos, vel])
        return np.array([pos, vel])

