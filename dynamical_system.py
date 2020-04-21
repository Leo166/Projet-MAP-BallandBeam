import numpy as np
from scipy.integrate import odeint
from utils import *

"""Caractéristique système"""
dt = 50*10**-3      # frequency of mesure [s]
g = 9.81*10**2           # acceleration [cm/s^2]
m = 0.0559          # mass [kg]
# m = 0.0576          # mass [kg] FW
r = 15*10**-1       # ball radius [cm]
# r = 15.1*10**-1       # ball radius [cm] FW
J = (2*m*r**2)/5    # inertial moment
eta = 10**-5        # dynamic viscosity [kg/ms]
vs = 1.414*10       # ball volume [cm^3]
ro = 997/(10**6)            # density of water [kg/cm^3]
param = [0, 0, 0, 0, 0]
# param = [0.10745968, 0.30602608, 0,-0.93278782]#ok
param = [0.11809485,  0.2819055,  -0.0154096,  -0.41265424]   #ok ref
# param = [0.11734496,  0.28836237, -0.12857947, -0.61645939]   #pas mal
# param = [ 0.10997653,  0.29186298, -0.00358023, -0.72726524]
# param = [0.12921003,  0.27934707, -0.12065793, -0.62997013]
# param = [ 0.11970217,  0.27541133, -0.13573101, -0.62753956]
# param = [0.10827614,  0.27581379, -0.14696738, -0.44238413]
# param = [ 0.12857387,  0.25753001,  1.17179222, -0.5302399] #tout fichier
eq = [m + J/r**2 + param[0], 6*r*eta*np.pi + param[1], 0, -m*g + ro*vs*g + param[2], 0]   #equa_diff 5y" + 4y' + 3y = 2u + const 0.4
# eq = [5, 4, 3, 2, 0]

class DynamicalSystem:
    def __init__(self, control, bc):
        self.control = control
        self.bc = bc
        self.current_ctrl = 0
        self.position = []
        self.velocity = []
        self.acceleration = []

    def add_data(self, data):
        self.position.append(data[0])
        if len(self.velocity) != 0:
            self.acceleration.append((data[1]-self.velocity[-1])/dt)
        self.velocity.append(data[1])

    def solve_equation(self, y, t):
        u = self.current_ctrl
        [x1, x2] = y
        dx1dt = x2
        if len(self.control.control) >= 2:
            derivative_control = self.control.get_derivative()
        else:
            derivative_control = 0
        dx2dt = (-eq[1]*x2 - eq[2]*x1 + eq[3]*np.sin(u) + m*x1*derivative_control**2 + param[3] + eq[4])/eq[0]
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

        # print(vel)
        # if abs(vel) <= 0.34:
        #     if self.control.count >= 1:
        #         if abs(self.control.control[self.control.count] - self.current_ctrl) <= np.deg2rad(0.5):
        #             pos = ic[0]
        #             vel = 0

        # vel = correction_low_speed(vel)

        self.add_data([projection(self.current_ctrl, pos, False), vel])
        return np.array([pos, vel])

