import numpy as np
from scipy.integrate import odeint
from utils import *

"""Caractéristique système"""
dt = 50*10**-3      # frequency of mesure [s]
g = 9.81*10**2           # acceleration [cm/s^2]
# m = 0.0559          # mass [kg]
m = 0.0576          # mass [kg] FW
# r = 15*10**-1       # ball radius [cm]
r = 15.1*10**-1       # ball radius [cm] FW
J = (2*m*r**2)/5    # inertial moment
eta = 10**-5        # dynamic viscosity [kg/ms]
vs = 1.414*10       # ball volume [cm^3]
ro = 997/(10**6)            # density of water [kg/cm^3]
param = [0, 0, 0, 0, 0]
# param = [0.10745968, 0.30602608, 0, -0.93278782]#ok
# param = [0.11809485,  0.2819055,  -0.0154096,  -0.41265424]   #ok ref
# param = [0.11734496,  0.28836237, -0.12857947, -0.61645939]   #pas mal
# param = [0.12065253,  0.36546263, -0.00968162, -0.36841601]   #sans correct
# param = [0.04665993,  0.04139925, -0.01280646, -0.51983207]
# param = [0.10767055,  0.29354308,-0.00990855, -0.20403751] #good
# param = [ 0.05279321,  0.30701569, -0.00741271, -0.24989793] #0.2
# param = [0.10453377,  0.29179852, -0.01422285, -0.24478251] #0.3
# param = [0.09231504,  0.29565666, -0.01016332, -0.19932541] #0.4 - 3.82
# param = [ 0.10210506,  0.29080151, -0.00997114, -0.18775064]#0.36 with big one- 3.9721
# param = [0.10210506,  0.29080151, -0.00997114 ,-0.18775064]#0.36 with 2 big one- ?
param = [0.10399143,  0.28519767, -0.01006414, -0.17522689]#0.4 with 2 big one - 4.028 very good


#Autre test:
# param = [0.09814629,  0.27466383, -0.01002565, -0.20379595] #erreur 7.23 angle 0.5 vel 0.4
# param = [ 0.10935873,  0.29425049, -0.00970073, -0.23647272] #erreur 9.7977 angle 5 vel 0.4

eq = [m + J/r**2 + param[0], 6*r*eta*np.pi + param[1], 0, -m*g + ro*vs*g + param[2], 0]   #equa_diff 5y" + 4y' + 3y = 2u + const 0.4
# eq = [5, 4, 3, 2, 0]


class DynamicalSystem:
    def __init__(self, control, bc, idiot_proof, speed_limit):
        self.control = control
        self.bc = bc
        self.current_ctrl = 0
        self.position = []
        self.velocity = []
        self.acceleration = []
        self.idiot_proof = idiot_proof
        self.speed_limit = speed_limit

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

        # if self.speed_limit != None:
        #     vel_max = self.speed_limit[1]
        #     vel_min = self.speed_limit[0]
        #     if abs(vel) > abs(vel_max) or abs(vel) < abs(vel_min):
        #         if vel >=0:
        #             if vel > vel_max:
        #                 vel = vel_max
        #             else:
        #                 vel = vel_min
        #         else:
        #             if abs(vel) > vel_max:
        #                 vel = -vel_max
        #             else:
        #                 vel = -vel_min

        # Correction sur la balle à faible vitesse à faible angle
        # if abs(vel) <= 0.4:
        #     if self.control.count >= 1:
        #         if abs(self.control.control[self.control.count] - self.current_ctrl) <= np.deg2rad(0.5):
        #             pos = ic[0]
        #             vel = 0

        self.add_data([projection(self.current_ctrl, pos, False), vel])
        # self.add_data([pos, vel])
        return np.array([pos, vel])

