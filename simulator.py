import numpy as np
from scipy import *

from matplotlib import pyplot as plt
from scipy.optimize import minimize

from dynamical_system import *
from utils import *
from controller import *

class Simulation:
    def __init__(self, time, system, controller, set_point, ic, bc):
        self.time = time
        self.system = system
        self.controller = controller
        self.set_point = set_point
        self.ic = ic
        self.bc = bc
        self.run = True

    def start_simulation(self):
        self.run = True
        t = 0
        xt = self.ic
        xs = self.set_point
        self.system.add_data(self.ic)
        while t <= self.time and self.run:  # bad
            if self.controller.need_error:
                self.controller.add_error(xs - xt[0])
            xt = self.system.get_solution(xt)
            t += dt
        return xt

    def break_simulation(self):
        self.run = False
        return

    def stop_simulation(self):
        return


time_simulation = 15   # duration of the simulation [s]
set_point = 3  # point where the ball has to be stabilized [m]
ic = [0, 0]            # initial position and speed [m]
bc = [-34.1, 38.15]    #[-34.1, 38.15]

# controller = PIDController([-10, 0, 0], dt)
controller = ManualController("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/sinus/sin_30_005.txt", time_simulation)
# controller = ManualController("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/squarred/square_20_10.txt", time_simulation)
# controller = ManualController("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/closed_loop/test_1.txt", time_simulation)
system = DynamicalSystem(controller, bc)
ic = [controller.position[0], 0]            # initial position and speed [m]
simulation = Simulation(time_simulation, system, controller, set_point, ic, bc)
simulation.start_simulation()

position = system.position
velocity = system.velocity
command = controller.control
print(command)
# print(command)
positionsys = controller.position
# print(np.array(command))

t = np.arange(0.0, time_simulation, dt)
graphique([t], [position[0:-1]], [velocity[0:-1]], [positionsys[1:len(position)]], [command[1:len(position)]])

# def solve_equation(y, t, p, k):
#     u = controller.last_u
#     [x1, x2] = y
#     dx1dt = x2
#     if len(controller.control) >= 2:
#         derivative_control = controller.get_derivative()
#     else:
#         derivative_control = 0
#     dx2dt = (-(eq[1] + p)*x2 - eq[2]*x1 + eq[3]*np.sin(u) + m*x1*derivative_control**2 + eq[4])/(eq[0] + k)
#     return [dx1dt, dx2dt]
#
# def get_solution(ic, arg):
#     controller.last_u = controller.get_control(True)
#     f = odeint(solve_equation, [ic[0], ic[1]], [0, dt], args=arg)
#     pos = f[:, 0][-1]
#     vel = f[:, 1][-1]
#     if pos <= bc[0]:
#         pos = bc[0]
#         vel = 0
#     elif pos >= bc[1]:
#         pos = bc[1]
#         vel = 0
#     system.add_data([pos, vel])
#     return np.array([pos, vel])
#
# def find_best_param(x):
#     t = 0
#     xt = ic
#     xs = set_point
#     system.add_data(ic)
#     while t < time_simulation:  # bad
#         xt = get_solution(xt, (x[0], x[1]))
#         t += dt
#     # print(len(system.position))
#     # print(len(controller.position))
#     ppp = np.mean(abs(controller.position[0:len(system.position)] - system.position))
#     # print(system.position)
#     # print(ppp)
#     system.position = []
#     system.velocity = []
#     system.current_ctrl = 0
#     controller.count = 0
#     lastu = 0
#     print("ok")
#     return ppp
#
# #[0, 0] -- sinus -- Nelder-Mead -- [0.05392127 0.31127394] -- 14.831
# #[0, 0] -- sinus -- BFGS, Powell -- [0.60275368 0.14233554] -- 2.3928
#
# #[0, 0] -- carré -- BFGS, Powell -- [0.52422432 0.08500503] -- 5.0794
# #[0, 0] -- carré -- Nelder-Mead -- [0.08980634 0.27940342] -- 13.1126
# res = minimize(find_best_param, np.array([0, 0]), method='Powell', tol=1e-6)
# print(res)
# print(res.x)
# print(res.fun)

# t = np.arange(0.0, time_simulation+2*dt, dt)
# # plt.plot(t[0:-2], np.array(positionsys[1:len(position)]), label="System [cm]")
# plt.plot(t[0:-1], np.array(position), "--", label="Simulation [cm]")
# # plt.plot(t[0:-1], np.array(vitesse), label="Velocity", color= "green")
# # plt.plot(t[0:-2], np.array(command), label="Commande")
# # plt.plot(t, commandmanu, label="Motor control [°]")
# plt.plot(t[0:-2], np.rad2deg(command[0:len(position)]), label="Control")
# plt.hlines(set_point, 0, time_simulation, 'black', '--', linewidth=1)
# # plt.hlines(bc[0], 0, time_simulation, 'red', '--', linewidth=1)
# # plt.hlines(bc[1], 0, time_simulation, 'red', '--', linewidth=1)
# # plt.hlines(-34.1, 0, time_simulation, 'black', '--', linewidth=1)
# plt.xlabel("Time [s]")
# # plt.ylabel("Position [cm]")
# plt.legend()
# plt.show()
