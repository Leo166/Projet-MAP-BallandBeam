import numpy as np
from scipy import *

from matplotlib import pyplot as plt
from scipy.optimize import minimize

from dynamical_system import *
from utils import *
from controller import *
from optimize import *

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
        self.system.add_data([projection(self.controller.control[0], self.ic[0], False), self.ic[1]])
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


time_simulation = 20   # duration of the simulation [s]
set_point = 3  # point where the ball has to be stabilized [m]
ic = [0, 0]            # initial position and speed [m]
bc = [-38.15, 38.15]    #[-34.1, 38.15]

# controller = PIDController([-10, 0, 0], dt)
# controller = ManualController("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/sinus/sin_20_005.txt", time_simulation)
# controller = ManualController("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/sinus/sin_30_005.txt", time_simulation)
# controller = ManualController("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/sinus/sin_40_005.txt", time_simulation)
controller = ManualController("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/squarred/square_20_20.txt", time_simulation)
controller = ManualController("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/squarred/square_30_20.txt", time_simulation)
# controller = ManualController("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/squarred/square_40_20.txt", time_simulation)
# controller4 = ManualController("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/free_control/test2.txt", time_simulation)
# controller5 = ManualController("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/free_control/test3.txt", time_simulation)
system = DynamicalSystem(controller, bc)
ic = [controller.position[0], 0]            # initial position and speed [m]
simulation = Simulation(time_simulation, system, controller, set_point, ic, bc)
simulation.start_simulation()

position = system.position
velocity = system.velocity
command = controller.control
# print(command)
# print(command)
positionsys = controller.position
print(position)
# print(np.where(positionsys==38.817591))
print(command)
# print(np.array(command))

t = np.arange(0.0, time_simulation, dt)
graphique([t], [position[0:-1]], [velocity[0:-1]], [positionsys[1:len(position)]], [command[1:len(position)]])
breakpoint()

# ctrl = [controller, controller1, controller2, controller3, controller4, controller5]
ctrl = [controller, controller1]
n = 0
def opt_para_more_data(x):
    pp = []
    for cont in ctrl:
        pp.append(opt_param(x, cont))
    return max(pp)

# [0, 0] -- sinus -- Nelder-Mead -- [0.05392127 0.31127394] -- 14.831
# [0, 0] -- sinus -- BFGS, Powell -- [0.60275368 0.14233554] -- 2.3928

# with conversion position sur sin_30_005
# BFGS - [0.22357534, 0.17836804] avec conversion chaque position
# BFGS - [0.21826902, 0.17563941]juste pour plot

# [0, 0] -- carré -- BFGS, Powell -- [0.52422432 0.08500503] -- 5.0794
# [0, 0] -- carré -- Nelder-Mead -- [0.08980634 0.27940342] -- 13.1126
res = minimize(opt_para_more_data, np.array([0.2, 0.2, 0, 0]), method='BFGS', tol=1e-6)
print(res)
print(res.x)
print(res.fun)

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
