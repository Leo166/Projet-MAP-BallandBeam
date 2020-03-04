import numpy as np
from scipy import *

from matplotlib import pyplot as plt
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
        while t < self.time and self.run:  # bad
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
bc = [-34.1, 38.15]

# controller = PIDController([-1, 0, 0], dt)
controller = ManualController("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/sinus/sin_30_005.txt", time_simulation)
system = DynamicalSystem(controller, bc)
ic = [controller.position[0], 0]            # initial position and speed [m]
simulation = Simulation(time_simulation, system, controller, set_point, ic, bc)
simulation.start_simulation()

position = system.position
vitesse = system.velocity
command = controller.control
print(command)
positionsys = controller.position
# print(np.array(command))


t = np.arange(0.0, time_simulation+2*dt, dt)
plt.plot(t[0:-2], np.array(positionsys[1:len(position)]), label="System [cm]")
plt.plot(t[0:-1], np.array(position), "--", label="Simulation [cm]")
plt.plot(t[0:-1], np.array(vitesse), label="Velocity", color= "green")
# plt.plot(t[0:-2], np.array(command), label="Commande")
# plt.plot(t, commandmanu, label="Motor control [°]")
plt.plot(t[0:-2], np.rad2deg(command[1:len(position)]), label="Control")
plt.hlines(set_point, 0, time_simulation, 'black', '--', linewidth=1)
# plt.hlines(-34.1, 0, time_simulation, 'black', '--', linewidth=1)
plt.xlabel("Time [s]")
# plt.ylabel("Position [cm]")
plt.legend()
plt.show()
