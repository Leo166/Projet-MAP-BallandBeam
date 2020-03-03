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
            # ut = commandmanu[n]
            # ut -= 6.5
            # ut = convert_angle(ut)
            # ut = np.deg2rad(ut)
            xt = self.system.get_solution(xt)
            t += dt
        return xt

    def break_simulation(self):
        self.run = False
        return

    def stop_simulation(self):
        return


time_simulation = 15   # duration of the simulation [s]
set_point = 15*10**-2  # point where the ball has to be stabilized [m]
ic = [0, 0]            # initial position and speed [m]
bc = [-30.35*10**-2, 35.46*10**-2]

controller = PIDController([-1, 0, 0], dt)
system = DynamicalSystem(controller, bc)
simulation = Simulation(time_simulation, system, controller, set_point, ic, bc)
simulation.start_simulation()

position = system.position
vitesse = system.velocity
command = controller.control
print(np.array(command))


t = np.arange(0.0, time_simulation+2*dt, dt)
# plt.plot(t, np.array(positionsys*100), label="System [cm]")
plt.plot(t[0:-1], np.array(position)*100, "--", label="Simulation [cm]")
plt.plot(t[0:-1], np.array(vitesse)*100, label="Vitesse", color= "green")
# plt.plot(t[0:-1], np.array(command), label="Commande")
# plt.plot(t, commandmanu, label="Motor control [°]")
# plt.plot(t[0:-2], np.rad2deg(command[1::]), label="Commande")
plt.hlines(set_point*100, 0, time_simulation, 'black', '--', linewidth=1)
plt.xlabel("Time [s]")
# plt.ylabel("Position [cm]")
plt.legend()
plt.show()
