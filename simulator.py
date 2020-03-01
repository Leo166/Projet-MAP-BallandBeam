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
        xs = self.setpoint
        while t < self.time and self.run:  # bad
            if self.controller.need_error:
                self.controller.add_error(xs - xt[0])
            # ut = commandmanu[n]
            # ut -= 6.5
            # ut = convert_angle(ut)
            # ut = np.deg2rad(ut)
            xt = self.system.get_solution()
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

controller = PIDController([-1, -0.1, -0.4], dt)
system = DynamicalSystem(controller, bc)
simulation = Simulation(time_simulation, system, controller, set_point, ic, bc)
