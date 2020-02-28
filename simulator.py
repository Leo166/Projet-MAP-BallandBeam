import numpy as np
from scipy import *
from scipy.integrate import odeint
from matplotlib import pyplot as plt
from dynamical_system import *
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



time_simulation = 15   # duration of the simulation [s]
set_point = 15*10**-2  # point where the ball has to be stabilized [m]
ic = [0, 0]            # initial position and speed [m]
bc = [-30.35*10**-2, 35.46*10**-2]

controller = PIDController([-1, -0.1, -0.4], dt)
system = DynamicalSystem(3*10**-2, controller, bc)
simulation = Simulation(time_simulation, system, controller)