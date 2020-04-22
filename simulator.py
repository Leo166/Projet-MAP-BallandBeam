import numpy as np
from scipy import *

from matplotlib import pyplot as plt
from scipy.optimize import minimize

from dynamical_system import *
from utils import *
from controller import *
from optimize import *

class Simulation:
    def __init__(self, system, controller, set_point, ic, bc):
        self.time = 0
        self.step = 0
        self.system = system
        self.controller = controller
        self.set_point = set_point
        self.ic = ic
        self.bc = bc
        self.run = True

    def start_simulation(self):
        self.run = True
        xt = self.ic
        xs = self.set_point
        # sequence = False
        # if isinstance(xs, list) or isinstance(xs, np.ndarray):
        #     sequence = True

        #vérfier le conditions initials pid-manual projection(self.controller.control[0], self.ic[0], False)
        self.system.add_data([self.ic[0], self.ic[1]])
        self.system.current_ctrl = ic[2]

        while self.run:
            if self.controller.need_error:
                # if sequence:
                if self.step == len(self.set_point)-1:
                    self.break_simulation()
                xs = self.set_point[self.step]
                self.controller.add_error(xs - xt[0])
            else:
                if len(system.position) == len(controller.control)-1:
                    self.break_simulation()

            xt = self.system.get_solution(xt)
            self.time += dt
            self.step += 1
        return xt

    def break_simulation(self):
        self.run = False
        return

    def stop_simulation(self):
        return


# time_simulation = 90   # duration of the simulation [s]
set_point = np.ones(400)*10   # point where the ball has to be stabilized [m]
# set_point = get_setpoints("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/document/datafromothers/Group5-Data_Exp/pid/square_slow.txt")
ic = [0, 0]            # initial position and speed [m]
bc = [-38.15, 38.15]    #[-34.1, 38.15]

#pid parameter
# [-36, -80, -4.05]
# [-6, -2, -5]

controller = PIDController([-3, -0.4, -1], dt)
# controller = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/document/datafromothers/Group5-Data_Exp/data2.txt", dt)
# controller = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/sinus/sin_20_005.txt", dt)
# controller = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/sinus/sin_30_005.txt", dt)
# controller = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/sinus/sin_40_005.txt", dt)
# controller = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/squarred/square_20_20.txt", dt)
# controller = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/squarred/square_30_20.txt", dt)
# controller = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/squarred/square_40_20.txt", dt)
# controller = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/free_control/test2.txt", dt)


#Francois Wielant
# controller = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/FW-Data_Exp/Data_TestCL_sine_A20cm_P50_1.txt.txt", dt)
# controller = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/FW-Data_Exp/Data_TestCL_Step_1.txt.txt", dt)

system = DynamicalSystem(controller, bc, idiot_proof, speed_limit)
# ic = [get_initial_value(controller.position[0], controller.control[0]), 0]            # initial position and speed [m]
simulation = Simulation(system, controller, set_point, ic, bc)
simulation.start_simulation()

position = system.position
velocity = system.velocity
acceleration = system.acceleration
command = controller.control
# positionsys = controller.position


t = np.arange(0.0, simulation.time, dt)
print("temps", len(t))
print("step :", simulation.step)
print("position :", len(position))
# print("positionsys", len(positionsys))
print("setpoint", len(set_point))
print("cmd", len(command))
# graphique([t], [position], [velocity], [positionsys], [command])         #manual
graphique([t[0:-1]], [position[0:-1]], [velocity[0:-1]], [], [command])    #pid
breakpoint()

# ctrl = [controller, controller1, controller2, controller3, controller4, controller5]
# ctrl = [controller, controller1, controller2, controller3]
# n = 0
# def opt_para_more_data(x):
#     pp = []
#     for cont in ctrl:
#         pp.append(opt_param(x, cont))
#     return max(pp)
# res = minimize(opt_para_more_data, np.array([0.2, 0.2, 0, 0]), method='BFGS', tol=1e-6)
# print(res.x)
# print(res.fun)


#Objective 3
# x_init est la condition initiale ([cm] avec point de référence au milieu du “beam”)
def idiot_proof_test(x_init, u, flag_idiot_proof):
    controller = ManualController(u, dt)
    system = DynamicalSystem(controller, bc, flag_idiot_proof, None)
    simulation = Simulation(system, controller, x_init, ic, bc)
    return simulation.start_simulation()

#Objective 4, 5
def multiple_positions(x_init, x_desired):
    controller = PIDController([-10, 0, 0], dt)
    system = DynamicalSystem(controller, bc, flag_idiot_proof, None)
    simulation = Simulation(system, controller, x_desired, ic, bc)
    return simulation.start_simulation()

#Objective 6
# def multiple_positions_avec_contraintes(x, t, v_max, v_min):
#
#     return ;