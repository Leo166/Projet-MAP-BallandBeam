"""
Mathématique appliqué - Projet4 - 2019_2020
Ball and Beam
Version 1.0
"""

import numpy as np
from scipy import *

from matplotlib import pyplot as plt
from scipy.optimize import minimize

from dynamical_system import *


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
        self.system.add_data([projection(0, self.ic[0], False), self.ic[1]])
        # self.system.add_data([self.ic[0], self.ic[1]])
        self.system.current_ctrl = self.ic[2]

        while self.run:
            if self.controller.need_error:
                # if sequence:
                if self.step == len(self.set_point)-1:
                    self.break_simulation()
                xs = self.set_point[self.step]
                # self.controller.add_error(xs - xt[0])
                self.controller.add_error(projection(self.system.current_ctrl, xs, False) - projection(self.system.current_ctrl, xt[0], False)) #le setpoint est déjà la position pour la caméra
            else:
                if len(self.system.position) == len(self.controller.control)-1:
                    self.break_simulation()

            xt = self.system.get_solution(xt)
            self.time += dt
            self.step += 1
        return self.system.position

    def break_simulation(self):
        self.run = False
        return

    def stop_simulation(self):
        return


# time_simulation = 90   # duration of the simulation [s]
# set_point = get_trajectory("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/FW-Data_Exp/TestCL_sine_A20cm_P50.txt")
set_point = np.ones(400)*25   # point where the ball has to be stabilized [m]
# set_point = get_setpoints("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/document/datafromothers/Group5-Data_Exp/pid/square_slow.txt")
ic = [-10, 0, 0]            # initial position and speed [m]
bc = [-38.15, 38.15]    #[-34.1, 38.15]

#pid parameter
# [-36, -80, -4.05]
# [-3, -0.1, -1.2]
#[-8,  -6,  -3.2]

# ku = -17
# tu = 1.35
# controller = PIDController([0.6*ku,  1.2*ku/tu,  3*tu*ku/40], dt) #Ziegler–Nichols method
# controller = PIDController([0.2*ku,  -0.5*tu,  -0.333*tu], dt) #Chau method
# controller = PIDController([-3, -0.5, 0], dt)
# controller = INLSEF(dt)

# controller = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/simulateur_ball_and_beam/simulateur/raw_data/OpenLoop/sinus_30_05.txt", dt)

# controller = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/document/datafromothers/Group5-Data_Exp/data2.txt", dt)
# controller = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/sinus/sin_20_005.txt", dt)
# controller = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/sinus/sin_30_005.txt", dt)
# controller = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/sinus/sin_40_005.txt", dt)
# controller = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/squarred/square_20_20.txt", dt)
# controller = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/squarred/square_30_20.txt", dt)
# controller = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/squarred/square_40_20.txt", dt)
# controller = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/free_control/test2.txt", dt)


#Francois Wielant
# controller0 = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/FW-Data_Exp/Data_TestCL_3_sines_1.txt.txt", dt) #ok90
# controller0 = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/FW-Data_Exp/Data_TestCL_3_sines_2.txt.txt", dt) #90
# controller0 = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/FW-Data_Exp/Data_TestOL_3_sines_1.txt.txt", dt) #ok
# controller0 = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/FW-Data_Exp/Data_TestOL_3_sines_2.txt.txt", dt)
# controller0 = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/FW-Data_Exp/Data_TestCL_Step_1.txt.txt", dt) #big one
# controller0 = ManualControllerFile("C:/Users/Scorpion/Desktop/cours/Cours-Q6/Projet4/Projet-MAP-BallandBeam/raw_data/FW-Data_Exp/Data_TestCL_sine_A20cm_P50_1.txt.txt", dt)


# system = DynamicalSystem(controller, bc, False)
# ic = [get_initial_value(controller.position[0], controller.control[0]), 0, 0]            # initial position and speed [m]
# simulation = Simulation(system, controller, set_point, ic, bc)
# simulation.start_simulation()
#
# position = system.position
# velocity = system.velocity
# acceleration = system.acceleration
# command = controller.control
# positionsys = controller.position
#
#
# t = np.arange(0.0, simulation.time, dt)
# print("temps", len(t))
# print("step :", simulation.step)
# print("position :", len(position))
# # print("positionsys", len(positionsys))
# print("setpoint", len(set_point))
# print("cmd", len(command))
# # graphique([t], [position], [velocity], [positionsys], [command])         #manual
# graphique([t[1:-1]], [position[1:-1]], [velocity[1:-1]], [positionsys[1:-1]], [command[1:-1]])    #pid
# breakpoint()
def get_index_pos1(pos, x1):
    indposition1 = np.where(pos == x1)
    tol = 1
    if len(indposition1[0]) == 0:
        indposition1 = np.where(abs(x1 - pos) < tol)
    if len(indposition1[0]) == 0:
        return False
    ind = np.searchsorted(pos[indposition1[0]], x1)
    return ind+indposition1[0][0]

def opt_trajectory(ic, speed_limit, x1, x2, time_guess):
    def find_max_time_travel(t2):
        def linear_straight(x):
            a = ic[0]
            b = (x2 - a) / t2
            return a + b * x
        t = np.linspace(0, t2, int(t2/dt))
        pos = linear_straight(t)

        bc = [-38.15, 38.15]
        controller = PIDController([-2, -0.4, -0.8], dt)
        # controller = INLSEF(dt)
        system = DynamicalSystem(controller, bc, False)
        simulation = Simulation(system, controller, pos, ic, bc)
        position = simulation.start_simulation()
        t = np.arange(0.0, simulation.time+dt, dt)
        velocity = system.velocity
        command = controller.control
        # graphique([t[0:len(command)]], [position[0:len(command)]], [velocity[0:len(command)]], [], [command])
        return t2, pos, velocity


    find = False
    traking_time = 0
    index = 0
    while not find:
        time_travel, trajectory, velocity = find_max_time_travel(time_guess)
        index = get_index_pos1(trajectory, x1)
        vel_max = max(velocity)
        vel_min = velocity[index]
        print("neighboord x1", trajectory[index-2:index+2])
        print("x1", trajectory[index])
        # vel_min = fonction_olivier(trajectory, x1)
        if vel_min < speed_limit[0]:
            print("Impossible to find a trajectory with this low bound of speed !")
            if time_travel >= 15:
                return "Impossible to find a trajectory with this low bound of speed !"

        if vel_max > speed_limit[1] or vel_min < speed_limit[0]:  # en dehors des bornes vitesse
            time_guess += 0.2
        else:
            print("Find it !")
            find = True
            traking_time = time_travel
            print(traking_time)

    def linear_straight(x):
        a = ic[0]
        b = (x2 - a) / traking_time
        return a + b * x

    bc = [-38.15, 38.15]
    controller = PIDController([-2, -0.4, -0.8], dt)
    # controller = INLSEF(dt)
    system = DynamicalSystem(controller, bc, False)


    t = np.linspace(0, traking_time, int(traking_time / dt))
    simulation = Simulation(system, controller, linear_straight(t), ic, bc)
    trajectory = simulation.start_simulation()
    t = np.arange(0.0, simulation.time+dt, dt)
    velocity = system.velocity
    command = controller.control
    # graphique([t[0:len(command)]], [trajectory[0:len(command)]], [velocity[0:len(command)]], [], [command])
    plt.subplot(2, 1, 1)
    plt.plot(t[0:-1], np.rad2deg(command), label="Control")
    plt.ylabel('Beam angle [°]')
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.plot(t, trajectory, label="Simulation")
    plt.plot(t, velocity, "--", label="Velocity (simul)")
    print(speed_limit)
    plt.hlines(10, 0, t[-1], 'orange', '--', linewidth=1)
    plt.hlines(4, 0, t[-1], 'orange', '--', linewidth=1)
    plt.vlines(t[index], -38.5, 38.5, 'red', '--', linewidth=1)
    plt.show()
    return trajectory


t2 = 0.1
opt_trajectory([-20, 0, 0], [5, 10], -1, 0.0, t2)
#Optimisation paramètre
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
# def idiot_proof_test(x_init, u, flag_idiot_proof):
#     controller = ManualController(u, dt)
#     system = DynamicalSystem(controller, bc, flag_idiot_proof, None)
#     simulation = Simulation(system, controller, 0, ic, bc) #0= setpoint unused
#     return simulation.start_simulation()

def idiot_proof_test(x_init, u, flag_idiot_proof):
    controller = ManualController(u, dt)
    system = DynamicalSystem(controller, bc, flag_idiot_proof)
    simulation = Simulation(system, controller, 0, x_init, bc)
    simulation.start_simulation()
    t = np.arange(0.0, simulation.time, dt)
    position = system.position
    velocity = system.velocity
    acceleration = system.acceleration
    command = controller.control
    graphique([t], [position], [velocity], [], [system.controlmdf])
    return

# u = get_control_vector("C:/Users/Scorpion/Desktop/echelon_1_40_15.txt")
# print(u)
# idiot_proof_test([0, 0, 0], u, True)


#Objective 4, 5
def multiple_positions(x_init, x_desired):
    controller = PIDController([-10, 0, 0], dt)
    system = DynamicalSystem(controller, bc, True)
    simulation = Simulation(system, controller, x_desired, x_init, bc)
    return simulation.start_simulation()

import time
#Objective 6
#x_inti
# x = position [x0, x1, x2]
def multiple_positions_avec_contraintes(x_init, x, v_max, v_min):
    new_x0 = 0
    if abs(x[1]) > 34:
        print("Impossible to find a trajectory without risking touching the edges !!")
        return False
    if x[1] > x[2]:
        new_x0 = 34
    elif x[1] == x[2]:
        print("No trajectory is possible because x1 equals x2")
        return False
    elif x[1] < x[2]:
        new_x0 = -34

    # controller = PIDController([-2, -0.4, -0.8], dt)
    controller = INLSEF(dt)
    system = DynamicalSystem(controller, bc, True)
    simulation = Simulation(system, controller, np.ones(200)*new_x0, x_init, bc)
    trajectory_to_x0 = simulation.start_simulation()

    start_time = time.time()
    trajectory = opt_trajectory([new_x0, 0, 0], [v_min, v_max], x[1], x[2], 0.1)
    interval = time.time() - start_time
    print('Total time in seconds:', interval)
    controller = PIDController([-2, -0.4, -0.8], dt)
    system = DynamicalSystem(controller, bc, True)
    simulation = Simulation(system, controller, trajectory, [new_x0, 0, 0], bc)
    trajectory_to_x2 = simulation.start_simulation()
    all_trajectory = trajectory_to_x0.append(trajectory_to_x2)
    return all_trajectory

