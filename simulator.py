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
        self.system.add_data([projection(0, self.ic[0], False), self.ic[1]])
        # self.system.add_data([self.ic[0], self.ic[1]])
        self.system.current_ctrl = self.ic[2]

        while self.run:
            if self.controller.need_error:
                # if sequence:
                if self.step == len(self.set_point)-1:
                    self.break_simulation()
                xs = self.set_point[self.step]
                self.controller.add_error(xs - xt[0])
                # self.controller.add_error(projection(self.system.current_ctrl, xs, False) - projection(self.system.current_ctrl, xt[0], False)) #le setpoint est déjà la position pour la caméra
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


def get_trajectory(filename):
    x = []
    with open(filename) as f:
        for line in f:
            line = line.strip().replace(',', '.')
            if len(line) == 0:
                continue
            tmp = list(map(float, line.split()))
            x.append(tmp[0])
    return np.array(x)


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
#
#
# system = DynamicalSystem(controller, bc, False, None)
# ic = [get_initial_value(controller.position[0], controller.control[0]), 0, 0]            # initial position and speed [m]
# simulation = Simulation(system, controller, set_point, ic, bc)
# simulation.start_simulation()

def opt_trajectory(ic, speed_limit, x1, x2, time_guess):
    def find_max_time_travel(t2):
        def linear_straight(x):
            a = ic[0]
            b = (x2 - a) / t2
            return a + b * x
        t = np.linspace(0, t2, int(t2/dt))
        pos = linear_straight(t)
        f = plt.figure(figsize=(10, 4))
        ax1 = f.add_subplot(111)

        # ax1.plot(t, pos, label='fit')
        # ax1.legend()
        # plt.show()
        controller = PIDController([-2, -0.4, -0.8], dt)
        # controller = INLSEF(dt)
        system = DynamicalSystem(controller, bc, False, None)
        simulation = Simulation(system, controller, pos, ic, bc)
        position = simulation.start_simulation()
        t = np.arange(0.0, simulation.time+dt, dt)
        velocity = system.velocity
        command = controller.control
        graphique([t[0:-1]], [position[0:-1]], [velocity[0:-1]], [], [command])
        return t2, max(system.velocity), min(system.velocity)

    out_speed = False
    in_speed = False
    find = False

    time2, vel_max, vel_min = find_max_time_travel(time_guess)

    if vel_min < speed_limit[0]:
        print("Impossible to find a trajectory with this low bound of speed !")

    #Initialisation de connaissance état
    # if vel_max > speed_limit[1] or vel_min < speed_limit[0]: #en dehors des bornes vitesse
    #     out_speed = True
    # else:
    #     in_speed = True

    traking_time = 0
    while not find:
        time_travel, vel_max, vel_min = find_max_time_travel(time_guess)
        if vel_max > speed_limit[1] or vel_min < speed_limit[0]:  # en dehors des bornes vitesse
            time_guess += 0.2
            traking_time = time_travel
        else:
            find = True
            traking_time = time_travel


    # controller = PIDController([-3, -0.1, -1.2], dt)
    # controller = INLSEF(dt)
    system = DynamicalSystem(controller, bc, False, None)
    simulation = Simulation(system, controller, linear_straight(t), ic, bc)
    position = simulation.start_simulation()
    t = np.arange(0.0, simulation.time, dt)
    velocity = system.velocity
    command = controller.control
    graphique([t[0:-1]], [position[0:-1]], [velocity[0:-1]], [], [command])
    # graphique([t], [position[0:-1]], [velocity[0:-1]], [], [command])
    exit()
    return trajectory

t2 = 5
opt_trajectory([-20, 0, 0], [-4, 4], -10, 0.0, t2)



# def opt_trajectory(ic, setpoint):
#     def find_best_trajectory(k):
#         bc = [-38.15, 38.15]
#         controller = PIDController([k[0], k[1], k[2]], dt)
#         system = DynamicalSystem(controller, bc, False, None)
#         simulation = Simulation(system, controller, setpoint, ic, bc)
#         position = simulation.start_simulation()
#         erreur = np.mean(abs(position[1::] - setpoint))
#         print("Erreur", erreur)
#         return erreur
#     res = minimize(find_best_trajectory, np.array([-1, -1, -1]), method='BFGS', tol=1e-2, bounds=((None, 0), (None, 0)))
#     print(res)
#     exit()
#     return np.array(res.x)
#
# opt_trajectory([0, 0, 0], np.ones(400)*10)

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
# graphique([t], [position], [velocity], [positionsys], [command])         #manual
# # graphique([t[0:-1]], [position[0:-1]], [velocity[0:-1]], [], [command])    #pid
# breakpoint()


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
def idiot_proof_test(x_init, u, flag_idiot_proof):
    controller = ManualController(u, dt)
    system = DynamicalSystem(controller, bc, flag_idiot_proof, None)
    simulation = Simulation(system, controller, 0, ic, bc) #0= setpoint unused
    return simulation.start_simulation()

# idiot_proof_test([0, 0, 0], transformer_en_vecteur("C:/Users/Scorpion/Desktop/echelon_1_30_5.txt"), False)


#Objective 4, 5
def multiple_positions(x_init, x_desired):
    controller = PIDController([-10, 0, 0], dt)
    system = DynamicalSystem(controller, bc, True, None)
    simulation = Simulation(system, controller, x_desired, x_init, bc)
    return simulation.start_simulation()

#Objective 6
# def multiple_positions_avec_contraintes(x, t, v_max, v_min):
#
#     return ;