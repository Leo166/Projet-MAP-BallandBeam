import numpy as np
from matplotlib import pyplot as plt

"""Tools used in the simulator"""

"""
Convert the angle of the motor to the angle of the beam (theoretical)
@param float; al;  angle of the motor in degree !
@param boolean; dg;  True beta is in degree False otherwise (default is True)
@return angle of the beam in degree !
"""
def convert_angle(al, dg=True):
    if dg:
        al = al * np.pi / 180
    # Position of differents points
    Ax = 2.05
    Ay = 0.0

    Dx = 0.0
    Dy = 0.90
    
    # Length of radius
    M = 0.9
    L = 1.5
    
    R = 0.55 # radius of the ball

    # Calculation of B coordinates
    Bx = Ax - R * np.cos(al)
    By = Ay + R * np.sin(al)   
    # Thanks to equations of circles centered in D with radius L and centered in B 
    # with radius M we can find an expression of x in term of y : x=E-Fy 
    E = (Bx ** 2 - Dx ** 2 + By ** 2 - Dy ** 2 + L ** 2 - M ** 2) / (2 * (Bx - Dx))
    F = (By - Dy) / (Bx - Dx)
    # By injecting x in an equation of circle with L radius here, we obtain a function 
    # of the second degree that we are going to resolve
    b = (2 * Dx * F) - (2 * E * F) - (2 * Dy)
    a = (F ** 2) + 1
    c = (E ** 2) - (2 * Dx * E) + Dx ** 2 + Dy ** 2 - L ** 2
    delta = b ** 2 - (4 * a * c)
    y = (-1 * b + np.sqrt(delta)) / (2 * a)
    # Calculation of the beam angle + conversion to degree
    beta = np.arcsin((y - Dy) / L)
    beta = beta * 180 / np.pi
    return beta


"""
Convert the motor angle to the corresponding beam angle
@param float; a;  angle of the motor in degree !
@return angle of the beam in degree !
"""
def convert_angle_experimental(a):
    # correction = 0
    # a += correction
    coef = [2.87661110e-04,  2.72469099e-01, - 2.05888233e+00]
    return coef[0]*a**2 + coef[1]*a + coef[2]


"""
Convert the beam angle to the corresponding motor angle
@param float; beta;  angle of the beam in degree !
@return angle of the motor in degree !
 """
def convert_angle_experimental_inverse(a):
    coef = [-0.01465599, 3.6137046, 7.53448346]
    return coef[0] * a ** 2 + coef[1] * a + coef[2]



# def correction_motor_control(cmd):
#     control = []
#     deriv = [0]
#     acc = [0]
#     prev_control = 0
#     derivation_limit = np.deg2rad(0.1)
#     acc_limit = np.deg2rad(1000)
#     for count, i in enumerate(cmd):
#         current_control = cmd[count]
#         # if prev_control != 0:
#         derivation = (current_control - prev_control)
#         deriv.append(derivation)
#         acceleration = (deriv[-1] - deriv[-2])
#         acc.append(acceleration)
#         if abs(acceleration) > acc_limit:
#             if acceleration > 0:
#                 d = deriv[-2] + acc_limit
#             else:
#                 d = deriv[-2] - acc_limit
#             deriv[-1] = d
#             derivation = d
#         if abs(derivation) > derivation_limit:
#             if derivation > 0:
#                 current_control = prev_control + derivation_limit
#             else:
#                 current_control = prev_control - derivation_limit
#         control.append(current_control)
#         prev_control = current_control
#     return control, deriv, acc


# def projection(beta, r_reel, dg=True): #vraie
#     h = 0.70 * 10 ** 2  # hauteur de la caméra [cm]
#     d = -0.022 * 10 ** 2  # décalage du 0 de la caméra par rapport au centre de la beam
#     c = 0.035 * 10 ** 2  # hauteur du centre de la balle par rapport au centre de rotation de la beam
#     lam = -np.arctan(d / h)  # angle de décalage
#
#     # Convertir beta
#     if dg:
#         beta = beta * np.pi / 180
#
#     # Origine est le centre de rotation de la beam
#     Bx = d * np.sin(beta) + (r_reel * np.cos(beta))  # position du centre de la balle
#     By = -d * np.cos(beta) + (r_reel * np.sin(beta))  # position du centre de la balle
#     Cy = c + h  # position de la caméra
#
#     # calcul de gamma centré(angle 0 à la verticale)
#     gamma_centre = np.arctan(Bx / (Cy - By))
#     # calcul de gamma (angle réel de la caméra (centre décalé))
#     gamma = gamma_centre - lam
#
#     r_ordi = (gamma * 1.306482 * 180 + 4.159161 * np.pi) / np.pi
#     return r_ordi


"""
Convert the real position on the beam to the position seen by the camera
@param float; beta;  angle of the beam in radian or degree
@param float; actual_pos; real position of the ball from the center of the beam
@param boolean; dg; True if beta is in degree False otherwise
@return position of the ball seen by the camera 
 """
def projection(beta, actual_pos, dg=True):
    h = 0.70 * 10 ** 2  # camera height [cm]
    d = -0.022 * 10 ** 2  # shift between 0 camera and center of the beam
    c = 0.035 * 10 ** 2  # height of the centre of the ball relative to the centre of rotation of the beam
    lam = -np.arctan(d / h)  # offset angle
    # Convert beta
    if dg:
       beta = beta * np.pi / 180
    # Origin is the centre of rotation of the beam
    # Position of the centre of the ball
    Bx = - c * np.sin(beta) + (actual_pos * np.cos(beta))  
    By = c * np.cos(beta) + (actual_pos * np.sin(beta))  
    # Position of the camera
    Cy = c + h  
    # calculation of gamma center (0 angle to the vertical)
    gamma_centre = np.arctan(Bx / (Cy - By))
    # calculation of gamma (real camera angle (center shifted))
    gamma = gamma_centre - lam
    # calculation of the position of the ball seen by the camera
    camera_pos = (gamma * 1.283 * 180 + 4.1 * np.pi) / np.pi
    return camera_pos

# def projection_inverse(beta, r_ordi, dg=True): #vraie
#     h = 0.70 * 10 ** 2  # hauteur de la caméra [cm]
#     d = -0.022 * 10 ** 2  # décalage du 0 de la caméra par rapport au centre de la beam
#     c = 0.035 * 10 ** 2  # hauteur du centre de la balle par rapport au centre de rotation de la beam
#     lam = -np.arctan(d / h)  # angle de décalage
#
#     # Convertir beta
#     if dg:
#         beta = beta * np.pi / 180
#
#     # Calcul de l'angle de la caméra grâce à la fonction d'interpolation
#     gamma_theorique = (r_ordi - 4.159161) * np.pi / (1.306482 * 180)
#     # gamma centré (centre de la beam)
#     gamma_c = gamma_theorique + lam
#
#     # Calcul de la position de la balle dans la beam
#     r_theorique = ((np.tan(gamma_c) * (c + h - (d * np.cos(beta)))) + (d * np.sin(beta))) / (
#                 np.cos(beta) + (np.tan(gamma_c) * np.sin(beta)))
#     # conditions des limites de la beam
#
#     return r_theorique


"""
Convert the position seen by the camera to the real position on the beam
@param float; beta;  angle of the beam in radian or degree
@param float; r_ordi; position of the ball seen by the camera
@param boolean; dg; True if beta is in degree False otherwise
@return real position of the ball from the center of the beam
 """
def projection_inverse(beta, r_ordi, dg=True):
    h = 0.70 * 10 ** 2  # camera height [cm]
    d = -0.022 * 10 ** 2  # shift between 0 camera and center of the beam
    c = 0.035 * 10 ** 2  # height of the centre of the ball relative to the centre of rotation of the beam
    lam = -np.arctan(d / h)  # offset angle
    # Convert beta
    if dg:
       beta = beta * np.pi / 180
    # Calculation of the camera viewing angle thanks to the interpolation function
    gamma_theorique = (r_ordi - 4.1) * np.pi / (1.283 * 180)
    # Calculation of gamma center (0 angle to the vertical)
    gamma_c = gamma_theorique + lam
    # Calculation of the actual position of the ball
    actual_pos = ((np.tan(gamma_c) * (c + h - (c * np.cos(beta)))) + (c * np.sin(beta))) / (
                np.cos(beta) + (np.tan(gamma_c) * np.sin(beta)))
    return actual_pos


"""
Extract the controls of file. One control (motor control [°]) per lign
@param string; filename;  name of the corresponding file
@return (motor) controls
 """
def get_control_vector(filename):
    cmd = []
    with open(filename, "r") as f:
        n = 0
        for line in f:
            if n > 2:
                lgn = line.split()
                cmd.append(float(lgn[0].replace(',', '.')))
            n += 1
    return np.array(cmd)


"""
Use to plot graphics of the position, velocity and control of the dynamical system.
Each array can contain many arrays of position in order to do more graphics.
@param array; t;  time of the simulation
@param array; position; position of the ball from the simulation
@param array; velocity; velocity of the ball from the simulation
@param array; posys; in the case of ManualControlFile, there is the position seen by the camera
@param array; cmd; control from the simulation or the system
@return /
 """
def graphique(t, pos, vel, posys, cmd):
    for count, i in enumerate(t):
        plt.subplot(2, 1, 1)
        # c = correction_motor_control(cmd[count])
        plt.plot(t[count], convert_angle_experimental_inverse(np.rad2deg(cmd[count])), label="Control")
        # plt.plot(t[count], c[0][0:len(c[0])], label="Control")
        # plt.plot(t[count], c[1][0:len(c[0])], label="Control speed")
        # plt.plot(t[count], c[2][0:len(c[0])], label="Control acc")
        # plt.scatter(t[count], correction_motor_control(cmd[count]), s = 5, c='red')
        # plt.title('A tale of 2 subplots')
        plt.ylabel('Motor angle [°]')
        plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(t[count], pos[count], label="Simulation")
        # plt.scatter(5, 15)
        # plt.plot(t[count], vel[count], "--", label="Velocity (simul)")
        # plt.hlines(5, 0, t[0][-1], 'orange', '--', linewidth=1)
        # plt.hlines(-5, 0, t[0][-1], 'orange', '--', linewidth=1)
        # plt.plot(t[count], acc[count], "--", label="acceleration (simul)")
        # for c, j in enumerate(acc[count]):
        #     if j <= 0.1 and j >= -0.1:
        #         plt.scatter(t[count][c], j, s=5)
        if len(posys) != 0:
            plt.plot(t[count], posys[count], label="Raw data")
        # plt.hlines(projection(convert_angle_experimental(0), -10), 0, t[0][-1], 'black', '--', linewidth=1)
        # plt.hlines(0, 0, t[0][-1], 'black', '--', linewidth=1)
        plt.legend()

        plt.xlabel('Time [s]')
        plt.ylabel('Position [cm]')
        plt.show()


# Find the true initial value by having that of the camera so that projection gives it
# x camera intial value [cm], ctrl_initial[rad]
def get_initial_value(x, ctrl_initial):
    x_sys = x*8/10   #first guess
    x_cam = projection(ctrl_initial, x_sys, False)
    while abs(x-x_cam) >= 0.01:
        # print("top")
        if x_cam > x:
            x_sys -= 0.001
        else:
            x_sys += 0.001
        x_cam = projection(ctrl_initial, x_sys, False)
    return x_sys


##############################
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

# opt_trajectory([0, 0, 0], np.ones(400)*10)
