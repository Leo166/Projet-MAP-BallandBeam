import numpy as np
from matplotlib import pyplot as plt

"""Tools used in the simulator"""

# Convert the angle of the motor to the angle of the beam
def convert_angle(al, dg=True):
    if dg:
        al = al * np.pi / 180
    Ax = 2.05
    Ay = 0.0

    Dx = 0.0
    Dy = 0.90

    R = 0.55
    M = 0.9
    L = 1.5
    # Calcul des coordonnees de B
    Bx = Ax - R * np.cos(al)
    By = Ay + R * np.sin(al)   #+ avant
    E = (Bx ** 2 - Dx ** 2 + By ** 2 - Dy ** 2 + L ** 2 - M ** 2) / (2 * (Bx - Dx))
    F = (By - Dy) / (Bx - Dx)
    b = (2 * Dx * F) - (2 * E * F) - (2 * Dy)
    a = (F ** 2) + 1
    c = (E ** 2) - (2 * Dx * E) + Dx ** 2 + Dy ** 2 - L ** 2
    delta = b ** 2 - (4 * a * c)
    y = (-1 * b + np.sqrt(delta)) / (2 * a)
    beta = np.arcsin((y - Dy) / L)
    beta = beta * 180 / np.pi
    return beta

#le décalage d'angle entre la beam et le moteur est pris en compte dans la conversion et donc dans le système. Par conséquent, pas besoin de rajouter +-7.5
def convert_angle_experimental(a):
    coef = [2.87661110e-04,  2.72469099e-01, - 2.05888233e+00]
    return coef[0]*a**2 + coef[1]*a + coef[2]

dt = 50*10**-3
def correction_motor_control(cmd):
    control = []
    deriv = [0]
    acc = [0]
    prev_control = 0
    derivation_limit = np.deg2rad(0.1)
    acc_limit = np.deg2rad(1000)
    for count, i in enumerate(cmd):
        current_control = cmd[count]
        # if prev_control != 0:
        derivation = (current_control - prev_control)
        deriv.append(derivation)
        acceleration = (deriv[-1] - deriv[-2])
        acc.append(acceleration)
        if abs(acceleration) > acc_limit:
            if acceleration > 0:
                d = deriv[-2] + acc_limit
            else:
                d = deriv[-2] - acc_limit
            deriv[-1] = d
            derivation = d

        if abs(derivation) > derivation_limit:
            if derivation > 0:
                current_control = prev_control + derivation_limit
            else:
                current_control = prev_control - derivation_limit
        control.append(current_control)
        prev_control = current_control
    return control, deriv, acc

def correction_low_speed(speed):
    new_speed = speed
    su = 0.5
    # c = np.sqrt(np.abs(speed))
    c = (speed**2)
    if abs(speed) <= su:
        if speed > 0:
            new_speed = c
        elif speed < 0:
            new_speed = -c
    return new_speed


def projection(beta, r_reel, dg=True):
    h = 0.70 * 10 ** 2  # hauteur de la caméra [cm]
    d = -0.022 * 10 ** 2  # décalage du 0 de la caméra par rapport au centre de la beam
    c = 0.035 * 10 ** 2  # hauteur du centre de la balle par rapport au centre de rotation de la beam
    lam = -np.arctan(d / h)  # angle de décalage

    # Convertir beta
    if dg:
        beta = beta * np.pi / 180

    # Origine est le centre de rotation de la beam
    Bx = d * np.sin(beta) + (r_reel * np.cos(beta))  # position du centre de la balle
    By = -d * np.cos(beta) + (r_reel * np.sin(beta))  # position du centre de la balle
    Cy = c + h  # position de la caméra

    # calcul de gamma centré(angle 0 à la verticale)
    gamma_centre = np.arctan(Bx / (Cy - By))
    # calcul de gamma (angle réel de la caméra (centre décalé))
    gamma = gamma_centre - lam

    r_ordi = (gamma * 1.306482 * 180 + 4.159161 * np.pi) / np.pi
    return r_ordi


def projection_inverse(beta, r_ordi, dg=True):
    h = 0.70 * 10 ** 2  # hauteur de la caméra [cm]
    d = -0.022 * 10 ** 2  # décalage du 0 de la caméra par rapport au centre de la beam
    c = 0.035 * 10 ** 2  # hauteur du centre de la balle par rapport au centre de rotation de la beam
    lam = -np.arctan(d / h)  # angle de décalage

    # Convertir beta
    if dg:
        beta = beta * np.pi / 180

    # Calcul de l'angle de la caméra grâce à la fonction d'interpolation
    gamma_theorique = (r_ordi - 4.159161) * np.pi / (1.306482 * 180)
    # gamma centré (centre de la beam)
    gamma_c = gamma_theorique + lam

    # Calcul de la position de la balle dans la beam
    r_theorique = ((np.tan(gamma_c) * (c + h - (d * np.cos(beta)))) + (d * np.sin(beta))) / (
                np.cos(beta) + (np.tan(gamma_c) * np.sin(beta)))
    # conditions des limites de la beam

    return r_theorique

def get_setpoints(filename):
    pos = []
    with open(filename, "r") as f:
        n = 0
        for line in f:
            if n > 2:
                lgn = line.split()
                pos.append(float(lgn[0].replace(',', '.')))
            n += 1
    return np.array(pos)

def graphique(t, pos, vel, posys, cmd):
    for count, i in enumerate(t):
        plt.subplot(2, 1, 1)
        plt.plot(t[count], np.rad2deg(cmd[count]), label="Control")
        c = correction_motor_control(cmd[count])
        # plt.plot(t[count], c[0][0:len(c[0])], label="Control")
        # plt.plot(t[count], c[1][0:len(c[0])], label="Control speed")
        # plt.plot(t[count], c[2][0:len(c[0])], label="Control acc")
        # plt.scatter(t[count], correction_motor_control(cmd[count]), s = 5, c='red')
        # plt.title('A tale of 2 subplots')
        plt.ylabel('Beam angle [°]')
        # plt.ylabel('Position [cm]')
        plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(t[count], pos[count], label="Position (simul)")
        # plt.plot(t[count], vel[count], "--", label="Velocity (simul)")
        # plt.plot(t[count], acc[count], "--", label="acceleration (simul)")
        # for c, j in enumerate(acc[count]):
        #     if j <= 0.1 and j >= -0.1:
        #         plt.scatter(t[count][c], j, s=5)
        if len(posys) != 0:
            plt.plot(t[count], posys[count], label="Position")
        plt.hlines(projection(convert_angle_experimental(0), 3), 0, t[0][-1], 'black', '--', linewidth=1)
        # plt.hlines(projection(convert_angle_experimental(0), -20), 0, t[0][-1], 'black', '--', linewidth=1)
        plt.legend()

        plt.xlabel('Time [s]')
        plt.ylabel('Position [cm]')
        plt.show()
