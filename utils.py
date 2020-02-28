import numpy as np

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