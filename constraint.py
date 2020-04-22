import numpy as np


def idiot_proofing(system, controller, active=True):  # FONCTION IDIOT PROOFING (GERE LA VITESSE DE LA BALL QUAND ELLE SE RAPPROCHE DES BORDS)
    if active:
        if (abs(system.position[-1]) > 20):
            v_max = 1 / system.position[-1]

            if (system.velocity[-1] > v_max):
                a = abs(10 / system.position[
                    -1])  # en fonction de la position, pcq definir a en fonction de la v_max est inutile, puisque v_max depend de la position
                controller.control[-1] = a * controller.control[-1]


# #-------------------------------------------------- ##################################################### PARTIE DE L'IDIOT PROOFING FAIT AVEC OLIVIER
#                                                     ############### MAIS QUE J'AI MODIFIE (A PRIORI INUTILE PUISQU'ON DETERMINE LE SETPOINT NOUS MEME)
# R = 15*10**-3       # ball radius [m]
# if (set_point > 38.15 - R) or (set_point < -34.1 + R) : #si le setpoint est en dehors du beam, on le met à 0 par défaut (ou autre, à voir ce qui a
#     set_point = 0                                       #finalement été décidé dans le rapport