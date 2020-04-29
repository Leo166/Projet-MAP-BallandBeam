import numpy as np

"""

@param array; x parameters to find
@param object ManualControllerFile; controller;  controller use in for the dynamical system
@return mean squared error for these parameters
 """
def idiot_proofing(position, velocity, v, active=True):  # FONCTION IDIOT PROOFING (GERE LA VITESSE DE LA BALL QUAND ELLE SE RAPPROCHE DES BORDS)
    if (abs(position[-1]) >= 34):   # MOINS RESTIRCTIF, UN PEU BRUSQUE PARFOIS
        p1 = 10
        v_max = p1 / abs(position[-1])
        # print("vmax", v_max)
        # # print(velocity[-1])
        if (abs(velocity[-1]) > v_max):
            p2 = -100
            a = p2*abs(1 / position[-1]) # en fonction de la position, pcq definir a en fonction de la v_max est inutile, puisque v_max depend de la position
            v -= p2*np.deg2rad(np.tan((position[-1]-18*np.sign(position[-1]))*np.pi/80))
        # v = - 3*v #méthode Adrien
        # print("ok")
    return v