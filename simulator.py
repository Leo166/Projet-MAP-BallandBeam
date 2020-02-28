import numpy as np
from scipy import *
from scipy.integrate import odeint      # Module de résolution des équations différentielles
from matplotlib import pyplot as plt
from utils import *


class Simulation:
    def __init__(self):


class PIDController:
    def __init__(self, term, error, time):
        self.term = term
        self.error = error
        self.time = time

    def add_error(self, e):
        self.error = self.error.append(e)

    def get_control(self):
        k = self.term
        error = self.error
        u = 0
        u += k[0] * error[-1]
        if len(error) >= 2:
            u = k[1] * sum(error) * self.time + k[2]*self.get_derivative()
        return u

    def get_derivative(self):
        error = self.error
        return (error[-1] - error[-2]) / self.time

class ManualController:
    def __init__(self, namefile):
        self.position, self.control = self.get_manual_control(namefile)

    def get_manual_control(namefile): #u vecteur de toutes les commandes effectués, données par labview per exemple
        cmd = []
        pos = []
        with open(namefile, "r") as f:
            n = 0
            for line in f:
                if n > 2:
                    lgn = line.split()
                    col1 = lgn[1].split("E")
                    col2 = lgn[2].split("E")
                    com = float((col1[0]).replace(',', '.'))*10**float(col1[1]) #commande en degré
                    posm = float((col2[0]).replace(',', '.'))*10**float(col2[1]) #position en centimètre
                    cmd.append(com)
                    pos.append(posm*10**-2)
                    # print(lgn[2].split("E")[0])
                n += 1
        return np.array(cmd), np.array(pos)