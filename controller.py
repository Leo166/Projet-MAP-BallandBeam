import numpy as np
from utils import *

class PIDController:
    def __init__(self, term, time):
        self.term = term
        self.error = []
        self.time = time
        self.control = []
        self.need_error = True

    def add_error(self, e):
        self.error.append(e)

    def add_control(self, a):
        self.control.append(a)

    def get_control(self, *other): #control en radian
        k = self.term
        error = self.error
        u = 0
        u += k[0] * error[-1]
        if len(error) >= 2:
            u += k[1] * sum(error) * self.time + k[2] * (error[-1] - error[-2]) / self.time
        u = convert_angle_experimental(u) #motor angle [°]
        u = np.deg2rad(u)
        if other:
            self.add_control(u)
        return u

    def get_derivative(self):
        return (self.control[-1] - self.control[-2]) / self.time


class ManualController:
    def __init__(self, filename, time):
        self.control, self.position = self.load_manual_control(filename)
        self.time = time
        self.count = 0
        self.need_error = False
        self.last_u = 0

    def counter(self):
        self.count += 1

    def get_control(self, *other):
        c = self.control[self.count]
        # if abs(np.rad2deg(self.control[self.count+1]) - np.rad2deg(self.control[self.count])) >= 10:
        #     c /= 2
        #     print("ok")
        if other:
            self.counter()
        return c

    def get_derivative(self):
        return (self.control[self.count] - self.control[self.count-1]) / self.time

    def load_manual_control(self, filename): #u vecteur de toutes les commandes effectués, données par labview per exemple
        cmd = []
        pos = []
        with open(filename, "r") as f:
            n = 0
            for line in f:
                if n > 2:
                    lgn = line.split()
                    # print(lgn)
                    col1 = lgn[1].split("E")
                    col2 = lgn[2].split("E")
                    com = float((col1[0]).replace(',', '.'))*10**float(col1[1]) #commande en degré
                    posm = float((col2[0]).replace(',', '.'))*10**float(col2[1]) #position en centimètre
                    # com -= 7.5
                    com = convert_angle_experimental(com)
                    com = np.deg2rad(com)
                    cmd.append(com)
                    pos.append(posm)
                n += 1
        return np.array(cmd), np.array(pos)
