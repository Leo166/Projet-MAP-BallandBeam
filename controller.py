from utils import *

class PIDController:
    def __init__(self, term, time):
        self.term = term
        self.error = []
        self.time = time
        self.control = []
        self.need_error = True
        self.count = 0
        self.integration = 0

    def counter(self):
        self.count += 1

    def add_error(self, e):
        self.error.append(e)

    def add_control(self, a):
        self.control.append(a)

    def get_control(self, *other): #control en radian
        alpha = 1
        k = self.term
        error = self.error
        u = 0
        u += k[0] * error[-1]
        new_integration = 0
        if len(error) >= 2:
            new_integration = self.integration + error[-1]
            derivative = (error[-1] - error[-2]) / self.time
            u += k[1] * new_integration * self.time + k[2] * derivative
        update = True
        if u > 50:
            u = 50
            if error[-1] > 0:
                update = False
        elif u < -50:
            u = -50
            if error[-1] < 0:
                update = False
        if update:
            self.integration = new_integration

        u = convert_angle_experimental(u) #motor angle [°]
        u = np.deg2rad(u)
        if other:
            self.add_control(u)
            self.counter()
        return u

    def get_derivative(self):
        return (self.control[-1] - self.control[-2]) / self.time

class ManualController:
    def __init__(self, u, time):
        self.control = np.deg2rad(convert_angle_experimental(u))
        self.time = time
        self.count = 0
        self.need_error = False
        self.last_u = 0

    def counter(self):
        self.count += 1

    def get_control(self, *other):
        c = self.control[self.count]
        if other:
            self.counter()
        return c

    def get_derivative(self):
        return (self.control[self.count] - self.control[self.count-1]) / self.time


class ManualControllerFile:
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
                    com = convert_angle_experimental(com)
                    com = np.deg2rad(com)
                    cmd.append(com)
                    pos.append(posm)
                n += 1
        return np.array(cmd), np.array(pos)


class INLSEF:
    def __init__(self, time, vel_max=8.4):
        self.error = []
        self.time = time
        self.control = []
        self.need_error = True
        self.count = 0
        self.vel_max = vel_max

    def counter(self):
        self.count += 1

    """
    Trouve le paramètre delta à appliquer pour atteindre la vitesse max
    """
    def param_vit_max(self):
        vel_max = self.vel_max
        m = 0.3861
        p = 0.7532
        return m * vel_max + p

    def add_error(self, e):
        self.error.append(e)

    def add_control(self, a):
        self.control.append(a)

    def get_control(self, *other):
        error = np.array(self.error)
        alpha1 = 1.4245
        alpha2 = 0.9303
        alpha3 = 0.01
        mu1 = 2.5266
        mu2 = 0.2970
        mu3 = 10**(-6)
        k11 = 2.2772
        k12 = 1.5979
        k21 = 2.7004
        k22 = 1.5868
        k3 = 0.01
        delta = self.param_vit_max()
        signeE = -np.sign(error[-1])
        u1 = k11+(k12/(1+np.exp(mu1*((error[-1])**2))))
        u1 = u1*((abs(error[-1]))**alpha1)*signeE
        u2 = 0
        if len(error) >= 2:
            dedt = (error[-1]-error[-2])/self.time
            signeDEDT = -np.sign(dedt)
            u2 = (k21+(k22)/(1+np.exp(mu2*(dedt**2))))*((abs(dedt))**alpha2)*signeDEDT
        integral = sum(error) * self.time
        ui = (k3/(1+np.exp(mu3*integral**2))) * (abs(integral)**alpha3)*np.sign(integral)
        terme = 0.2
        uINLSEF = u1+ui+u2-(0.012946+terme)*np.sign(error[-1])
        u = delta*np.tanh(uINLSEF/delta)
        u = np.deg2rad(u)
        if other:
            self.add_control(u)
        return u

    def get_derivative(self):
        return (self.control[-1] - self.control[-2]) / self.time


