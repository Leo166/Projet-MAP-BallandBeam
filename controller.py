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

    def get_control(self, *other):
        k = self.term
        error = self.error
        u = 0
        u += k[0] * error[-1]
        if len(error) >= 2:
            u += k[1] * sum(error) * self.time + k[2] * (error[-1] - error[-2]) / self.time
        u = convert_angle(u-6.5)
        u = np.deg2rad(u)
        #print(u)
        if other:
            self.add_control(u)
        return u
        

    def get_derivative(self):
        return (self.control[-1] - self.control[-2]) / self.time

def f(x,alpha,delta):
        if abs(x)>delta:
            return np.sign(x)*abs(x)**alpha
        return delta**(alpha-1) * x

class NLPIDController:
    def __init__(self, time):
        #self.term = term
        self.error = []
        self.time = time
        self.control = []
        self.need_error = True

    

    def add_error(self, e):
        self.error.append(e)

    def add_control(self, a):
        self.control.append(a)

    def get_control(self, *other):
        #k = self.term
        error = self.error

        #0<alpha<1
        alphap=0.3
        alphai=0.1
        alphad=0.5

        #small positive number
        deltap=0.1
        deltai=0.01
        deltad=0.1

        #les K
        kp=-12   #au dessus de 12 ça bug
        ki=-0
        kd=-4

        u = 0
        u += kp * f(error[-1],alphap,deltap) 
        if len(error) >= 2:
            u += ki * f(sum(error) * self.time,alphai, deltai) 
            u += kd * f((error[-1] - error[-2]) / self.time, alphad, deltad)
        u = convert_angle(u-6.5)
        u = np.deg2rad(u)
        #print(u)
        if other:
            self.add_control(u)
        return u

    def get_derivative(self):
        return (self.control[-1] - self.control[-2]) / self.time




class INLSEF:
    def __init__(self, time):
        #self.term = term
        self.error = []
        self.time = time
        self.control = []
        self.need_error = True


    def add_error(self, e):
        self.error.append(e)

    def add_control(self, a):
        self.control.append(a)

    def get_control(self, *other):
        #k = self.term
        error = np.array(self.error)

        error=error*0.01
        alpha1=1.4245
        alpha2=0.9303
        #alpha3=
        mu1=2.5266
        mu2=0.2970
        #mu3=
        k11=2.2772
        k12=1.5979
        k21=2.7004
        k22=1.5868
        #k3=
        delta=0.5072

        signeE=-np.sign(error[-1])
        u1=k11+(k12/(1+np.exp(mu1*((error[-1])**2))))
        u1= u1*((abs(error[-1]))**alpha1)*signeE
        u2=0
        if len(error) >=2:
            dedt=(error[-1]-error[-2])/self.time
            signeDEDT=-np.sign(dedt)
            u2= (k21+(k22)/(1+np.exp(mu2*(dedt**2))))*((abs(dedt))**alpha2)*signeDEDT
        
        ui=0
        #ui=((k3)/(1+exp(mu3*(Flag(1))^2))) * (abs(Flag(1))**alpha3)*(Flag(1)/abs(Flag(1))
        uINLSEF=u1+u2+ui
        u=delta*np.tanh(uINLSEF/delta)


        # u = 0
        # u += k[0] * error[-1]
        # if len(error) >= 2:
        #     u += k[1] * sum(error) * self.time + k[2] * (error[-1] - error[-2]) / self.time
        #u = convert_angle(u-6.5)
        u = np.deg2rad(u)
        print(u)
        if other:
            self.add_control(u)
        return u

    def get_derivative(self):
        return (self.control[-1] - self.control[-2]) / self.time


class SU:
    def __init__(self, time):
        #self.term = term
        self.error = []
        self.time = time
        self.control = []
        self.need_error = True


    def add_error(self, e):
        self.error.append(e)

    def add_control(self, a):
        self.control.append(a)

    def get_control(self, *other):
        #k = self.term
        Kp=1.952
        Kd=1.264
        Ki=0.7533
        emax=28

        k0=0.1112

        error = np.array(self.error)
        if abs(error[-1])<=emax:
            err=error[-1]
        else:
            err=emax*np.sign(error[-1])
            #print(error[-1])
        error[-1]=err
        print("err: ",err)
        k=(np.exp(k0*err)+np.exp(-k0*err))/2
        up=0
        ui=0
        ud=0
        up=Kp*k*err
        if len(error)>=2:
            ui=Ki *k* sum(error) * self.time
            ud=Kd*((err-error[-2])/self.time)

        u=(up+ui+ud)
        print("u: ",u)
        u = convert_angle(u-6.5)
        u = np.deg2rad(u)
        if other:
            self.add_control(u)
        return u

    def get_derivative(self):
        return (self.control[-1] - self.control[-2]) / self.time


class ManualController:
    def __init__(self, namefile, time):
        self.control, self.position = self.load_manual_control(namefile)
        self.time = time
        self.count = 0
        self.need_error = False

    def counter(self):
        self.count += 1

    def get_control(self, *other):
        c = self.control[self.count]
        if other:
            self.counter()
        return c

    def get_derivative(self):
        return (self.control[self.count] - self.control[self.count-1]) / self.time

    def load_manual_control(self, namefile): #u vecteur de toutes les commandes effectués, données par labview per exemple
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
                    # com -= 6.5
                    com = convert_angle(com)
                    com = np.deg2rad(com)
                    cmd.append(com)
                    pos.append(posm)
                n += 1
        return np.array(cmd), np.array(pos)
