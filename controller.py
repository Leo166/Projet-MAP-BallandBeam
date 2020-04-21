import numpy as np
from utils import *
from math import *

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
        kp=-5   #au dessus de 12 ça bug
        ki=-1
        kd=-2

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
    def __init__(self, time, sign):
        #self.term = term
        self.error = []
        self.time = time
        self.control = []
        self.need_error = True
        self.sign=sign


    def add_error(self, e):
        self.error.append(e)

    def add_control(self, a):
        self.control.append(a)

    def get_control(self, *other):
        #k = self.term
        error = np.array(self.error)

        #error=error*0.01
        alpha1=1.4245
        alpha2=0.9303
        alpha3=0.01 #10**-2


        mu1=2.5266
        mu2=0.2970
        mu3=10**(-6) 

        k11=2.2772
        k12=1.5979
        k21=2.7004
        k22=1.5868
        k3=0.01

        delta=0.5072
        delta=15

        signeE=-np.sign(error[-1])
        u1=k11+(k12/(1+np.exp(mu1*((error[-1])**2))))
        u1= u1*((abs(error[-1]))**alpha1)*signeE
        u2=0
        if len(error) >=2:
            dedt=(error[-1]-error[-2])/self.time
            signeDEDT=-np.sign(dedt)
            u2= (k21+(k22)/(1+np.exp(mu2*(dedt**2))))*((abs(dedt))**alpha2)*signeDEDT
        
        ui=0
        integral=sum(error) * self.time
        #print(np.exp(mu3*(integral)**2))
        ui=((k3)/(1+np.exp(mu3*(integral)**2))) * (abs(integral)**alpha3)*np.sign(integral)
        # print("ui: ",ui)
        # print("u1: ",u1)
        # print("u2: ",u2)
        # print("")
        uINLSEF=u1+u2+ui+0.012946*self.sign
        #print(uINLSEF)
        u=delta*np.tanh(uINLSEF/delta)


        # u = 0
        # u += k[0] * error[-1]
        # if len(error) >= 2:
        #     u += k[1] * sum(error) * self.time + k[2] * (error[-1] - error[-2]) / self.time
        #u = convert_angle(u-6.5)
        u = np.deg2rad(u)
        #print(u)
        if other:
            self.add_control(u)
        return u

    def get_derivative(self):
        return (self.control[-1] - self.control[-2]) / self.time


class IADRC:
    def __init__(self, time,sp,sign):
        #self.term = term
        self.error = []
        self.time = time
        self.control = []
        self.need_error = True
        self.setpoint=sp
        self.sign=-sign

        self.r1=0
        self.r2=0

        self.x1=0
        self.x2=0
        self.x3=0


    def add_error(self, e):
        self.error.append(e)

    def add_control(self, a):
        self.control.append(a)

    def get_control(self, *other):
        #k = self.term
        error = np.array(self.error)
        dt = self.time
        r=self.setpoint

        

        # alpha = 0.5 # alpha dans (0,1)
        # beta = 2 # >1
        # gamma = 15 # >0
        # R = 16 #>0

        # self.r1 = self.r1 + dt * self.r2
        # self.r2 = self.r2 + dt * (-(R**2) * tanh((beta*self.r1-(1-alpha)*r)/gamma) - R*self.r2)

        # uinlsef = 0
        
        # e0 = self.r1 - self.x1
        # e1 = self.r2 - self.x2

        # k11 = 2.2772
        # k12 = 1.5979
        # mu1 = 2.5266
        # k21 = 2.7004
        # k22 = 1.5868
        # mu2 = 0.2970

        # ke0 = k11 + ((k12)/(1+np.exp(mu1 * (e0)**2)))
        # ke1 = k21 + ((k22)/(1+np.exp(mu2 * (e1)**2)))


        # alpha1 = 1.4245
        # alpha1 = 0.5
        # alpha2 = 0.9303
        # alpha2 = 0.3

        # f1 = (abs(e0)**alpha1) * np.sign(e0)
        # f2 = (abs(e1)**alpha2) * np.sign(e1)

        # uinlsef+= f1 * ke0
        # uinlsef+= f2 * ke1

        # # print("f1: ",f1)
        # # print("ke0: ",ke0)
        # # print("f2: ",f2)
        # # print("ke1: ",ke1)
        # # print()

        # alphai = 0.01
        # k = 0.01
        # mu = 10**(-6) 

        # integral = sum(error) * self.time
        # uintegrator = ((abs(integral)**alphai) * np.sign(integral)) * (k/(1+np.exp(mu* integral**2)))

        # uinlsef+=uintegrator

        # #print(uintegrator)

        # b = 1
        # # print("uinlsef: ",uinlsef)
        # # print("x3: ",self.x3)
        # # print()

        # u = uinlsef - (self.x3/b)

        # y = (-error[-1]) + r

        # Ka = 0.6265
        # alphaa = 0.8433
        # Kb = 0.5878
        # betaa = 0.004078
        # beta1 = 30.4
        # beta2 = 513.4
        # beta3 = 1570.8/1000

        # g = Ka * abs(y-self.x1)**alphaa * np.sign(y-self.x1) + Kb * abs(y-self.x1)**betaa * (y-self.x1)
        # self.x1 = self.x1 + dt * (self.x2 + beta1 * g)
        # self.x2 = self.x2 + dt * (self.x3 + b * u + beta2 * g)
        # self.x3 = self.x3 + dt * beta3 * g

        # #print(g)

        # u = np.deg2rad(u)
        # #print(u)
        # if other:
        #     self.add_control(u)

        # return u
        alpha1=1.4245
        alpha2=0.9303
        alpha3=0.01 #10**-2


        mu1=2.5266
        mu2=0.2970
        mu3=10**(-6) 

        k11=2.2772
        k12=1.5979
        k21=2.7004
        k22=1.5868
        k3=0.01

        delta=0.5072
        delta=15

        signeE=-np.sign(error[-1])
        u1=k11+(k12/(1+np.exp(mu1*((error[-1])**2))))
        u1= u1*((abs(error[-1]))**alpha1)*signeE
        u2=0
        if len(error) >=2:
            dedt=(error[-1]-error[-2])/self.time
            signeDEDT=-np.sign(dedt)
            u2= (k21+(k22)/(1+np.exp(mu2*(dedt**2))))*((abs(dedt))**alpha2)*signeDEDT
        
        ui=0
        integral=sum(error) * self.time
        #print(np.exp(mu3*(integral)**2))
        ui=((k3)/(1+np.exp(mu3*(integral)**2))) * (abs(integral)**alpha3)*np.sign(integral)
        # print("ui: ",ui)
        # print("u1: ",u1)
        # print("u2: ",u2)
        # print("")
        y=(-error[0]) + r
        terme=abs(r-y)*0.0185-0.0605
        #print(terme)
        uINLSEF=u1+u2+ui+(0.012946+terme)*self.sign  #0.55
        #uINLSEF+=0.55
        #uINLSEF=u1+u2+ui
        #print(uINLSEF)
        u=delta*np.tanh(uINLSEF/delta)

        b=1
        # print("x3: ",self.x3)
        # print("u: ",u)
        u -= self.x3/b
        
        y=3
        Ka = 0.6265
        alphaa = 0.8433
        Kb = 0.5878
        betaa = 0.004078
        beta1 = 30.4
        beta2 = 513.4
        beta3 = 1570.8/100

        g = Ka * abs(y-self.x1)**alphaa * np.sign(y-self.x1) + Kb * abs(y-self.x1)**betaa * (y-self.x1)
        self.x1 = self.x1 + dt * (self.x2 + beta1 * g)
        self.x2 = self.x2 + dt * (self.x3 + b * u + beta2 * g)
        self.x3 = self.x3 + dt * beta3 * g


        # u = 0
        # u += k[0] * error[-1]
        # if len(error) >= 2:
        #     u += k[1] * sum(error) * self.time + k[2] * (error[-1] - error[-2]) / self.time
        #u = convert_angle(u-6.5)
        u = np.deg2rad(u)
        #print(u)
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
        Kp=-1.952
        Kp=-10
        Kd=-1.5
        Ki=-0.5
        emax=50

        k0=-0.001112*4.3

        error = np.array(self.error)
        #print("error",error[-1])
        if abs(error[-1])<=emax:
            err=error[-1]
            #print("err <=emax", err)
        else:
            err=emax*np.sign(error[-1])
            #print("err >emax",err)
        #error[-1]=err
        #print("err: ",err)
        #k=(np.exp(k0*err)+np.exp(-k0*err))/2
        #print("koerr: ",k0*err)
        k=np.cosh(k0*err)
        #print("cosh ",k)
        up=0
        ui=0
        ud=0
        up=Kp*k*err
        if len(error)>=2:
            ui=Ki *k* sum(error) * self.time
            ud=Kd*k*((err-error[-2])/self.time)

        u=(up+ui+ud)
        #print("u: ",u)
        u = convert_angle(u-6.5)
        u = np.deg2rad(u)
        if abs(u)>(np.pi*50/180):
            u=(np.pi*50/180)*np.sign(u)
        if other:
            self.add_control(u)
        return u

    def get_derivative(self):
        return (self.control[-1] - self.control[-2]) / self.time


class ADRC:
    def __init__(self, time):
        #self.term = term
        self.error = []
        self.time = time
        self.control = []
        self.need_error = True
        self.x1=0
        self.x2=0
        self.z1=0
        self.z2=0
        self.z3=0
        self.u1=0
        self.esum=0


    def fal(self,e, alfa, delta):

        s = (np.sign(e+delta)-np.sign(e-delta))/2.0
        fal_out = e * s/(pow(delta, 1-alfa))+pow(abs(e), alfa)*np.sign(e)*(1-s)
        return fal_out

    def fsg(self,x, d):

        fsg_out = (np.sign(x+d) - np.sign(x-d))/2.0
        return fsg_out


    def fhan(self,x1, x2, r, h0):

        d = r * h0 * h0
        a0 =h0 * x2
        y =x1+a0
        a1 = sqrt(d * (d + 8 * abs(y)))
        a2 = a0 + np.sign(y) * (a1-d)/2.0
        a = (a0 + y) * self.fsg(y, d) + a2 * (1 - self.fsg(y, d))
        fhan_out = -1 * r * (a / d) * self.fsg(a, d) - r * np.sign(a) * (1 - self.fsg(a, d))
        return fhan_out
        



    def add_error(self, e):
        self.error.append(e)

    def add_control(self, a):
        self.control.append(a)

    def get_control(self, *other):
        #k = self.term
        error = np.array(self.error)

        # h=self.time
        # y1_star=3
        # r0=20
        # b01 = 1
        # b02 = 1/(2*(h**0.5))
        # b03 = 2/(25*(h**1.2))
        # h1=25*h
        # c=1
        # r=10000
        # b0=0.8

        # fh = self.fhan(self.x1 - y1_star, self.x2, r0, h)
        # self.x1 += h * self.x2
        # self.x2 += h * fh
        
        # #e = self.z1 - self.x1
        # e=error[-1]-3+self.z1

        # fe = self.fal(e, 0.5, h)
        # fe1 = self.fal(e, 0.25, h)

        # self.z1 += h * self.z2 - b01 * e
        # self.z2 += h * self.z3 - b02 * fe + self.u1 * h * b0
        # self.z3 -= h * b03 * fe1

        # e1 = self.x1 - self.z1
        # e2 = self.x2 - self.z2

        # self.u1 = (self.fhan(e1, c * e2, r, h1) + self.z3)/b0

        # u=self.u1
        h=self.time

        b11 = 3
        b12 = 1
        b21 = 3
        b22 = 2
        r = 100
        r0 = 20
        b01 = 0.1
        b02 = 0.5
        b03 = 1
        h1 = 25*h
        c = 1.0
        y1_star = 3
        b0=0.8


        fh = self.fhan(self.x1 - y1_star, self.x2, r0, h)
        self.x1= self.x1 + h * self.x2
        self.x2 = self.x2 + h * fh
        e = self.z1 - self.x1
        fe = self.fal(e, 0.5, h)
        fe1 = self.fal(e, 1, h)
        self.z1 = self.z1 + h * (self.z2 - b01 * e)
        self.z2 = self.z2 + h * (self.z3 - b02 * fe + self.u1 * b0)
        self.z3 = self.z3 + h * (-b03 * fe1)
        e1 = self.x1 - self.z1
        e2 = self.x2 - self.z2
        self.u1= (-self.fhan(e1, c * e2, r, h1) - self.z3)/b0

        u=self.u1



        # u = 0
        # u += k[0] * error[-1]
        # if len(error) >= 2:
        #     u += k[1] * sum(error) * self.time + k[2] * (error[-1] - error[-2]) / self.time
        #u = convert_angle(u-6.5)
        u = np.deg2rad(u)
        #print(u)
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
