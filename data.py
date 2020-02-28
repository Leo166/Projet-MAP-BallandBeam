import numpy as np

"""Caractéristique système"""
dt = 50*10**-3      # frequency of mesure [s]
g = 9.81            # acceleration [m/s^2]
m = 0.0559          # mass [kg]
r = 15*10**-3       # ball radius [m]
J = (2*m*r**2)/5    # inertial moment
eta = 10**-3        # dynamic viscosity [kg/ms]
vs = 1.414*10**-5   # ball volume [m^3]
ro = 997            # density of water [kg/m^3]

eq = [m + J/r**2, 6*r*eta*np.pi + 0.4, 0, -m*g + ro*vs*g, 0]   #equa_diff 5y" + 4y' + 3y = 2u + const 0.4




# Process the raw date in usable arrays
