#!/usr/bin/env python

import numpy as np
import math
import matplotlib.pyplot as plt

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import time 
import numpy as np

class static_var():
    y_static = 0;
    ylist = np.array([])
    def get_some_measurement(self):
        
        t = time.clock()
        y = math.sin( 0.1 * math.pi* t)
        ydot = y - self.y_static
        self.y_static = y
        # return (np.array([y, ydot]).T )
        return (np.array(y))
    
    def do_something_amazing(self, x):
        self.ylist = np.hstack((self.ylist, x))
        

        plt.plot(self.ylist)
        plt.draw()
        


def plotTest():
    k = static_var();
    plt.ion()
    plt.figure(1)
    plt.show()
    while 1:
        tmp = k.get_some_measurement()
        print (tmp)
        k.do_something_amazing(tmp)
        plt.pause(0.01)

        


def test():
    dt = 0.001


    my_filter = KalmanFilter(dim_x=2, dim_z=1)
    my_filter.x = np.array([[0.0],
                            [0.0]])       
    # initial state (location and velocity)

    my_filter.F = np.array([[1.0, dt ],
                            [0.0, 1.0]])    
    # state transition matrix

    my_filter.H = np.array([[1.0, 0.0]])    
    # Measurement function
    my_filter.P *= 1000.                 
    # covariance matrix
    my_filter.R = 5                      
    # state uncertainty
    my_filter.Q = Q_discrete_white_noise(2, dt, 0.1) 
    # process uncertainty


    while True:
        my_filter.predict()
        my_filter.update(get_some_measurement())

        # do something with the output
        x = my_filter.x
        do_something_amazing(x)
    pass

if __name__ == "__main__":
    plotTest();