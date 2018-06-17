#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import time 
import numpy as np

class static_var():
    y_static = 0
    ylist = np.array([[0],[0]])
    t = None
    
    def get_some_measurement(self):
        
        self.t = np.linspace(0, 1, 1e3)
        
        #  self.t = time.clock()
        y = list(map(math.sin,2 * math.pi* self.t))
        y = np.asarray(y)
        # y = math.sin( 2 * math.pi* self.t)
        r = np.random.normal(0,0.1,size= 1000)
        y = y + r
        ydot = np.diff(y)
        # ydot = np.array([0, ydot])
        ydot = np.insert(ydot, 0, 0)
        print(y.size)
        print(ydot.size)
        # ydot = y - self.y_static
        # self.y_static = y
        tmp = np.asarray([y, ydot]).T
        return ( tmp )
        # return (np.array(y))
    
    def do_something_amazing(self, x):
        pass
        


def plotTest():
    k = static_var();

    plt.ion()
    
    tmp = k.get_some_measurement()
    print (tmp)
    # print (self.ylist)
    # print (x)
    
    # self.ylist = np.hstack((self.ylist, x.T))
    
    # print(self.ylist.shape)
    # print (self.ylist)
    plt.figure(1)
    self.ylist = x 
    plt.plot( x[:, 0 ])
    # plt.plot(self.t, self.ylist[1])
    
    # plt.show()
    # k.do_something_amazing(tmp)
    # plt.pause(0.001)
    x = input()

        


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
    # my_filter.P = 1000.
    # covariance matrix
    my_filter.R = 0.1
    # state uncertainty
    my_filter.Q = Q_discrete_white_noise(2, dt, 1e5) 
    # process uncertainty

    x_list = list()
    
    k = static_var()
    tmp = k.get_some_measurement()
    for i in tmp: 
        my_filter.predict()
        # print ( i )
        my_filter.update( i[0] )

        # do something with the output
        # x_list = my_filter.x
        # print( my_filter.x)

        x_list.append( my_filter.x[0][0].tolist() )
    
    # plt.figure(1)
    # plt.plot( tmp ) 
    a = tmp[:, 0]
    b = np.asarray( x_list )
    c = np.vstack((a,b)).T

    plt.figure(2)
    # tmp = np.asarray(tmp).T
    plt.plot( c )
    # plt.plot( np.hstack((x_list, tmp)) )
    plt.show()

    input() 
    

if __name__ == "__main__":
    test();