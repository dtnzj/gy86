#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


from multiprocessing import Pool, Process, Queue
import os, time


class KF():

    dataPlot_flag = 1
    dt = None
    kf = None
    dataPlot = None
    que = None
    def __init__(self):

        # if the data dispaly flag is true, call the proc init funcion
        if self.dataPlot_flag == 1:
            self.__dataPlotProcInit()

        # init the kf
        self.__kfInit()

    def __kfInit(self):
        self.dt = 0.001
        self.kf = KalmanFilter(dim_x=2, dim_z=1)
        # initial state (location and velocity)
        self.kf.x = np.array([[0.0],
                              [0.0]])       

        # state transition matrix
        self.kf.F = np.array([[1.0, self.dt ],
                              [0.0, 1.0]])    
        
        # Measurement function
        self.kf.H = np.array([[1.0, 0.0]])
        # covariance matrix
        # self.kf.P = 1000.
        # state uncertainty
        self.kf.R = 0.1
        # process uncertainty
        self.kf.Q = Q_discrete_white_noise(2, self.dt, 1e5) 

    def sim(self):
        
        
        while True:
            
            data = self.simDataGen()
            self.kf.predict()
            
            self.kf.update( data )
            
            if self.dataPlot_flag == 1 and self.que != None:
                self.que.put( self.kf.x[0].tolist()[0] )

            time.sleep( 0.01 )

            # print( self.kf.x[0])
        
        
    def kfupdate(self):
        
        data = self.simDataGen()
        self.kf.predict()
        
        self.kf.update( data )

        
    def __dataPlotProcInit(self):
        
        # see multi_proc.py file for details
        print('Data display process loading...')
        self.que = Queue()
        qq = self.que
        
        p = Process(target = self.dataPlot, args = (self.que, )) 
        
        p.start()

        print('Data display process start.')

    def dataPlot(self, q):
        
        plt.ion()  # 开启matplotlib的交互模式
        print('Data display process running...')
        self.dataPlot = list()
        while 1:
            # print('data plotting!')
            
            # plt.xlim (0, 50)  # 首先得设置一个x轴的区间 这个是必须的
            # plt.ylim (-int (Y_lim), int (Y_lim))  # y轴区间
            # print( q.empty())
            if not q.empty():
                while not q.empty():
                    self.dataPlot.append( q.get() )
                
                # plt.draw
                plt.plot (self.dataPlot)
                
            
            plt.pause (1) 

    def simDataGen(self):
        # self.t = np.linspace(0, 1, 1e3)
        
        t = time.clock()
        # y = list(map(math.sin,2 * math.pi* t))
        y = math.sin( 20 * math.pi* t )
        y = np.asarray(y)
        # y = math.sin( 2 * math.pi* self.t)
        r = np.random.normal(0,0.1,size= 1)
        y = y + r
        # ydot = np.diff(y)
        # ydot = np.array([0, ydot])
        # ydot = np.insert(ydot, 0, 0)
        # print(y.size)
        # print(ydot.size)
        # ydot = y - self.y_static
        # self.y_static = y
        # tmp = np.asarray([y, ydot]).T
        # return ( tmp )
        
        return ( y )
        # return (np.array(y))


def KFtest():
    
    k = KF()

    k.sim()

def test():
    k = KF()
    while True:
        print( k.simDataGen() )

    pass

if __name__ == "__main__":
    KFtest()
