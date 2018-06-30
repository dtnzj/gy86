#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import time 
import numpy as np
from multiprocessing import Pool
import os, time


class KF():
    
    dataDisp_flag = 1
    dt = None
    kf = None
    dataList = None
    def __init__(self):
        
        # if the data dispaly flag is true, call the proc init funcion
        if self.dataDisp_flag == 1:
            self.__dataDispProcInit()
        
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
        
        self.dataList = list()
        while True:
            
            data = self.__simDataGen()
            self.kf.predict()
            
            self.kf.update( data )
            
            self.dataList.append( self.kf.x[0].tolist() )
        
        pass
    def kfupdate(self):
        
        data = self.__simDataGen()
        self.kf.predict()
        
        self.kf.update( data )

        pass
    def __dataDispProcInit(self):
        
        # see multi_proc.py file for details
        print('Data display process loading...')
        p = Pool(1)
        p.apply_async(self.dataDisp, args=('',))
        p.close()
        print('Data display process start.')

    def dataDisp(self):
        
        plt.ion ()  # 开启matplotlib的交互模式
        print('Data display process running...')    
        while 1:
            print('tmp data displaying!')
            # time.sleep(1)
        
            # plt.xlim (0, 50)  # 首先得设置一个x轴的区间 这个是必须的
            # plt.ylim (-int (Y_lim), int (Y_lim))  # y轴区间
            plt.plot (self.dataList)  # 将list传入plot画图
            plt.pause (1)  # 这个为停顿0.01s，能得到产生实时的效果

    def __simDataGen(self):
        # self.t = np.linspace(0, 1, 1e3)
        
        t = time.clock()
        y = list(map(math.sin,2 * math.pi* t))
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

    pass


if __name__ == "__main__":
    KFtest();
    pass