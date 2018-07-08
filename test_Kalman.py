#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''
This program works for kalman filter test.

Functions:
    F1. simplify the kf functions.
    F2. give a real-time data waveform for KF with a independent process.
    F3. give a simulate waveform for function test.
    F4. more

To Do:
    TD1. a child proc for data display
    TD2. simulate that for no sensors given.
    TD3. read the details about Kalman filter in IMU[1]. It gives a differenct method to apply KF. 
        [1]: (https://github.com/wildcat5566/MPU6050_Kalman_RPi/blob/master/kalman.py)
Problem:

Bug:
    Bug1. The waveforms covers the previous ones, rather than renew it, \
        so the waveform color always changes and it runs slow.

Upgrades description:
    2017/07/08 achieve the data plot with idenpendent process.
'''
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
        '''
        Inital the KF class.
        '''
        # if the data plot flag is true, call the proc init function
        if self.dataPlot_flag == 1:
            self.__dataPlotProcInit()

        # init the kf
        self.__kfInit()

    def __kfInit(self):
        '''
        Inital the Kalman Filter parameters.
        '''
        
        # set the sampling time 
        self.dt = 0.001
        # Inital the kf matrix shape
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
        '''
        give a test code for kf with simulate data
        '''
        
        while True:
            
            # get the simulate data for data generationg function
            data = self.simDataGen()

            self.kf.predict()
            
            self.kf.update( data )
            
            # sent the plot data if the data plot flag is set and the data queue is not None
            if self.dataPlot_flag == 1 and self.que != None:
                # sent plot data to queue
                self.que.put( self.kf.x[0].tolist()[0] )

            # set a sampling time 
            time.sleep( 0.01 )

            # print( self.kf.x[0])
        
        
    def kfupdate(self):
        
        data = self.simDataGen()
        self.kf.predict()
        
        self.kf.update( data )

        
    def __dataPlotProcInit(self):
        '''
        Request a independent process to plot the data waveform 
        '''
        # see multi_proc.py file for details
        print('Data display process loading...')
        # intial the plot data transfer queue
        self.que = Queue()

        # request the process
        p = Process(target = self.dataPlot, args = (self.que, )) 
        # child process start
        p.start()

        print('Data display process start.')

    def dataPlot(self, q):
        '''
        It is a independent child process to plot the data waveform

        '''
        
        # 开启matplotlib的交互模式
        plt.ion()  
        print('Data display process running...')
        # inital the variable to save the waveform data
        self.dataPlot = list()

        while 1:
            # print('data plotting!')
            
            # plt.xlim (0, 50)  # 首先得设置一个x轴的区间 这个是必须的
            # plt.ylim (-int (Y_lim), int (Y_lim))  # y轴区间
            # print( q.empty())
            
            if not q.empty():
                # get all of the waveform data sent to this process and append them to "self.dataPlot"
                while not q.empty():
                    self.dataPlot.append( q.get() )
                
                # plt.draw
                plt.plot (self.dataPlot)
                
            # set the data plot sampling time
            plt.pause (1) 

    def simDataGen(self):
        '''
        Give a simulate waveform
        the simulte is a sine wave + white noise

        '''
        # self.t = np.linspace(0, 1, 1e3)

        # get the time clock
        t = time.clock()
        # y = list(map(math.sin,2 * math.pi* t))
        y = math.sin( 20 * math.pi* t )
        y = np.asarray(y)
        # y = math.sin( 2 * math.pi* self.t)
        # get the white noise
        r = np.random.normal(0,0.1,size= 1)
        y = y + r
        
        return ( y )
        # return (np.array(y))


def KFtest():
    '''
    simulate the kf function

    '''
    k = KF()

    k.sim()


if __name__ == "__main__":
    KFtest()
