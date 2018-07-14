#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''
This program works for Combine all of the data from chips in gy86, and add the KF support.

Functions:
    F1. Use the IMU update to combine the data sensors.

To Do:
    TD1. add IMUupdata function
    TD2. IMUupdata test
    TD3. the print func will slow down the func compute speed, 
        so it is needed to create a new process to run IMU and print in the main proc.
Problem:
    P1. the IMUupdata func gives an incorrect result, which is think to be caused 
        by the sampling time
    P2. 
Bug:
    B1. In TD3, the LoopTime cannot tranmit to the print pro, only 0 printed. 
        And the other variable is transmit correctly.

Upgrades description:
    2018/07/14  Add the IMUupdate func bases on the C code.
                Add a child proc to run the IMU func 
                and print the varibles in the main proc. (P1, B1)
'''


from mpu6050 import mpu6050
from multiprocessing import Process, Manager, Value
# from hmc5883l.hmc5883l import hmc5883l
import time
import math

class IMU():
    '''
    '''
    Gyro_G 	= 0.0610351	
    Gyro_Gr = 0.0010653

    # proportional gain governs rate of convergence to accelerometer/magnetometer
    Kp = 10.0
    # integral gain governs rate of convergence of gyroscope biases
    Ki = 0.008
    # half the sample period???????
    halfT = 0.001
    
    AngleOffset_Rol=0
    AngleOffset_Pit=0

    # quaternion elements representing the estimated orientation
    q0 = 1
    q1 = 0
    q2 = 0
    q3 = 0
    
    # scaled integral error
    exInt = 0
    eyInt = 0
    ezInt = 0
    
    # angle = {'yaw': 0, 'pit': 0, 'rol': 0 }
    manager = Manager()
    angle = manager.dict()
    angle['yaw'] = 0
    angle['pit'] = 0
    angle['rol'] = 0
    
    LoopTime = Value('f',0)
    
    def __init__(self):
        self.mpu = mpu6050(0x68)
        self.hmc = None
        self.ms  = None
        self.__imuProcInit()

    def __imuProcInit(self):
        '''
        Request a independent process to updat the IMU
        '''
        # see multi_proc.py file for details
        print('IMUupdata process loading...')
        # intial the plot data transfer queue
        
        # request the process
        p = Process(target = self.__IMUupdataProc)
        # child process start
        p.start()

        print('Data display process start.')
    
    def __IMUupdataProc(self):
        
        t = time.time()
        count = 0 
        while True:
            count += 1
            self.IMUupdata()
            
            if count >=100:
                count = 0
                t_end = time.time()
                self.LoopTime.value = t_end -t
                print( t_end, self.LoopTime.value) 
                print( t_end, self.LoopTime.value) 
                print( t_end, self.LoopTime.value) 
                t = t_end
            
            time.sleep(0.001)

        
    def IMUupdata(self):
        '''
        Give the Euler angles with the data of gyr and acc sensor.
        Un-tested

        Input:  T_int16_xyz gyr, T_int16_xyz acc
        OutPUt: T_float_angle angle
        '''
        acc = self.mpu.get_accel_data()
        gyr = self.mpu.get_gyro_data()
        
        ax = acc['x']
        ay = acc['y']
        az = acc['z']

        gx = gyr['x']
        gy = gyr['y']
        gz = gyr['z']

        q0 = self.q0
        q1 = self.q1
        q2 = self.q2
        q3 = self.q3
        
        # hx, hy, hz, bx, bz
        # vx, vy, vz 
        # # wx, wy, wz
        # ex, ey, ez

        # ???????????
        q0q0 = q0 * q0
        q0q1 = q0 * q1
        q0q2 = q0 * q2
        #  q0q3 = q0 * q3
        q1q1 = q1 * q1
        #  q1q2 = q1 * q2
        q1q3 = q1 * q3
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q3q3 = q3 * q3

        if ax * ay * az==0:
            return
        
        gx *= self.Gyro_Gr
        gy *= self.Gyro_Gr
        gz *= self.Gyro_Gr
        
        norm = math.sqrt(ax * ax + ay * ay + az * az)       #acc?????
        ax = ax /norm
        ay = ay / norm
        az = az / norm

        # estimated direction of gravity and flux (v and w)              ?????????/??
        vx = 2 * (q1q3 - q0q2)												#????xyz???
        vy = 2 * (q0q1 + q2q3)
        vz = q0q0 - q1q1 - q2q2 + q3q3 

        # error is sum of cross product between reference direction of fields and direction measured by sensors
        ex = (ay * vz - az * vy)                            					 #???????????????
        ey = (az * vx - ax * vz) 
        ez = (ax * vy - ay * vx) 

        #???????
        self.exInt = self.exInt + ex * self.Ki  
        self.eyInt = self.eyInt + ey * self.Ki
        self.ezInt = self.ezInt + ez * self.Ki

        # adjusted gyroscope measurements
        #???PI???????,???????
        gx = gx + self.Kp * ex + self.exInt
        gy = gy + self.Kp * ey + self.eyInt
        gz = gz + self.Kp * ez + self.ezInt
        #???gz????????????????,??????????????

        # integrate quaternion rate and normalise						   #????????
        q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * self.halfT
        q1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * self.halfT
        q2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * self.halfT
        q3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * self.halfT

        # normalise quaternion
        norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
            
        q0 = q0 / norm
        q1 = q1 / norm
        q2 = q2 / norm
        q3 = q3 / norm

        self.angle['yaw'] += gyr['z'] * self.Gyro_G * 0.002

        # pitch
        self.angle['pit'] = math.asin(-2 * q1 * q3 + 2 * q0 * q2)* 57.3 - self.AngleOffset_Pit
        # roll
        self.angle['rol'] = -math.atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 - self.AngleOffset_Rol
        
    
    
    def __DataPlotInit(self):
        '''
        Inital the child proc of data plot function with DataPlotProc()
        '''

        pass
    def DataPlotProc(self):
        '''
        Plot the data waveform in this child proc
        '''
        pass
    

    def mpu6050read(self):
        
        
        k = self.mpu.get_accel_data()
        print( 'x= %f, y= %f, z= %f\r'%( k['x'], k['y'], k['z']),end = '')
        time.sleep(0.1)
        

def test():
    k = IMU()
    # while True:
    #     k.mpu6050read()
    
    while True:
        print( ' x= %f, y= %f, z= %f'%( k.angle['rol'], k.angle['pit'], k.angle['yaw']),'loop time = %f\r '% (k.LoopTime.value), end = '')
        # print( 'loop time = %f\r '% (k.LoopTime.value) )
        
        time.sleep(0.1)

    pass


def basicTest():
    
    sensor6050 = mpu6050(0x68)

    accel_data = sensor6050.get_accel_data()
    gyro_data = sensor6050.get_gyro_data()
    temp = sensor6050.get_temp()

    # print("Accelerometer data")
    # print("x: " + str(accel_data['x']))
    # print("y: " + str(accel_data['y']))
    # print("z: " + str(accel_data['z']))

    # print("Gyroscope data")
    # print("x: " + str(gyro_data['x']))
    # print("y: " + str(gyro_data['y']))
    # print("z: " + str(gyro_data['z']))

    # print("Temp: " + str(temp) + " C")
    # sleep(0.5)

    sensor6050.bypass_mode_en()
    pass


if __name__ == '__main__':
    
    test()


