#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''
This program works for Combine all of the data from chips in gy86, and add the KF support.

Functions:
    F1. Use the IMU update to combine the data sensors.
To Do:
    TD1. add IMUupdata function

Problem:

Bug:
    
Upgrades description:
    
'''


from mpu6050 import mpu6050
# from hmc5883l.hmc5883l import hmc5883l
import time
import math

class IMU():
    '''
    '''

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
    
    
    angle = {'yaw': 0, 'pit': 0, 'rol': 0 }


    def __init__(self):
        self.mpu = mpu6050(0x68)
        self.hmc = None
        self.ms  = None

    
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

        self.angle['yaw'] += gyr['z'] * Gyro_G * 0.002

        # pitch
        self.angle['pit'] = asin(-2 * q1 * q3 + 2 * q0 * q2)* 57.3 - self.AngleOffset_Pit
        # roll
        self.angle['rol'] = -atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 - self.AngleOffset_Rol
        
    
    
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
    while True:
        k.mpu6050read()
    
    while True:
        k.IMUupdata()
        print( 'x= %f, y= %f, z= %f\r'%( k.angle['rol'], k.angle['pit'], k.angle['yaw']),end = '')
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


