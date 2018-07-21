#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''

'''

from IMUgy86 import IMU

def test():
    k = IMU()
    # while True:
    #     k.mpu6050read()
    
    while True:
        # k.mpu6050read()
        acc = k.mpu.get_accel_data()
        print( ' angle: x= %f, y= %f, z= %f. '%( k.angle['rol'], \
            k.angle['pit'], k.angle['yaw']),' loop time = %f\r '\
            % (k.LoopTime.value), end = '')
        # print( 'loop time = %f\r '% (k.LoopTime.value) )

        time.sleep(0.1)

    pass

if __name__ == '__main__':
    
    test()

