from mpu6050 import mpu6050
from hmc5883l.hmc5883l import hmc5883l
import time

class IMU():
    '''
    '''
    def __init__(self):
        
        pass

    pass

def mpu6050read():
    sensor = mpu6050(0x68)

    while True:
        k = sensor.get_accel_data()
        print( 'x= %f, y= %f, z= %f\r'%( k['x'], k['y'], k['z']),end = '')
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
    
     basicTest()


