
# gy86
A combined library for gy86 which contains mpu6050, hmc5883l, ms5611

## TO DO
- add the KF for data filer
- a better HMC library 
- MS library
- improve the MPU lib
- the setup file ?

## Outline
### MPU6050 class 
a improved MPU6050 lib from [MPU6050](https://github.com/Tijndagamer/mpu6050) 
which add the support to control all of the register and functions, i.e. by pass mode. 
### HMC5883l class
I don't find a excellent lib for this chip, so I prefer to give a better one. 
### MS5611 class
same as the last
### Combined class
Combine all of the data from chips in gy86, and add the KF support. 
### Kalman filter
filter the data from chips. 
1. capture the 3-D data
2. display the data
    3. a child proc for data display
    4. how to simulate that for no sensors given. 
3. give the filtered data with KF
2. make a 3-D KF model for gyro and acc data filter. 

