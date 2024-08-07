# berryIMU.py

import time
import math
from . import IMU
import datetime
import os
import sys

RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA = 0.40  # Complementary filter constant

################# Compass Calibration values ############
# Use calibrateBerryIMU.py to get calibration values
# Calibrating the compass isn't mandatory, however, a calibrated
# compass will result in a more accurate heading value.
'''
magXmin = 516
magYmin = -331
magZmin = -1787
magXmax = 1790
magYmax = 1303
magZmax = -256
'''

magXmin = 874
magYmin = -243
magZmin = -1773
magXmax = 1668
magYmax = 1149
magZmax = -259



# Kalman filter variables
Q_angle = 0.02
Q_gyro = 0.0015
R_angle = 0.005
y_bias = 0.0
x_bias = 0.0
XP_00 = 0.0
XP_01 = 0.0
XP_10 = 0.0
XP_11 = 0.0
YP_00 = 0.0
YP_01 = 0.0
YP_10 = 0.0
YP_11 = 0.0
KFangleX = 0.0
KFangleY = 0.0

def kalmanFilterY(accAngle, gyroRate, DT):
    y = 0.0
    S = 0.0

    global KFangleY, Q_angle, Q_gyro, y_bias, YP_00, YP_01, YP_10, YP_11

    KFangleY = KFangleY + DT * (gyroRate - y_bias)

    YP_00 = YP_00 + (-DT * (YP_10 + YP_01) + Q_angle * DT)
    YP_01 = YP_01 + (-DT * YP_11)
    YP_10 = YP_10 + (-DT * YP_11)
    YP_11 = YP_11 + (+Q_gyro * DT)

    y = accAngle - KFangleY
    S = YP_00 + R_angle
    K_0 = YP_00 / S
    K_1 = YP_10 / S

    KFangleY = KFangleY + (K_0 * y)
    y_bias = y_bias + (K_1 * y)

    YP_00 = YP_00 - (K_0 * YP_00)
    YP_01 = YP_01 - (K_0 * YP_01)
    YP_10 = YP_10 - (K_1 * YP_00)
    YP_11 = YP_11 - (K_1 * YP_01)

    return KFangleY

def kalmanFilterX(accAngle, gyroRate, DT):
    x = 0.0
    S = 0.0

    global KFangleX, Q_angle, Q_gyro, x_bias, XP_00, XP_01, XP_10, XP_11

    KFangleX = KFangleX + DT * (gyroRate - x_bias)

    XP_00 = XP_00 + (-DT * (XP_10 + XP_01) + Q_angle * DT)
    XP_01 = XP_01 + (-DT * XP_11)
    XP_10 = XP_10 + (-DT * XP_11)
    XP_11 = XP_11 + (+Q_gyro * DT)

    x = accAngle - KFangleX
    S = XP_00 + R_angle
    K_0 = XP_00 / S
    K_1 = XP_10 / S

    KFangleX = KFangleX + (K_0 * x)
    x_bias = x_bias + (K_1 * x)

    XP_00 = XP_00 - (K_0 * XP_00)
    XP_01 = XP_01 - (K_0 * XP_01)
    XP_10 = XP_10 - (K_1 * XP_00)
    XP_11 = XP_11 - (K_1 * XP_01)

    return KFangleX

def initialize_imu():
    IMU.detectIMU()  # Detect if BerryIMU is connected.
    if IMU.BerryIMUversion == 99:
        print(" No BerryIMU found... exiting ")
        sys.exit()
    IMU.initIMU()  # Initialize the accelerometer, gyroscope, and compass
    
initialize_imu()

def read_imu():
    ACCx = IMU.readACCx()
    ACCy = IMU.readACCy()
    ACCz = IMU.readACCz()
    GYRx = IMU.readGYRx()
    GYRy = IMU.readGYRy()
    GYRz = IMU.readGYRz()
    MAGx = IMU.readMAGx()
    MAGy = IMU.readMAGy()
    MAGz = IMU.readMAGz()

    # Apply compass calibration
    MAGx -= (magXmin + magXmax) / 2
    MAGy -= (magYmin + magYmax) / 2
    MAGz -= (magZmin + magZmax) / 2

    return ACCx, ACCy, ACCz, GYRx, GYRy, GYRz, MAGx, MAGy, MAGz

def compute_heading():
    #initialize_imu()

    gyroXangle = 0.0
    gyroYangle = 0.0
    gyroZangle = 0.0
    CFangleX = 0.0
    CFangleY = 0.0
    kalmanX = 0.0
    kalmanY = 0.0

    a = datetime.datetime.now()

    while True:
        ACCx, ACCy, ACCz, GYRx, GYRy, GYRz, MAGx, MAGy, MAGz = read_imu()

        b = datetime.datetime.now() - a
        a = datetime.datetime.now()
        LP = b.microseconds / (1000000 * 1.0)

        rate_gyr_x = GYRx * G_GAIN
        rate_gyr_y = GYRy * G_GAIN
        rate_gyr_z = GYRz * G_GAIN

        gyroXangle += rate_gyr_x * LP
        gyroYangle += rate_gyr_y * LP
        gyroZangle += rate_gyr_z * LP

        AccXangle = (math.atan2(ACCy, ACCz) * RAD_TO_DEG)
        AccYangle = (math.atan2(ACCz, ACCx) + M_PI) * RAD_TO_DEG

        if AccYangle > 90:
            AccYangle -= 270.0
        else:
            AccYangle += 90.0

        CFangleX = AA * (CFangleX + rate_gyr_x * LP) + (1 - AA) * AccXangle
        CFangleY = AA * (CFangleY + rate_gyr_y * LP) + (1 - AA) * AccYangle

        kalmanY = kalmanFilterY(AccYangle, rate_gyr_y, LP)
        kalmanX = kalmanFilterX(AccXangle, rate_gyr_x, LP)

        heading = 180 * math.atan2(MAGy, MAGx) / M_PI
        if heading < 0:
            heading += 360
            
        #-----------------------compensated heading-----------------------------
        #Normalize accelerometer raw values.
        accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
        accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
        

        #Calculate pitch and roll
        pitch = math.asin(accXnorm)
        #print("debugging numbers", math.cos(pitch), accYnorm/math.cos(pitch))
        roll = -math.asin(min(accYnorm/math.cos(pitch), 1))
        


        #Calculate the new tilt compensated values
        #The compass and accelerometer are orientated differently on the the BerryIMUv1, v2 and v3.
        #This needs to be taken into consideration when performing the calculations

        #X compensation
        if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
            magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
        else:                                                                #LSM9DS1
            magXcomp = MAGx*math.cos(pitch)-MAGz*math.sin(pitch)

        #Y compensation
        if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
            magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)
        else:                                                                #LSM9DS1
            magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)
            
        #Calculate tilt compensated heading
        tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI

        if tiltCompensatedHeading < 0:
            tiltCompensatedHeading += 360

        return heading, tiltCompensatedHeading
'''
if __name__ == "__main__":
    print("Heading: ", compute_heading())
'''