from ctypes import * #import ctypes to use C library in python
from array import *
import RPi.GPIO as GPIO #SET UP FOR GPIO
import math
import sys
import signal
import numpy as np
import time
import SERIAL

#======================================================================#
#initialization for the IMU library

#=====================================
#==========IMPORTING THE IMU==========
#=====================================

path = "/home/pi/LSM9DS1_RaspberryPi_Library/lib/liblsm9ds1cwrapper.so"
lib = cdll.LoadLibrary(path)

lib.lsm9ds1_create.argtypes = []
lib.lsm9ds1_create.restype = c_void_p

lib.lsm9ds1_begin.argtypes = [c_void_p]
lib.lsm9ds1_begin.restype = None

lib.lsm9ds1_calibrate.argtypes = [c_void_p]
lib.lsm9ds1_calibrate.restype = None

lib.lsm9ds1_gyroAvailable.argtypes = [c_void_p]
lib.lsm9ds1_gyroAvailable.restype = c_int
lib.lsm9ds1_accelAvailable.argtypes = [c_void_p]
lib.lsm9ds1_accelAvailable.restype = c_int
lib.lsm9ds1_magAvailable.argtypes = [c_void_p]
lib.lsm9ds1_magAvailable.restype = c_int

lib.lsm9ds1_readGyro.argtypes = [c_void_p]
lib.lsm9ds1_readGyro.restype = c_int
lib.lsm9ds1_readAccel.argtypes = [c_void_p]
lib.lsm9ds1_readAccel.restype = c_int
lib.lsm9ds1_readMag.argtypes = [c_void_p]
lib.lsm9ds1_readMag.restype = c_int

lib.lsm9ds1_getGyroX.argtypes = [c_void_p]
lib.lsm9ds1_getGyroX.restype = c_float
lib.lsm9ds1_getGyroY.argtypes = [c_void_p]
lib.lsm9ds1_getGyroY.restype = c_float
lib.lsm9ds1_getGyroZ.argtypes = [c_void_p]
lib.lsm9ds1_getGyroZ.restype = c_float

lib.lsm9ds1_getAccelX.argtypes = [c_void_p]
lib.lsm9ds1_getAccelX.restype = c_float
lib.lsm9ds1_getAccelY.argtypes = [c_void_p]
lib.lsm9ds1_getAccelY.restype = c_float
lib.lsm9ds1_getAccelZ.argtypes = [c_void_p]
lib.lsm9ds1_getAccelZ.restype = c_float

lib.lsm9ds1_getMagX.argtypes = [c_void_p]
lib.lsm9ds1_getMagX.restype = c_float
lib.lsm9ds1_getMagY.argtypes = [c_void_p]
lib.lsm9ds1_getMagY.restype = c_float
lib.lsm9ds1_getMagZ.argtypes = [c_void_p]
lib.lsm9ds1_getMagZ.restype = c_float

lib.lsm9ds1_calcGyro.argtypes = [c_void_p, c_float]
lib.lsm9ds1_calcGyro.restype = c_float
lib.lsm9ds1_calcAccel.argtypes = [c_void_p, c_float]
lib.lsm9ds1_calcAccel.restype = c_float
lib.lsm9ds1_calcMag.argtypes = [c_void_p, c_float]
lib.lsm9ds1_calcMag.restype = c_float
#======================================================================#

Ranges = [[10, -10],
          [10, -10],
          [10, -10]]     #Offsets for [[x][y][z]], format is [min max]
MagOffset = [0, 0, 0]    #Might not need these variables, might be able to just have them local to the function, then return them
RPY = [0,0,0]            #Roll Pitch and Yaw to be used with my scuffed logic
CFRP = [0,0]             #Complementary filter Roll and Pitch

#============================================
#==========MAGNETOMETER CALIBRATION==========
#============================================
def CalMag(MagArray, R, O):
     for i in range(3):
        if MagArray[i] < R[i][0]:
            R[i][0] = MagArray[i]
        if MagArray[i] > R[i][1]:
            R[i][1] = MagArray[i]
        O[i] = (R[i][0]+R[i][1])/2
return None
#===================================================
#==========GYROSCOPE CALIBRATION ALGORITHM==========
#===================================================
def CalGyro(O):
    LOffsets = [0, 0, 0]
    for i in range(20000):
        Cgx = lib.lsm9ds1_calcGyro(imu, gx)
        Cgy = lib.lsm9ds1_calcGyro(imu, gy)
        Cgz = lib.lsm9ds1_calcGyro(imu, gz)

return None
#=================================================
#==========COMPLEMENTARY FILTER FUNCTION==========
#=================================================
def ComplementaryFilter(xldata, gdata, mdata, RPYaw, mOffs):
     #Integrating the gyro data
     RPYaw[0] += gdata[0]*dt
     RPYaw[1] -= gdata[1]*dt

     #Calculating the roll and pitch from the Accelerometer (xl == Accel)
     rollXL = math.degrees(math.atan2(xldata[1], xldata[2]))
     pitchXL = math.degrees(math.atan2(-1*xldata[0], math.sqrt(xldata[1]**2+xldata[2]**2)))

     ratio = 0.98    #The percentage of Gyro data vs Accelerometer data
     RPYaw[0] = RPYaw[0]*ratio + rollXL*(1-ratio)
     RPYaw[1] = RPYaw[1]*ratio + pitchXL*(1-ratio)

     #Calculating the Yaw from the Magnetometer data (this is noisy)
     RPYaw[2] = math.degrees(math.atan2(-1*(mdata[1]-mOffs[1]), (mdata[0]-mOffs[0])))
return None

def IMU():
     if __name__ == "__main__":
         imu = lib.lsm9ds1_create()
     lib.lsm9ds1_begin(imu)
     if lib.lsm9ds1_begin(imu) == 0:
          print("Failed to communicate with LSM9DS1.")
          quit()
     lib.lsm9ds1_calibrate(imu)
        
     while lib.lsm9ds1_gyroAvailable(imu) == 0:
          pass
     lib.lsm9ds1_readGyro(imu)
     while lib.lsm9ds1_accelAvailable(imu) == 0:
          pass
     lib.lsm9ds1_readAccel(imu)
     while lib.lsm9ds1_magAvailable(imu) == 0:
          pass
     lib.lsm9ds1_readMag(imu)

     gx = lib.lsm9ds1_getGyroX(imu)
     gy = lib.lsm9ds1_getGyroY(imu)
     gz = lib.lsm9ds1_getGyroZ(imu)

     ax = lib.lsm9ds1_getAccelX(imu)
     ay = lib.lsm9ds1_getAccelY(imu)
     az = lib.lsm9ds1_getAccelZ(imu)

     mx = lib.lsm9ds1_getMagX(imu)
     my = lib.lsm9ds1_getMagY(imu)
     mz = lib.lsm9ds1_getMagZ(imu)

     cgx = lib.lsm9ds1_calcGyro(imu, gx)
     cgy = lib.lsm9ds1_calcGyro(imu, gy)
     cgz = lib.lsm9ds1_calcGyro(imu, gz)

     cax = lib.lsm9ds1_calcAccel(imu, ax)
     cay = lib.lsm9ds1_calcAccel(imu, ay)
     caz = lib.lsm9ds1_calcAccel(imu, az)

     cmx = lib.lsm9ds1_calcMag(imu, mx)
     cmy = lib.lsm9ds1_calcMag(imu, my)
     cmz = lib.lsm9ds1_calcMag(imu, mz)

     XLData = [cax, cay, caz]
     GyroData = [cgx, cgy, cgz]
     MagData = [cmx, cmy, cmz]

     CalMag(MagData, Ranges, MagOffset)
     ComplementaryFilter(XLData, GyroData, MagData, RPY, MagOffset)
     #ComplementaryFilter(XLData, GyroData, CFRP[0], CFRP[1])
     #print("RPY:  %f %f %f" % (RPY[0], RPY[1], RPY[2]))             #comparing the scuffed logic to the complementary filter logic
     #print("\n")
     #print(GyroData)
     #print("\n")
     a = Formatting(round(RPY[0],4),8)
     b = Formatting(round(RPY[1],4),8)
     c = Formatting(round(RPY[2],4),8)