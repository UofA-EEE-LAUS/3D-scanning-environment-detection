#This file was brought to you by Glenn Walsh
#with some assistance from the rest of my team, whose names you can find in the github repository you got this file from :)
import serial
import socket
import ctypes
from ctypes import *
from array import *
import math
import numpy as np
import time
import sys
import signal
import random
import melopero_vl53l1x as mp
import RPi.GPIO as GPIO #SET UP FOR GPIO
dt = 1/80


#============================================
#==========ESTABLISHING SERIAL PORT==========
#============================================
#prt = '/dev/ttyACM0'    #This is the serial port that an arduino is plugged in
#ser = serial.Serial(port=prt, baudrate=9600, parity=serial.PARITY_NONE,
#        stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=dt)
# the ser function sets up all the variables that we need, and doesnt really need to be toucher

#=====================================
#==========IMPORTING THE IMU==========
#=====================================
path = "/home/pi/LSM9DS1_RaspberryPi_Library/lib/liblsm9ds1cwrapper.so" #THis is the path to the library of our IMU
lib = cdll.LoadLibrary(path)                                            #Feel free to setup your IMU as it needs to be

#The following sets up all the functions that may need to be used throughout this program
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

#========================================
#==========GLOBAL VARIABLE AREA==========
#========================================
Ranges = [[10, -10],
          [10, -10],
          [10, -10]]    #Offsets for [[x][y][z]], format is [min max]
MagOffset = [0, 0, 0]                       #Magnetometer offsets, have these set to 0 when calibrating, then once calibration has been performed, change them to the calibrated values
RPY = [0,0,0]                               #Roll Pitch and Yaw

#=================================================
#==========COMPLEMENTARY FILTER FUNCTION==========
#=================================================
def ComplementaryFilter(xldata, gdata, mdata, RPYaw, mOffs):
    #Integrating the gyro data
    RPYaw[0] += gdata[0]*dt
    RPYaw[1] -= gdata[1]*dt
    #RPYaw[2] -= gdata[2]
    
    #Calculating the roll and pitch from the Accelerometer (xl == Accel)
    rollXL = math.degrees(math.atan2(xldata[1], xldata[2]))
    pitchXL = math.degrees(math.atan2(-1*xldata[0], math.sqrt(xldata[1]**2+xldata[2]**2)))
    
    #Getting the yaw angle from the Magnetometer
    MagYaw = math.degrees(math.atan2(-1*(mdata[1]-mOffs[1]), (mdata[0]-mOffs[0])))
    
    ratio = 0.96    #The percentage of Gyro data vs Accelerometer data
    RPYaw[0] = RPYaw[0]*ratio + rollXL*(1-ratio)    #applying a low pass filter to the accelerometer, and a high pass filter to the gyroscope
    RPYaw[1] = RPYaw[1]*ratio + pitchXL*(1-ratio)   # to obtain the roll and pitch.
    
    #Calculating the Yaw from the Magnetometer data (this is noisy but accurate)
    RPYaw[2] = MagYaw+180
    
    
#============================================
#==========MAGNETOMETER CALIBRATION==========
#============================================
def CalMag(MagArray, R, O):
    #Simple logic going on here, it works by cheching the current magnetometer value against the max and min, and if it exceeds them, they are replaced
    for i in range(3):
        if MagArray[i] < R[i][0]:
            R[i][0] = MagArray[i]
        if MagArray[i] > R[i][1]:
            R[i][1] = MagArray[i]
        O[i] = (R[i][0]+R[i][1])/2
        
#====================================
#==========FORMATTING LOGIC==========
#====================================
def Formatting(inp, maxlength):  #Both inputs are numbers
    #This function checks if a numbers polarity, and outputs a string of maxLength with the correct polarity
    polarity = "-"
    if inp > 0:
        polarity = "+"
        
    tempVal = abs(inp)
    outp = str(tempVal)
    
    #This functions pads zeros to the start of inp till it reaches maxlength
    if len(str(tempVal)) < maxlength:
        for i in range(maxlength - len(str(tempVal))):
            outp = "0" + outp
    
    #THis is all done so that when the package is transmitted, the elements can have uniform string length
    return polarity + outp
    
#=========================================
#==========TIME OF FLIGHT SENSOR==========
#=========================================
LS = mp.VL53L1X()

def Lidar(sensr):
    #Code for obtaining the distance from the timeof flight sensor
     sensr.start_ranging(mp.VL53L1X.LONG_DST_MODE)
     TOF = sensr.get_measurement()
     TOF = TOF/10
     sensr.stop_ranging()
     #sensr.close_connection()
     return TOF

#==================================
#==========SONAR RETRIEVE==========
#==================================
def Sonar(TRIG, ECHO, LKV):
    #Code for obtaining the distance from the sonar sensor
    GPIO.setmode(GPIO.BOARD)      #BOARD LAYOUT OF GPIO
    GPIO.setup(TRIG, GPIO.OUT)     #OUTPUUT from which pin
    GPIO.setup(ECHO, GPIO.IN)      #INPUT FROM WHICH PIN

    GPIO.output(TRIG, True)        #Trigger on Low to start fresh
    GPIO.setwarnings(False)        #To ignore the warning when testing
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    #GPIO.output(TRIG, True)
    pulse_start = time.time()
    pulse_end = time.time()
    
    to = 0
    
    #print("WHILE LOOP 1")
    while GPIO.input(ECHO)==0:
        #print(GPIO.input(ECHO))
        pulse_start = time.time()
        
        if to>100:
            return LKV
        to +=1
        
    #print("ESCAPED LOOP 1")
    
    
    #print("WHILE LOOP 2")
    while GPIO.input(ECHO)==1:
        pulse_end = time.time()
    #print("ESCAPED LOOP 2")

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    sonar = round(distance+1.15, 2)
    GPIO.cleanup()
    return sonar

#=====================================
#==========TCP HOSTING SETUP==========
#=====================================
TCP_IP = '129.127.225.70'   #This has to be the ip address of the pi
TCP_PORT = 42072            #Generic unused port, feel free to change this as needed
BUFFER_SIZE = 32            #Good idea to set this around32 or 64
#print('Starting server: ',TCP_IP,':',TCP_PORT,', please enjoy it ^.^')  #Debugging code
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
print('listening...')
s.listen(1)                 #This sets it up so that the whole program waits until something is connected to the IP address, if you dont want to use tcp hosting 
                            #Comment out this code, and the [conn, addr = s.accept(), conn.send(msg.encode())] lines later in the code,


#=================================
#==========MAIN FUNCTION==========
#=================================
if __name__ == "__main__":
    #Sets up the IMU
    imu = lib.lsm9ds1_create()
    lib.lsm9ds1_begin(imu)
    if lib.lsm9ds1_begin(imu) == 0:
        print("Failed to communicate with LSM9DS1.")
        quit()
    #lib.lsm9ds1_calibrate(imu)
    #This line is required to make sure the sonar doesnt get stuck in an infinite loop
    #comment this out if you arent using our sonar sensor
    s_prev = 0

    conn, addr = s.accept()
    print('Connection address: ', addr)    
    while True:
        #The part after the connection to the tcp client has been established
        #or just the rest of the main loop function if TCP isnt being used
        #The following part makes sure that all of the sensors are available,
        #Then simply retrieves all the data from the IMU, then calculates the
        #real life values from the sensor
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
        
        #I place the sensor data respectively into their appropriate arrays
        XLData = [cax, cay, caz]
        GyroData = [cgx, cgy, cgz]
        MagData = [cmx, cmy, cmz]
        
        #I call the magnetometer calibrate funciton.
        #This is only needed if you need to calibrate your magnetometer
        #if not, comment this out, as its already calibrated
        CalMag(MagData, Ranges, MagOffset)
        # print(MagData)
        
        #THis calls the complementray filter, which makes a stable roll and pitch values, and returns the yaw
        ComplementaryFilter(XLData, GyroData, MagData, RPY, MagOffset)
        
        #==================================================
        #==========BACKBONE OF THE COMMUNICATIONS==========
        #==================================================
        d = Lidar(LS)               #This calls the Lidar function to get distance
        s = Sonar(7,11, s_prev)     #This calls the Sonar funciton to get distance
        s_prev = s;                 #This updates a variable for the sonar funciton
        #This creates a message from all of the data, and formats it according to how its needed
        msg = Formatting(round(RPY[0],4),8) + "/" + Formatting(round(RPY[1],4),8) + "/" + Formatting(round(RPY[2],4),8) + "/" + Formatting(d,6) + "/" + Formatting(s,6)
        
        #===========================
        #=====TCP COMMUNICATION=====
        #===========================
        conn.send(msg.encode())             #This sends the message to the connected TCP client
        
        #=============================
        #====SERIAL COMMUNICATION=====
        #=============================
        
        #print("Pi is Sending:" , msg)
        #ser.write(msg.encode())
        #myData = ser.readline()
        #myData = myData.decode("utf-8","ignore")
        #ser.close()
        
        #=====================
        #=====END OF LOOP=====
        #=====================
        #print (msg)    #Debugging code
        time.sleep(dt)
        