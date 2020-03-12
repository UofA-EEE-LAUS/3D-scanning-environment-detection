# 3D-scanning-environment-detection
University of Adelaide Summer Research Internship 2019

Team Members: Yingzhe Guo, Abdul Rahim Mohammad, Swapnil Srivastava, Glenn Walsh

# Project Description
From the Raspberry Pi, a suite of sensors that detect range will be used. This
includes an ultrasonic sensor, a laser sensor, an IR sensor, and a ZX range sensor. The Acceleration, angular
velocity, and magnetic field are obtained from the IMU. This information is then fused with a complementray filter. The distance to object will then be passed through a filter to obtain a more appropriate reading. All of this data will be packaged together to be sent as environmental data to be used by other modules.

# Main File Elements
Main File (Main.py):

-IMU and Distance Fusion

-TCP Transmission from Raspberry Pi (to Matlab / Simulink)

# Receive Files

Receive transmission from Raspberry Pi to Matlab  (TCPPackageRx.m)

Receive transmission from Raspberry Pi to Simulink (Unpack.slx)

Receive transmission from Raspberry Pi to Arduino via Serial (myduino.ino)

# SLAM Files
Base SLAM Algorithm (map.m)

SLAM Algorithm with cone spread (MapWithRays.m)

# Magnetometer Calibration
I didn't manually write any lengthy magnetometer calibration algorithms, but the following websites are great resources for learning about it. If you have any interest in this topic, these are all must reads:

https://www.fierceelectronics.com/components/compensating-for-tilt-hard-iron-and-soft-iron-effects

https://learn.adafruit.com/how-to-fuse-motion-sensor-data-into-ahrs-orientation-euler-quaternions/magnetic-calibration-with-motioncal

https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/

# FILTERS

Filters used for processing SONAR and LIDAR data, these files are optional. 

Kalmanfilter.py for Kalman filter and Alphatrimmer.py for alphatrimmer  

# MODULES

Individual files for all the sensors.

USS.py for SONAR, LIDAR.py for LIDAR, IMU.py for IMU, SERIAL_COMM.py for serial communication.
