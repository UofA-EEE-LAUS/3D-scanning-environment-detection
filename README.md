# 3D-scanning-environment-detection
University of Adelaide Summer Research Internship 2019

Team Members: Yingzhe Guo, Abdul Rahim Mohammad, Swapnil Srivastava, Glenn Walsh

# Project Description
From the Raspberry Pi, a suite of sensors that detect range will be used. This
includes an ultrasonic sensor, a laser sensor, an IR sensor, and a ZX range sensor. The Acceleration, angular
velocity, and magnetic field are obtained from the IMU. This information is then fused with a complementray filter. The distance to object will then be passed through a filter to obtain a more appropriate reading. All of this data will be packaged together to be sent as environmental data to be used by other modules.

# Scanning Elements
Main File (Main.py):

-IMU and Distance Fusion

-Transmission from Raspberry Pi to Matlab / Simulink

Receive transmission from Raspberry Pi to Matlab

Receive transmission from Raspberry Pi to Simulink

Transmission from Raspberry Pi to Arduino via Serial

Hard Iron Calibration for the Magnometer

Base SLAM Algorithm

SLAM Algorithm with cone spread
