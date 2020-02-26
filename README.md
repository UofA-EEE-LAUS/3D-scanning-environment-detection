# 3D-scanning-environment-detection
University of Adelaide Summer Research Internship 2019

Team Members: Yingzhe Guo, Abdul Rahim Mohammad, Swapnil Srivastava, Glenn Walsh

# Project Description
From the Raspberry Pi, a suite of sensors that detect range will be used. This
includes an ultrasonic sensor, a laser sensor, an IR sensor, and a ZX range sensor. As Raspberry Pi
is used, an Analogue to Digital Converter may be used to convert the analogue signals from the
sensor suite, to a digital version, such that the Raspberry Pi can use it. The Acceleration, and angular
acceleration are obtained from the IMU. The distance to object will then be
passed through a filter to obtain a more appropriate reading, and sent to be packaged. The reading
from the camera at that timestamp will also be output, this can either be in the form of a singular
colour pixel, a smaller macro shot, or the whole frame.

# Scanning Elements
IMU and Distance Fusion

Transmission from Raspberry Pi to Matlab / Simulink

Receive transmission from Raspberry Pi to Matlab

Receive transmission from Raspberry Pi to Simulink

Transmission from Raspberry Pi to Arduino via Serial

Hard Iron Calibration for the Magnometer
