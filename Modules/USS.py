# -*- coding: utf-8 -*-
"""
Created on Tue Dec 17 14:48:32 2019

@author: Curry, Galaxy Brain, Swapnil and Chloe
"""

import RPi.GPIO as GPIO #SET UP FOR GPIO
import math
import time

GPIO.setmode(GPIO.BOARD)      #BOARD LAYOUT OF GPIO


def Sonar(TRIG, ECHO, LKV):
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

    while GPIO.input(ECHO)==0:
        pulse_start = time.time()
        if to>100:
            return LKV
        to +=1

    while GPIO.input(ECHO)==1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    sonar = round(distance+1.15, 2)
    return sonar




