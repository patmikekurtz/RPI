#!/usr/bin/env python3
#############################################################################
# Filename    : Motor.py
# Description : Control Motor with L293D
# Author      : www.freenove.com
# modification: 2023/05/11
########################################################################
from gpiozero import DigitalOutputDevice,PWMOutputDevice
import time
from ADCDevice import *

# define the pins connected to L293D
motoRPin1 = DigitalOutputDevice(27)           # define L293D pin according to BCM Numbering
motoRPin2 = DigitalOutputDevice(17)           # define L293D pin according to BCM Numbering
enablePin = PWMOutputDevice(22,frequency=1000)

# mapNUM function: map the value from a range of mapping to another range.
def mapNUM(value,fromLow,fromHigh,toLow,toHigh):
    return (toHigh-toLow)*(value-fromLow) / (fromHigh-fromLow) + toLow

# motor function: determine the direction and speed of the motor according to the input ADC value input
def motor(speed):

    if (speed > 0):  # make motor turn forward
        motoRPin1.on()        # motoRPin1 output HIGH level
        motoRPin2.off()       # motoRPin2 output LOW level
        print ('Turn Forward...')
    elif (speed < 0): # make motor turn backward
        motoRPin1.off()
        motoRPin2.on()
        print ('Turn Backward...')
    else :
        motoRPin1.off()
        motoRPin2.off()
        print ('Motor Stop...')
    b=mapNUM(abs(speed),0,128,0,100)
    enablePin.value = b / 100.0     # set dc value as the duty cycle
    print ('The PWM duty cycle is %d%%\n'%(abs(speed)*100/127))   # print PMW duty cycle.

def loop():
    start = -50
    while start <= 50:
        print (start)
        motor(start)
        time.sleep(2)
        start = start + 25

def destroy():
    motoRPin1.close()
    motoRPin2.close()
    enablePin.close()

if __name__ == '__main__':  # Program entrance
    print ('Program is starting ... ')
    try:
        loop()
    except KeyboardInterrupt: # Press ctrl-c to end the program.
        destroy()
        print("Ending program")
