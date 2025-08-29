import RPi.GPIO as GPIO
import time

# Pin definitions
IN1 = 17  # GPIO pin connected to IN1
IN2 = 27  # GPIO pin connected to IN2

# Setup
GPIO.setmode(GPIO.BCM)  # use BCM pin numbering
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

def motor_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

def motor_backward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)

def motor_stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

try:
    print("Motor forward")
    motor_forward()
    time.sleep(2)

    print("Motor backward")
    motor_backward()
    time.sleep(2)

    print("Motor stop")
    motor_stop()
    time.sleep(1)

finally:
    GPIO.cleanup()
    print("GPIO cleanup done")
