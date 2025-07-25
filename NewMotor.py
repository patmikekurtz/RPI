import RPi.GPIO as GPIO
import time

# Motor A pins
IN1 = 23
IN2 = 24
ENA = 18

# Motor B pins
IN3 = 27
IN4 = 22
ENB = 13

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# PWM setup
pwmA = GPIO.PWM(ENA, 1000)
pwmB = GPIO.PWM(ENB, 1000)
pwmA.start(70)  # 70% speed
pwmB.start(70)

def motorA_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

def motorA_backward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)

def motorA_stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

def motorB_forward():
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def motorB_backward():
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def motorB_stop():
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

try:
    motorA_forward()
    motorB_backward()
    time.sleep(3)

    motorA_backward()
    motorB_forward()
    time.sleep(3)

    motorA_stop()
    motorB_stop()

finally:
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()
