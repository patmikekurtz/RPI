#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# BCM pins
IN1 = 17  # to DRV8833 IN1
IN2 = 27  # to DRV8833 IN2

PWM_FREQ = 1000  # 1 kHz works well for small DC motors

GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

pwm1 = GPIO.PWM(IN1, PWM_FREQ)
pwm2 = GPIO.PWM(IN2, PWM_FREQ)
pwm1.start(0)
pwm2.start(0)

def coast():
    """Hi-Z both outputs (motor coasts)."""
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

def brake():
    """Both inputs HIGH (active brake)."""
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.HIGH)

def forward(speed_pct):
    """
    Drive forward at speed_pct (0–100).
    DRV8833: IN1 PWM (HIGH side), IN2 LOW.
    """
    speed = max(0, min(100, int(speed_pct)))
    GPIO.output(IN2, GPIO.LOW)
    pwm2.ChangeDutyCycle(0)
    pwm1.ChangeDutyCycle(speed)

def backward(speed_pct):
    """
    Drive backward at speed_pct (0–100).
    DRV8833: IN2 PWM (HIGH side), IN1 LOW.
    """
    speed = max(0, min(100, int(speed_pct)))
    GPIO.output(IN1, GPIO.LOW)
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(speed)

def ramp(func, start=0, stop=100, step=5, dwell=0.05):
    """Generic ramp helper: func is forward/backward."""
    if start <= stop:
        rng = range(start, stop + 1, step)
    else:
        rng = range(start, stop - 1, -step)
    for s in rng:
        func(s)
        time.sleep(dwell)

try:
    print("Coast to start")
    coast()
    time.sleep(0.5)

    print("Ramp FORWARD up")
    ramp(forward, 0, 100, step=10, dwell=0.1)
    time.sleep(0.4)

    print("Ramp FORWARD down")
    ramp(forward, 100, 0, step=10, dwell=0.1)

    print("Brake")
    brake()
    time.sleep(0.5)

    print("Ramp BACKWARD up")
    ramp(backward, 0, 100, step=10, dwell=0.1)
    time.sleep(0.4)

    print("Ramp BACKWARD down")
    ramp(backward, 100, 0, step=10, dwell=0.1)

    print("Coast and done")
    coast()
    time.sleep(0.5)

except KeyboardInterrupt:
    pass
finally:
    coast()
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
    print("GPIO cleanup done")
