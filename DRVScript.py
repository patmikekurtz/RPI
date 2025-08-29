#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import sys

IN1 = 17  # BCM
IN2 = 27  # BCM
PWM_FREQ = 1000

def p(msg):
    print(msg, flush=True)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

pwm1 = GPIO.PWM(IN1, PWM_FREQ)
pwm2 = GPIO.PWM(IN2, PWM_FREQ)
pwm1.start(0)
pwm2.start(0)

def coast():
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

def brake():
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.HIGH)

def forward(speed_pct):
    s = max(0, min(100, int(speed_pct)))
    GPIO.output(IN2, GPIO.LOW)
    pwm2.ChangeDutyCycle(0)
    pwm1.ChangeDutyCycle(s)

def backward(speed_pct):
    s = max(0, min(100, int(speed_pct)))
    GPIO.output(IN1, GPIO.LOW)
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(s)

def ramp(func, start, stop, step=10, dwell=0.1):
    if start <= stop:
        rng = range(start, stop + 1, step)
    else:
        rng = range(start, stop - 1, -step)
    for s in rng:
        p(f"{func.__name__} -> {s}%")
        func(s)
        time.sleep(dwell)
#
try:
    p("Coast to start")
    coast(); time.sleep(0.5)

    p("Ramp FORWARD up")
    ramp(forward, 0, 100, step=20, dwell=0.2)
    time.sleep(0.3)

    p("Ramp FORWARD down")
    ramp(forward, 100, 0, step=20, dwell=0.2)
    time.sleep(0.3)

    p("Brake")
    brake(); time.sleep(0.6)

    p("Ramp BACKWARD up")
    ramp(backward, 0, 100, step=20, dwell=0.2)
    time.sleep(0.3)

    p("Ramp BACKWARD down")
    ramp(backward, 100, 0, step=20, dwell=0.2)

    p("Coast and done")
    coast(); time.sleep(0.5)

except KeyboardInterrupt:
    p("Interrupted")
finally:
    coast()
    pwm1.stop(); pwm2.stop()
    GPIO.cleanup()
    p("GPIO cleanup done")
