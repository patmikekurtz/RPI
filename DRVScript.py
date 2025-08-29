#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import gc

# ===== BCM pin map =====
# Motor A (DRV8833 A-side)
A_IN1 = 17   # physical pin 11
A_IN2 = 27   # physical pin 13
# Motor B (DRV8833 B-side)
B_IN1 = 19   # physical pin 35
B_IN2 = 26   # physical pin 37

PWM_FREQ = 1000  # 1 kHz

def p(msg):
    print(msg, flush=True)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

class Motor:
    def __init__(self, in1, in2, pwm_freq=PWM_FREQ):
        self.in1 = in1
        self.in2 = in2
        GPIO.setup(self.in1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.in2, GPIO.OUT, initial=GPIO.LOW)
        self.pwm1 = GPIO.PWM(self.in1, pwm_freq)
        self.pwm2 = GPIO.PWM(self.in2, pwm_freq)
        self.pwm1.start(0)
        self.pwm2.start(0)
        self.coast()

    def forward(self, speed_pct):
        s = max(0, min(100, int(speed_pct)))
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm2.ChangeDutyCycle(0)
        self.pwm1.ChangeDutyCycle(s)

    def backward(self, speed_pct):
        s = max(0, min(100, int(speed_pct)))
        GPIO.output(self.in1, GPIO.LOW)
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(s)

    def coast(self):
        # Hi-Z (coast)
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)

    def brake(self):
        # Active brake
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.HIGH)

    def close(self):
        # Stop PWM and release references BEFORE GPIO.cleanup()
        try:
            self.coast()
        except Exception:
            pass
        for pwm in (self.pwm1, self.pwm2):
            try:
                pwm.ChangeDutyCycle(0)
            except Exception:
                pass
            try:
                pwm.stop()
            except Exception:
                pass
        self.pwm1 = None
        self.pwm2 = None

def ramp(motor, func_name, start, stop, step=10, dwell=0.1, label=""):
    fn = getattr(motor, func_name)
    rng = range(start, stop + (1 if start <= stop else -1), step if start <= stop else -step)
    for s in rng:
        p(f"{label}{func_name} -> {s}%")
        fn(s)
        time.sleep(dwell)

def main():
    mA = Motor(A_IN1, A_IN2, PWM_FREQ)
    mB = Motor(B_IN1, B_IN2, PWM_FREQ)

    try:
        # --- Motor A test ---
        p
