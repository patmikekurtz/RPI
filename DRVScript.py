#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# ===== Pin map (BCM) =====
# Motor A (DRV8833 A-side)
A_IN1 = 17   # physical pin 11
A_IN2 = 27   # physical pin 13
# Motor B (DRV8833 B-side)
B_IN1 = 19   # physical pin 35
B_IN2 = 26   # physical pin 37

PWM_FREQ = 1000  # 1 kHz

def p(msg):
    print(msg, flush=True)

class Motor:
    def __init__(self, in1, in2, pwm_freq=PWM_FREQ):
        self.in1 = in1
        self.in2 = in2
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        self.pwm1 = GPIO.PWM(self.in1, pwm_freq)
        self.pwm2 = GPIO.PWM(self.in2, pwm_freq)
        self.pwm1.start(0)
        self.pwm2.start(0)
        self.coast()

    def forward(self, speed_pct):
        s = max(0, min(100, int(speed_pct)))
        # PWM on IN1, IN2 LOW
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm2.ChangeDutyCycle(0)
        self.pwm1.ChangeDutyCycle(s)

    def backward(self, speed_pct):
        s = max(0, min(100, int(speed_pct)))
        # PWM on IN2, IN1 LOW
        GPIO.output(self.in1, GPIO.LOW)
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(s)

    def coast(self):
        # Both LOW -> Hi-Z, motor coasts
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)

    def brake(self):
        # Both HIGH -> active brake
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.HIGH)

    def stop(self):
        # Alias for coast
        self.coast()

    def cleanup(self):
        self.coast()
        self.pwm1.stop()
        self.pwm2.stop()

def ramp(motor, func_name, start, stop, step=10, dwell=0.1, label=""):
    """
    func_name: 'forward' or 'backward'
    """
    fn = getattr(motor, func_name)
    if start <= stop:
        rng = range(start, stop + 1, step)
    else:
        rng = range(start, stop - 1, -step)
    for s in rng:
        p(f"{label}{func_name} -> {s}%")
        fn(s)
        time.sleep(dwell)

def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    mA = Motor(A_IN1, A_IN2, PWM_FREQ)
    mB = Motor(B_IN1, B_IN2, PWM_FREQ)

    try:
        # --- Motor A test ---
        p("Motor A: Coast to start"); mA.coast(); time.sleep(0.4)
        p("Motor A: Ramp FORWARD up");   ramp(mA, "forward", 0, 100, step=20, dwell=0.2, label="[A] ")
        p("Motor A: Ramp FORWARD down"); ramp(mA, "forward", 100, 0, step=20, dwell=0.2, label="[A] ")
        p("Motor A: Brake"); mA.brake(); time.sleep(0.5)
        p("Motor A: Ramp BACKWARD up");   ramp(mA, "backward", 0, 100, step=20, dwell=0.2, label="[A] ")
        p("Motor A: Ramp BACKWARD down"); ramp(mA, "backward", 100, 0, step=20, dwell=0.2, label="[A] ")
        p("Motor A: Coast"); mA.coast(); time.sleep(0.5)

        # --- Motor B test ---
        p("Motor B: Coast to start"); mB.coast(); time.sleep(0.4)
        p("Motor B: Ramp FORWARD up");   ramp(mB, "forward", 0, 100, step=20, dwell=0.2, label="[B] ")
        p("Motor B: Ramp FORWARD down"); ramp(mB, "forward", 100, 0, step=20, dwell=0.2, label="[B] ")
        p("Motor B: Brake"); mB.brake(); time.sleep(0.5)
        p("Motor B: Ramp BACKWARD up");   ramp(mB, "backward", 0, 100, step=20, dwell=0.2, label="[B] ")
        p("Motor B: Ramp BACKWARD down"); ramp(mB, "backward", 100, 0, step=20, dwell=0.2, label="[B] ")
        p("Motor B: Coast"); mB.coast(); time.sleep(0.5)

        # --- Both motors together ---
        p("Both motors: forward 50% for 2s")
        mA.forward(50); mB.forward(50); time.sleep(2.0)

        p("Both motors: opposite directions 60% for 2s")
        mA.forward(60); mB.backward(60); time.sleep(2.0)

        p("Both motors: brake for 0.7s")
        mA.brake(); mB.brake(); time.sleep(0.7)

        p("Both motors: coast and done")
        mA.coast(); mB.coast(); time.sleep(0.5)

    except KeyboardInterrupt:
        p("Interrupted")
    finally:
        mA.cleanup()
        mB.cleanup()
        GPIO.cleanup()
        p("GPIO cleanup done")

if __name__ == "__main__":
    main()
