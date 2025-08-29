#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import gc

# ===== BCM pins =====
# Motor A
A_IN1 = 17
A_IN2 = 27
# Motor B
B_IN1 = 19
B_IN2 = 26

PWM_FREQ = 1000           # you can also try 2000–20000 if you hear whine
SAFE_START_DELAY = 0.15   # seconds between starting motor A and B
RAMP_STEP = 8
RAMP_DWELL = 0.08

def p(msg): print(msg, flush=True)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

class Motor:
    def __init__(self, in1, in2, pwm_freq=PWM_FREQ):
        self.in1, self.in2 = in1, in2
        GPIO.setup(self.in1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.in2, GPIO.OUT, initial=GPIO.LOW)
        self.pwm1 = GPIO.PWM(self.in1, pwm_freq); self.pwm1.start(0)
        self.pwm2 = GPIO.PWM(self.in2, pwm_freq); self.pwm2.start(0)
        self.coast()

    def forward(self, pct):
        s = max(0, min(100, int(pct)))
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm2.ChangeDutyCycle(0)
        self.pwm1.ChangeDutyCycle(s)

    def backward(self, pct):
        s = max(0, min(100, int(pct)))
        GPIO.output(self.in1, GPIO.LOW)
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(s)

    def coast(self):
        self.pwm1.ChangeDutyCycle(0); self.pwm2.ChangeDutyCycle(0)
        GPIO.output(self.in1, GPIO.LOW); GPIO.output(self.in2, GPIO.LOW)

    def brake(self):
        self.pwm1.ChangeDutyCycle(0); self.pwm2.ChangeDutyCycle(0)
        GPIO.output(self.in1, GPIO.HIGH); GPIO.output(self.in2, GPIO.HIGH)

    def close(self):
        try: self.coast()
        except: pass
        for pwm in (self.pwm1, self.pwm2):
            try: pwm.ChangeDutyCycle(0)
            except: pass
            try: pwm.stop()
            except: pass
        self.pwm1 = None; self.pwm2 = None

def ramp_to(motor, dir_fn, target, step=RAMP_STEP, dwell=RAMP_DWELL):
    cur = 0
    for s in range(0, target + 1, step):
        dir_fn(s); cur = s; time.sleep(dwell)
    return cur

def ramp_down(motor, dir_fn, start, step=RAMP_STEP, dwell=RAMP_DWELL):
    for s in range(start, -1, -step):
        dir_fn(s); time.sleep(dwell)

def main():
    mA = Motor(A_IN1, A_IN2)
    mB = Motor(B_IN1, B_IN2)
    try:
        # Solo tests remain the same
        p("Motor A forward ramp"); top = ramp_to(mA, mA.forward, 100); ramp_down(mA, mA.forward, top); mA.brake(); time.sleep(0.4)
        p("Motor A backward ramp"); top = ramp_to(mA, mA.backward, 100); ramp_down(mA, mA.backward, top); mA.coast(); time.sleep(0.4)

        p("Motor B forward ramp"); top = ramp_to(mB, mB.forward, 100); ramp_down(mB, mB.forward, top); mB.brake(); time.sleep(0.4)
        p("Motor B backward ramp"); top = ramp_to(mB, mB.backward, 100); ramp_down(mB, mB.backward, top); mB.coast(); time.sleep(0.4)

        # === Both together with staggered soft-start ===
        target = 60  # start conservative; increase if stable
        p(f"Both forward soft-start to {target}% (staggered)")
        # Start A
        mA.forward(0); mB.forward(0); time.sleep(0.05)
        ramp_to(mA, mA.forward, target)
        time.sleep(SAFE_START_DELAY)
        ramp_to(mB, mB.forward, target)
        time.sleep(2.0)

        # Soft stop (coast) rather than hard brake to avoid spikes
        ramp_down(mA, mA.forward, target, step=RAMP_STEP); ramp_down(mB, mB.forward, target, step=RAMP_STEP)
        mA.coast(); mB.coast(); time.sleep(0.3)

        p(f"Both opposite dirs soft-start to {target}% (staggered)")
        ramp_to(mA, mA.forward, target)
        time.sleep(SAFE_START_DELAY)
        ramp_to(mB, mB.backward, target)
        time.sleep(2.0)
        ramp_down(mA, mA.forward, target); ramp_down(mB, mB.backward, target)
        mA.coast(); mB.coast(); time.sleep(0.3)

        p("Done")

    except KeyboardInterrupt:
        p("Interrupted")
    finally:
        mA.close(); mB.close(); gc.collect(); time.sleep(0.05)
        GPIO.cleanup(); p("GPIO cleanup done")

if __name__ == "__main__":
    main()
