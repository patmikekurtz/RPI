#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# Pin numbers (BCM mode)
BTN1 = 27
BTN2 = 22

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BTN1, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # button to GND
    GPIO.setup(BTN2, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # button to GND

def wait_for_button():
    print("Waiting for a button press on GPIO 27 or 22...")
    while True:
        if GPIO.input(BTN1) == GPIO.LOW:   # active low
            print("Button on GPIO 27 pressed!")
            break
        if GPIO.input(BTN2) == GPIO.LOW:
            print("Button on GPIO 22 pressed!")
            break
        time.sleep(0.05)  # debounce polling delay

if __name__ == "__main__":
    try:
        setup()
        wait_for_button()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        GPIO.cleanup()
