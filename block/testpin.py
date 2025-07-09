from machine import Pin
import time

pin = Pin(15, Pin.IN, Pin.PULL_UP)

while True:
    print("Pin value:", pin.value())
    time.sleep(0.1)
