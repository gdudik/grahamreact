from machine import Pin, SPI
import time
import struct
from lis3dh_spi import LIS3DH_SPI

# --- Setup SPI and Pins ---
spi = SPI(0, baudrate=5_000_000, polarity=1, phase=1,
          sck=Pin(18), mosi=Pin(19), miso=Pin(16))
cs = Pin(17, Pin.OUT)
gun = Pin(15, Pin.IN, Pin.PULL_UP)

accel = LIS3DH_SPI(spi, cs)

# --- Constants ---
GUN_MARKER = 0x7FFF  # Special int16 flag for gun
SAMPLE_RATE = 1344  # Hz
DT_US = int(1_000_000 / SAMPLE_RATE)
LOG_DURATION_US = 4_000_000  # 4 seconds

# --- Run Logger ---
gun_fired = False
start = time.ticks_us()

with open("accel_x_log.bin", "wb") as f:
    while time.ticks_diff(time.ticks_us(), start) < LOG_DURATION_US:
        x_batch = accel.read_fifo_raw_x()
        t_base = time.ticks_us()

        if not gun_fired and gun.value() == 0:
            gun_fired = True
            t_gun = time.ticks_us()
            f.write(struct.pack("<hI", GUN_MARKER, t_gun))

        for i, x in enumerate(x_batch):
            ts = time.ticks_add(t_base, i * DT_US)
            f.write(struct.pack("<hI", x, ts))

print("4-second binary log complete.")
