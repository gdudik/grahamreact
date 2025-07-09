from machine import Pin, SPI
import time
from lis3dh_spi import LIS3DH_SPI

# SPI on GP18 (SCK), GP19 (MOSI), GP16 (MISO)
spi = SPI(0, baudrate=5_000_000, polarity=1, phase=1,
          sck=Pin(18), mosi=Pin(19), miso=Pin(16))
cs = Pin(17, Pin.OUT)

accel = LIS3DH_SPI(spi, cs)

print("LIS3DH detected and initialized.")

while True:
    x, y, z = accel.read_accel()
    print("X: {:.2f}  Y: {:.2f}  Z: {:.2f}  (m/s²)".format(x, y, z))
    time.sleep(0.2)
