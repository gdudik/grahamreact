
from machine import Pin, SPI
import time
import struct

STANDARD_GRAVITY = 9.806

# Register addresses
WHO_AM_I = const(0x0F)
CTRL_REG1 = const(0x20)
CTRL_REG4 = const(0x23)
CTRL_REG5 = const(0x24)
FIFO_CTRL = const(0x2E)
FIFO_SRC = const(0x2F)
OUT_X_L = const(0x28)

# Config values
ODR_1344HZ = 0b1001 << 4
ENABLE_XYZ = 0x07
CTRL_REG1_VALUE = ODR_1344HZ | ENABLE_XYZ

HIGH_RES_BDU = 0b10001000  # HR=1, BDU=1
RANGE_2G = 0b00 << 4
CTRL_REG4_VALUE = HIGH_RES_BDU | RANGE_2G

FIFO_ENABLE = 0b01000000  # FIFO enable bit in CTRL_REG5
FIFO_MODE_STREAM = 0b10000000  # Stream mode in FIFO_CTRL

class LIS3DH_SPI:
    def __init__(self, spi, cs):
        self.spi = spi
        self.cs = cs
        self.cs.init(Pin.OUT, value=1)

        if self._read_register(WHO_AM_I, 1)[0] != 0x33:
            raise RuntimeError("Failed to detect LIS3DH")

        self._write_register(CTRL_REG1, CTRL_REG1_VALUE)
        self._write_register(CTRL_REG4, CTRL_REG4_VALUE)
        self._write_register(CTRL_REG5, FIFO_ENABLE)
        self._write_register(FIFO_CTRL, FIFO_MODE_STREAM)

        time.sleep(0.01)

    def _read_register(self, reg, length):
        reg |= 0xC0 if length > 1 else 0x80
        result = bytearray(length)
        self.cs(0)
        self.spi.write(bytearray([reg]))
        self.spi.readinto(result)
        self.cs(1)
        return result

    def _write_register(self, reg, val):
        reg &= 0x3F
        self.cs(0)
        self.spi.write(bytearray([reg, val]))
        self.cs(1)

    def read_fifo_x(self):
        fifo_status = self._read_register(FIFO_SRC, 1)[0]
        samples = fifo_status & 0x1F
        buf = []

        for _ in range(samples):
            raw = self._read_register(OUT_X_L, 2)
            x = struct.unpack('<h', raw)[0]
            scale = STANDARD_GRAVITY / 16380
            buf.append(x * scale)

        return buf
    
    def read_fifo_raw_x(self):
        fifo_status = self._read_register(FIFO_SRC, 1)[0]
        samples = fifo_status & 0x1F
        buf = []

        for _ in range(samples):
            raw = self._read_register(OUT_X_L, 2)
            x = struct.unpack('<h', raw)[0]
            buf.append(x)

        return buf
