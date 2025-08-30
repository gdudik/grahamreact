from machine import Pin, SPI
import time

BANK_SELECT_REG = 0x76
PWR_MGMT_0_REG = 0x4E
RESET_REG = 0x11
WHO_AM_I_REG = 0x75

class icm42688:
    def __init__(self, spi_id, baudrate, sck_pin, mosi_pin, miso_pin, cs_pin):
        self.spi = SPI(spi_id, baudrate=baudrate, polarity=0, phase=0,sck=Pin(sck_pin),mosi=Pin(mosi_pin), miso=Pin(miso_pin))
        self.cs = Pin(cs_pin, Pin.OUT, value=1)

    def read_reg(self, reg, n=1, into=None):
        self.cs.value(0)
        self.spi.write(bytearray([reg | 0b10000000])) #(register address bit 7 high to read)
        if into:
            self.spi.readinto(into)
            data = into
        else:
            data = self.spi.read(n)
        self.cs.value(1)
        return data

    def write_reg(self, reg, val):
        self.cs.value(0)
        self.spi.write(bytearray([reg & 0b01111111, val])) #(register address bit 7 low to write)
        self.cs.value(1)

    def set_reg_bank(self, bank):
        self.write_reg(BANK_SELECT_REG, bank & 0x07)
        time.sleep_us(10)

    def reset(self):
        self.set_reg_bank(0x00)
        self.write_reg(PWR_MGMT_0_REG, 0x00)
        ext_reg = self.read_reg(RESET_REG)[0] #DEVICE_CONFIG
        self.write_reg(RESET_REG,ext_reg & 0x01)
        time.sleep_ms(10)

    def who_am_i(self):
        self.set_reg_bank(0x00)
        return self.read_reg(WHO_AM_I_REG)
