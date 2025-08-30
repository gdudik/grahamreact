from machine import Pin, SPI, PWM
import time


# Overall Buffer Management
overall_buffer = bytearray()
FLUSH_THRESH = 4 * 1024


int_pin = Pin(22, Pin.IN)

# SPI config
spi = SPI(0, baudrate=2_000_000, polarity=0, phase=0, sck=Pin(18), mosi=Pin(19), miso=Pin(16))
cs = Pin(17, Pin.OUT, value=1)

fifo_ready = False
led = Pin(25, Pin.OUT, value=0)


# Register read/write
def read_reg(reg, n=1):
    cs.value(0)
    spi.write(bytearray([reg | 0b10000000])) #(register address bit 7 high to read)
    data = spi.read(n)
    cs.value(1)
    #print(' '.join(f'{b:08b}' for b in data))
    return data

def write_reg(reg, val):
    cs.value(0)
    spi.write(bytearray([reg & 0b01111111, val])) #(register address bit 7 low to write)
    cs.value(1)

def set_reg_bank(bank):
    write_reg(0x76, bank & 0x07)
    time.sleep_us(10)

def reset():
    set_reg_bank(0x00)
    write_reg(0x4E, 0x00)
    ext_reg = read_reg(0x11)[0] #DEVICE_CONFIG
    write_reg(0x11,ext_reg & 0x01)
    time.sleep_ms(1)

def setup():
    reset()
    write_reg(0x4F, 0b00000101) # GYRO_CONFIG0: FS_SEL=000, ODR=0101 (2kHz)
    write_reg(0x50, 0b00000101) # ACCEL_CONFIG0: FS_SEL=000, ODR=0101 (2kHz)
    write_reg(0x53, 0b00000101) # ACCEL_CONFIG1: ACCEL_UI_FILT_ORD=00 (1st order) --bit 0 is reserved and must be set to 1
    write_reg(0x52, 0b01100000) # GYRO_ACCEL_CONFIG0: ACCEL_UI_FILT_BW= 6, GYRO_UI_FILT_BW= 0
    write_reg(0x16, 0b01000000) # FIFO_CONFIG: FIFO_MODE=01 CHANGE ME BACK!!!!
    write_reg(0x5F, 0b00001111) # FIFO_CONFIG1
    write_reg(0x60, 0x00) # FIFO_CONFIG2: lower byte of threshold (1024 bytes)
    write_reg(0x61, 0x02) # FIFO_CONFIG3: upper byte of threshold 
    write_reg(0x54, 0b00100101) # TMST_CONFIG
    write_reg(0x4C, 0b00110000) # INTF_CONFIG0
    write_reg(0x63, 0b00000100) # INT_CONFIG0

    write_reg(0x14, 0b00000111) # INT_CONFIG: INT1_POLARITY = 1; INT1_DRIVE_CIRCUIT = 1, INT1_MODE = 1
    write_reg(0x64, 0b00000000) # INT_CONFIG1: INT_ASYNC_RESET = 0
    write_reg(0x65, 0b00000100) # INT_SOURCE0: Bit 2 = FIFO_THS_INT1_EN
    
    set_reg_bank(0x02)
    write_reg(0x03, 0b00110001) # ACCEL_CONFIG_STATIC2: ACCEL_AAF_DIS=1
    set_reg_bank(0x01)
    write_reg(0x7B,0b00000100) # PIN9_FUNCTION = 10 (CLKIN)
    set_reg_bank(0x00)
    
    write_reg(0x4D, 0b10010101) #INTF_CONFIG1
    write_reg(0x4B, 0b00000010) # SIGNAL PATH RESET
    read_reg(0x30, 2064) #DRAIN THE FIFO

def start():
    write_reg(0x4E, 0b00001111)
    
    
def get_count():
    fifo_count_raw = read_reg(0x2E, 2)
    fifo_count = fifo_count_raw[0] << 8 | fifo_count_raw[1]
    print(fifo_count)

def parse_fifo_temp(byte_val):
    if byte_val >= 128:
        byte_val -= 256
    return (byte_val / 2.07) + 25

def parse_packet3(pkt):
    def to_int16(msb, lsb):
        val = (msb << 8) | lsb
        if val & 0x8000:
            val -= 0x10000
        return val

    header = pkt[0]

    ax = to_int16(pkt[1], pkt[2])
    ay = to_int16(pkt[3], pkt[4])
    az = to_int16(pkt[5], pkt[6])
    gx = to_int16(pkt[7], pkt[8])
    gy = to_int16(pkt[9], pkt[10])
    gz = to_int16(pkt[11], pkt[12])
    temp = pkt[13]
    ts = (pkt[14] << 8) | pkt[15]

    return {
        "header": header,
        "ax_g": ax / 2048,
        "ay_g": ay / 2048,
        "az_g": az / 2048,
        "gx_dps": gx / 16.4,
        "gy_dps": gy / 16.4,
        "gz_dps": gz / 16.4,
        "temp_C": parse_fifo_temp(pkt[13]),
        "timestamp": ts
    }

def parse_fifo_dump(byte_count):
    dump_bytes = read_reg(0x30, byte_count)
    packets = []
    i = 0
    while i + 16 <= len(dump_bytes):
        pkt = dump_bytes[i:i+16]
        if pkt[0] & 0xC0 == 0x40:  # Check HEADER_ACCEL (bit 6) or HEADER_GYRO (bit 5)
            packets.append(parse_packet3(pkt))
        i += 16
    print("header,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_C,timestamp")
    for pkt in packets:
        print("{},{:.6f},{:.6f},{:.6f},{:.3f},{:.3f},{:.3f},{:.2f},{}".format(
            pkt['header'],
            pkt['ax_g'], pkt['ay_g'], pkt['az_g'],
            pkt['gx_dps'], pkt['gy_dps'], pkt['gz_dps'],
            pkt['temp_C'], pkt['timestamp']
        ))

# def fifo_interrupt(pin):
#     global fifo_ready
#     fifo_ready = True
#     led.value(pin.value())
#     #int_pin.irq(handler=None)  # disable further interrupts

# def check_and_drain():
#     global fifo_ready
#     if fifo_ready:
#         fifo_ready = False
#         drain_fifo(0)
#         #int_pin.irq(trigger=Pin.IRQ_HIGH_LEVEL, handler=fifo_interrupt)  # re-enable


def drain_fifo(_):
    global overall_buffer
    int_status = read_reg(0x2D)[0]
    if not int_status & 0x04:  # Bit 2 = FIFO_THS_INT
        #print("Drain called, but FIFO_THS_INT not set (INT_STATUS = {:08b})".format(int_status))
        return

    fifo_count_raw = read_reg(0x2E, 2)
    fifo_count = (fifo_count_raw[0] << 8) | fifo_count_raw[1]
    if fifo_count >= 16:
        #print("FIFO count:", fifo_count)
        dump = read_reg(0x30,fifo_count)
        # overall_buffer += dump
        return dump   
        

# def run_polling():
#     global overall_buffer
#     start = time.ticks_us()
#     duration = 4_000_000
#     try:
#         while time.ticks_diff(time.ticks_us(), start) < duration:
#             if int_pin.value() == 1:
#                 drain_fifo(0)
#             time.sleep_us(50)
#         print('2 seconds expired')
#         print(len(overall_buffer))
#         with open("overall_buffer.bin", "wb") as f:
#             f.write(overall_buffer)
#             print(f.tell())
#     except KeyboardInterrupt:
#         print(len(overall_buffer))

def run_polling():
    with open("overall_buffer.bin","wb") as f:
        buf_a = bytearray()
        buf_b = bytearray()
        active, writing = buf_a, buf_b
        start = time.ticks_us()
        while time.ticks_diff(time.ticks_us(), start) < 5_000_000:
            if int_pin.value():
                dump = drain_fifo(0)  # your read_reg(0x30,…)
                if dump:
                    active.extend(dump)
                    # once active crosses flush threshold:
                    if len(active) >= FLUSH_THRESH:
                        # swap buffers instantly
                        active, writing = writing, active
                        # clear new active
                        active[:] = b''
                        # blocking write of the full buffer
                        f.write(writing)
            time.sleep_us(50)
        if active:
            f.write(active)
        size = f.tell() / 16
        print(size)




#Interrupt

# int_pin.irq(trigger=Pin.IRQ_HIGH_LEVEL, handler=fifo_interrupt)
