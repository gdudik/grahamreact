from machine import Pin, SPI
import time

# --- SPI + Sensor Setup ---
spi = SPI(0, baudrate=1_000_000, polarity=0, phase=0,
          sck=Pin(18), mosi=Pin(19), miso=Pin(16))
cs = Pin(17, Pin.OUT, value=1)

# --- Sensor Registers ---
REG_WHO_AM_I = 0x75
REG_PWR_MGMT0 = 0x4E
REG_ACCEL_CONFIG0 = 0x50
REG_FIFO_CONFIG = 0x16
REG_FIFO_CONFIG1 = 0x5F
REG_FIFO_CONFIG2 = 0x60
REG_FIFO_CONFIG3 = 0x61
REG_INT_CONFIG = 0x14
REG_FIFO_COUNT = 0x2E
REG_FIFO_DATA = 0x30
REG_SIGNAL_PATH_RESET = 0x4B
WHO_AM_I_EXPECTED = 0x47

# --- FIFO Settings ---
FIFO_MODE_STREAM = 0x00
FIFO_ACCEL_ENABLE = 0x10
FIFO_TIMESTAMP_ENABLE = 0x40  # Bit 6 enables 2-byte timestamp

# --- Buffer Setup ---
SAMPLES = 10_000  # 5 seconds * 2 kHz
BYTES_PER_SAMPLE = 9  # 2 bytes timestamp + 6 bytes accel + 1 byte header
buf = bytearray(SAMPLES * BYTES_PER_SAMPLE)

# --- Helpers ---
def read_reg(reg, n=1):
    cs.value(0)
    spi.write(bytearray([reg | 0x80]))
    data = spi.read(n)
    cs.value(1)
    return data

def write_reg(reg, val):
    cs.value(0)
    spi.write(bytearray([reg & 0x7F, val]))
    cs.value(1)

def read_fifo_bytes(n):
    cs.value(0)
    spi.write(bytearray([REG_FIFO_DATA | 0x80]))
    data = spi.read(n)
    cs.value(1)
    return data

def fifo_count():
    count_bytes = read_reg(REG_FIFO_COUNT, 2)
    return (count_bytes[0] << 8) | count_bytes[1]

# --- Sensor Init ---
write_reg(0x76, 0x00)  # Ensure we're in Register Bank 0
whoami_bytes = read_reg(REG_WHO_AM_I, 1)
whoami = whoami_bytes[0]
print("WHO_AM_I:", hex(whoami))
if whoami != WHO_AM_I_EXPECTED:
    raise RuntimeError("ICM-42688-P not detected")

write_reg(REG_SIGNAL_PATH_RESET, 0x07)  # Reset signal paths
write_reg(0x11, 0x01)  # INTF_CONFIG1: force internal oscillator

# Delay to ensure reset completes
print("Waiting for sensor startup...")
time.sleep_ms(100)
write_reg(REG_PWR_MGMT0, 0x0F)  # Enable accel/gyro
pwr = read_reg(REG_PWR_MGMT0, 1)[0]
print("PWR_MGMT0:", hex(pwr))
time.sleep_ms(100)  # Allow accel to spin up
write_reg(REG_ACCEL_CONFIG0, 0x06)  # 2 kHz ODR
write_reg(0x51, 0x01)  # ACCEL_CONFIG1: enable DLPF, BW ~246Hz
odr = read_reg(REG_ACCEL_CONFIG0, 1)[0]
print("ACCEL_CONFIG0:", hex(odr))

write_reg(REG_FIFO_CONFIG, 0x40 | FIFO_MODE_STREAM)  # Enable header mode + stream
write_reg(REG_FIFO_CONFIG1, FIFO_ACCEL_ENABLE | FIFO_TIMESTAMP_ENABLE)  # Accel + timestamp
fifo_cfg1 = read_reg(REG_FIFO_CONFIG1, 1)[0]
print("FIFO_CONFIG1:", hex(fifo_cfg1))
write_reg(REG_FIFO_CONFIG2, 0x00)  # Watermark low byte (optional)
write_reg(REG_FIFO_CONFIG3, 0x00)  # Watermark high byte
write_reg(REG_INT_CONFIG, 0x00)  # Push-pull active high INT
write_reg(0x70, 0x03)  # FIFO_RECORD_CONFIG: enable FIFO headers + sensor record structure
write_reg(0x03, 0x40)  # USER_CTRL: enable FIFO engine

print("Initialized. Waiting 100 ms before logging...")
time.sleep_ms(100)

print("Logging...")

# One-shot sanity check of accel data registers
def read_accel_registers():
    raw = read_reg(0x1F, 6)
    # Convert to signed 16-bit values manually for MicroPython compatibility
    results = []
    for i in range(0, 6, 2):
        val = (raw[i] << 8) | raw[i+1]
        signed_val = val if val < 32768 else val - 65536
        results.append(signed_val)
    return results

print("Accel registers:", read_accel_registers())

# Debug: print FIFO count growth
for _ in range(10):
    print("FIFO count:", fifo_count())
    time.sleep_ms(100)

# --- Logging ---
start = time.ticks_us()
end_time = time.ticks_add(start, 5_000_000)
idx = 0

while time.ticks_diff(end_time, time.ticks_us()) > 0 and idx < SAMPLES:
    count = fifo_count()
    while count >= 9 and idx < SAMPLES:  # Changed from 10 to 9 bytes
        packet = read_fifo_bytes(9)  # Read 9 bytes: 1 header + 2 timestamp + 6 accel
        header = packet[0]
        
        # ICM-42688-P FIFO header format: bit 6 indicates accelerometer data
        if not (header & 0x40):  # Check if accelerometer bit is set
            print("Skipped header:", hex(header))
            count -= 1
            continue
            
        # Debug: print first few valid headers to verify
        if idx < 5:
            print("Valid header", idx, ":", hex(header))
            
        offset = idx * BYTES_PER_SAMPLE
        buf[offset:offset+9] = packet  # Store entire packet including header
        idx += 1
        count -= 9  # Changed from 10 to 9

print("Done. Dumping log:")

# --- Write to binary file ---
with open("accel_log.bin", "wb") as f:
    f.write(buf[:idx * BYTES_PER_SAMPLE])

print("Wrote", idx, "samples to accel_log.bin")
