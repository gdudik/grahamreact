from lib.icm42688_SPI import icm42688
import time
import machine
from machine import Pin
import gc
import micropython

## THRESHOLD
ACCEL_THRESHOLD_MEN = micropython.const(0.5)
ACCEL_THRESHOLD_WOMEN = micropython.const(8)
HYST = micropython.const(0.2)
RISE_STREAK_N = micropython.const(2)
accel_threshold = None
runner_started = False
runner_started_ts = None
rising_count = 0
prev_impulse = 0
reaction_time_timestamp = None
in_window = False


## SPI CONFIG
SPI_ID = micropython.const(0)
BAUDRATE = micropython.const(24_000_000)
SCK_PIN = micropython.const(18)
MOSI_PIN = micropython.const(19)
MISO_PIN = micropython.const(16)
CS_PIN = micropython.const(17)

imu = icm42688(SPI_ID, BAUDRATE, SCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN)


## BUFFER
DURATION_S = micropython.const(5)
SAMPLE_RATE_HZ = micropython.const(2048)
BYTES_PER_SAMP = micropython.const(16)
OVERHEAD = micropython.const(4096 * 2)
TOTAL_BYTES = DURATION_S * SAMPLE_RATE_HZ * BYTES_PER_SAMP + OVERHEAD
wp = 0 
buf = None
int_counter = 0
fifo_ready = False



ts_last = None
ts_rollovers = 0


## FIFO THRESHOLD INTERRUPT PIN
THRESHOLD_PIN = micropython.const(22) #Driven high by imu when threshold is hit, push/pull config
int_pin = Pin(THRESHOLD_PIN, Pin.IN)

## GUN INTERRUPT PIN
## Input that gun has fired
GUN_PIN = micropython.const(10)
gun_int_pin = None
gun_timestamp = None
gun_triggered = False

## ALERT PIN
## Output that a false start has occurred
ALERT_PIN = micropython.const(15)
fs_alert = Pin(ALERT_PIN, Pin.OUT, Pin.PULL_DOWN)

GUN_FIRED_PIN = micropython.const(12)
gun_fired = Pin(GUN_FIRED_PIN, Pin.OUT)

## ABORT PIN
## Abort the run early due to operator cancel
ABORT_PIN = micropython.const(27)
run_aborted = Pin(ABORT_PIN, Pin.IN)
# Push/pull driven by rpi controller. No pullup/pulldown necessary

## FIFO THRESHOLD INTERRUPT HANDLER
@micropython.native
def threshold_interrupt(pin):
    global int_counter, fifo_ready
    fifo_ready = True
    int_counter += 1

int_pin.irq(trigger=Pin.IRQ_RISING, handler=threshold_interrupt)

## GUN INTERRUPT
@micropython.native
def gun_stuff(_):
    global gun_timestamp
    imu.write_reg(0x4B, 0b00000100) #SIGNAL PATH RESET, STROBE FOR TIMESTAMP BIT
    imu.set_reg_bank(0x01)
    ts_lo, ts_mid, ts_hi = imu.read_reg(0x62, 3)
    imu.set_reg_bank(0x00)
    gun_timestamp = (int(ts_hi) << 16) | (int(ts_mid) << 8) | int(ts_lo)
    gun_fired(1)



    
def set_sensor_type(sensor):
    global gun_int_pin
    if sensor == 'NC':
        gun_int_pin = Pin(GUN_PIN, Pin.IN, Pin.PULL_UP)
        gun_int_pin.irq(trigger=Pin.IRQ_RISING, handler=gun_signal)
    elif sensor == 'NO': # WORKS, NO TOUCH IDIOT
        gun_int_pin = Pin(GUN_PIN, Pin.IN, Pin.PULL_DOWN)
        gun_int_pin.irq(trigger=Pin.IRQ_FALLING, handler=gun_signal)

@micropython.native
def gun_signal(pin):
    global gun_triggered
    if gun_triggered:
        return    
    gun_triggered = True
    if gun_int_pin:
        gun_int_pin.irq(handler=None)
    micropython.schedule(gun_stuff, None)

def create_buffer():
    global buf, wp
    if buf:
        buf = None
    gc.collect()
    free = gc.mem_free()
    assert free > TOTAL_BYTES + 4096,"not enough mem! free = {}, needed = {}".format(free, TOTAL_BYTES)
    buf = bytearray(TOTAL_BYTES)
    wp = 0


@micropython.native
def check_false_start(impulse, timestamp):
    global rising_count, prev_impulse, reaction_time_timestamp, in_window, runner_started, runner_started_ts
   
    if not in_window and accel_threshold and prev_impulse < accel_threshold <= impulse:
       in_window = True
       reaction_time_timestamp = timestamp
       rising_count = 0
    
    if in_window and not runner_started:
        if impulse > prev_impulse + HYST:
            rising_count += 1
        else:
            if accel_threshold and impulse < (accel_threshold - HYST):
                in_window = False
                reaction_time_timestamp = None
                rising_count = 0

        if rising_count >= RISE_STREAK_N:
            runner_started = True
            runner_started_ts = time.ticks_ms()
            rising_count = 0
            if runner_started and not gun_triggered: #rt is pre-gun
                fs_alert.value(1)
            elif runner_started and gun_triggered:  #rt is post-gun
                if (reaction_time_timestamp - gun_timestamp) * 0.000030517578125 < 0.1: # type: ignore #pre calculated 1/32768 for rtc timestamps
                    fs_alert.value(1) 
    
    prev_impulse = impulse
       


@micropython.native
def read_fifo_dump():
    global wp, buf, lost_pkt_total, ts_last, ts_rollovers
    if buf:
        buf_mv = memoryview(buf)

    count = get_count()
    read_len = (count // BYTES_PER_SAMP) * BYTES_PER_SAMP 
    
    imu.read_reg(0x30, read_len, into=buf_mv[wp : wp + read_len]) # retrieve the dump and read directly into memoryview

    for offset in range(wp, wp + read_len, BYTES_PER_SAMP): #inspect our dump
        ### HANDLE TIMESTAMP ROLLOVER ###
        hi = buf_mv[offset + 14] #timestamp hi byte
        lo = buf_mv[offset + 15] #timestamp lo byte
        ts_raw = (hi << 8) | lo
        if ts_last is not None and ts_raw - ts_last < -5000:
            ts_rollovers += 1
        ts_last = ts_raw
        ts_nibble = ts_rollovers & 0x0F
        buf_mv[offset + 13] = ts_nibble #overwrite the temp bit with the num of rollovers to create the top nibble of the full 20-bit timestamp
        ts_full = (ts_nibble << 16) | (hi << 8) | lo
    

        ### CONSTRUCT X-AXIS VALUE ###
        x_raw = (buf_mv[offset + 1] << 8) | buf_mv[offset + 2]
        x_axis_scaled = (x_raw - 0x10000 if x_raw & 0x8000 else x_raw) * 0.00048828125  # pre-calculated 1/2048

        check_false_start(x_axis_scaled, ts_full)
        
    wp += read_len
    
    _ = imu.read_reg(0x2D)[0] # read the INT_STATUS register. Using a _ means it doesn't write to the repl. This bit clears after a read.
    
### HOW MANY BYTES DOES THE FIFO HAVE ###
@micropython.native
def get_count():
    count_hi, count_lo = imu.read_reg(0x2E, 2)
    return (count_hi << 8) | count_lo

def setup(sensor_type, gender):
    global gun_timestamp, runner_started_ts, accel_threshold, int_counter, ts_rollovers
    machine.freq(200000000)
    set_sensor_type(sensor_type)
    if gender == 'M':
        accel_threshold = ACCEL_THRESHOLD_MEN
    elif gender == 'W':
        accel_threshold = ACCEL_THRESHOLD_WOMEN
    gun_fired(0)
    gun_timestamp = None
    runner_started_ts = None
    fs_alert.value(0)
    int_counter = 0
    ts_rollovers = 0
    create_buffer()
    imu.reset()
    imu.write_reg(0x4F, 0b00000101) # GYRO_CONFIG0: FS_SEL=000, ODR=0101 
    imu.write_reg(0x50, 0b000000101) # ACCEL_CONFIG0: FS_SEL=000, ODR=0101 (2kHz) , ODR=0110 (1kHz)
    imu.write_reg(0x53, 0b00000101) # ACCEL_CONFIG1: ACCEL_UI_FILT_ORD=00 (1st order) --bit 0 is reserved and must be set to 1
    imu.write_reg(0x52, 0b00010000) # GYRO_ACCEL_CONFIG0: ACCEL_UI_FILT_BW= 6, GYRO_UI_FILT_BW= 0
    imu.write_reg(0x16, 0b01000000) # FIFO_CONFIG: FIFO_MODE=01 
    imu.write_reg(0x5F, 0b00001111) # FIFO_CONFIG1
    imu.write_reg(0x60, 0x00) # FIFO_CONFIG2: lower byte of threshold 
    imu.write_reg(0x61, 0x02) # FIFO_CONFIG3: upper byte of threshold 
    imu.write_reg(0x54, 0b00111001) # TMST_CONFIG
    imu.write_reg(0x4C, 0b00110011) # INTF_CONFIG0
    imu.write_reg(0x63, 0b00000100) # INT_CONFIG0

    imu.write_reg(0x14, 0b00000111) # INT_CONFIG: INT1_POLARITY = 1; INT1_DRIVE_CIRCUIT = 1, INT1_MODE = 1
    imu.write_reg(0x64, 0b00000000) # INT_CONFIG1: INT_ASYNC_RESET = 0
    imu.write_reg(0x65, 0b00000100) # INT_SOURCE0: Bit 2 = FIFO_THS_INT1_EN
    
    imu.set_reg_bank(0x02)
    imu.write_reg(0x03, 0b00110001) # ACCEL_CONFIG_STATIC2: ACCEL_AAF_DIS=1
    imu.set_reg_bank(0x01)
    imu.write_reg(0x7B,0b00000100) # PIN9_FUNCTION = 10 (CLKIN)
    imu.set_reg_bank(0x00)
    
    imu.write_reg(0x4D, 0b10010100) #INTF_CONFIG1
    imu.write_reg(0x4B, 0b00000010) # SIGNAL PATH RESET
    imu.read_reg(0x30, 2064) #DRAIN THE FIFO
    
    
    
@micropython.native
def make_timestamp_packet(type, ts):
    pkt = bytearray(16)
    if type == 'gun':
        pkt[0] = 0x07
    elif type == 'rt':
        pkt[0] = 0x21
    pkt[13] = (ts >> 16) & 0x0F
    pkt[14] = (ts >> 8) & 0xFF
    pkt[15] = ts & 0xFF
    return pkt


def start_loop():
    print('starting loop')
    global wp, fifo_ready, gun_triggered, gun_timestamp, int_counter, ts_rollovers, runner_started_ts
    imu.write_reg(0x4E, 0b00000011)
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < (DURATION_S * 1000):
        if fifo_ready:
            fifo_ready = False
            read_fifo_dump()        
        time.sleep_us(400)
        if runner_started_ts:
            if time.ticks_diff(time.ticks_ms(), runner_started_ts) > 1000:
                break
        if run_aborted.value() == 1:
            break
    imu.write_reg(0x4E, 0x00)   
    read_fifo_dump()

    imu.write_reg(0x4E, 0x00)
    with open('overall_buffer.bin','wb') as f:
        if buf:
            f.write(memoryview(buf)[:wp])
        if gun_timestamp:
            f.write(make_timestamp_packet('gun', gun_timestamp))
        if reaction_time_timestamp:
            f.write(make_timestamp_packet('rt', reaction_time_timestamp))
    print('Wrote ', wp//16, ' packets')
    print("int called", int_counter, "times")
    if gun_timestamp:
        print('gun at', gun_timestamp)
    else:
        print('no gun detected')
    if reaction_time_timestamp:
        print('reaction time', reaction_time_timestamp)
    else: print('no rt')
    print("Final FIFO count:", get_count())
    print("Final INT_STATUS0:", imu.read_reg(0x2D)[0])
    lost0 = imu.read_reg(0x6C)[0]
    lost1 = imu.read_reg(0x6D)[0]
    lost_pkt_total = (lost1 << 8) | lost0
    print("lost packet total", lost_pkt_total)
    print(ts_rollovers, 'rollovers')  
    

    gc.collect()

    if gun_timestamp or reaction_time_timestamp:
        return gun_timestamp, reaction_time_timestamp

    


def man_start():
    imu.write_reg(0x4E, 0b00001111)
