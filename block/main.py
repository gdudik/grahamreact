from machine import Pin, UART
import time
import fifo_comms

# --- CONFIGURATION ---

DIP_PIN_ONES = 6
DIP_PIN_TWOS = 8
DIP_PIN_FOURS = 7
DIP_PIN_EIGHTS = 9

block_id_ones = Pin(DIP_PIN_ONES, Pin.IN, Pin.PULL_DOWN)
block_id_twos = Pin(DIP_PIN_TWOS, Pin.IN, Pin.PULL_DOWN)
block_id_fours = Pin(DIP_PIN_FOURS, Pin.IN, Pin.PULL_DOWN)
block_id_eights = Pin(DIP_PIN_EIGHTS, Pin.IN, Pin.PULL_DOWN)

BLOCK_ID = block_id_ones.value() << 0 | block_id_twos.value(
) << 1 | block_id_fours.value() << 2 | block_id_eights.value() << 3

if BLOCK_ID == 0:
    BLOCK_ID = 10

if BLOCK_ID > 10:
    raise ValueError(f'Illegal Block ID: {BLOCK_ID}')

BROADCAST_ID = 0x99
BAUD = 1000000

TX_PIN = 0
RX_PIN = 1
DIR_PIN = 2

BOOT_LIGHT = Pin(25, Pin.OUT)
BOOT_LIGHT.value(1)

# --- COMMAND CODES ---
STX = 0xAA
CMD_PING = 0x01
CMD_ARM = 0x02
CMD_SET = 0x03
CMD_DUMP = 0x04
CMD_SET_SENSOR = 0x05
CMD_SET_GENDER = 0x06
CMD_SEND_RT_REPORT = 0x07

# --- UART and GPIO Setup ---
uart = UART(0, baudrate=BAUD, tx=Pin(TX_PIN), rx=Pin(RX_PIN))
rts = Pin(DIR_PIN, Pin.OUT)
rts.value(0)


gun_sensor_type = 'NC'  # Can be 'NC' or 'NO'
current_gender = None

gun_timestamp = None
rt_timestamp = None

# --- Low-Level Functions ---


def calc_checksum(data: bytes) -> int:
    return sum(data) % 256


def send(data: bytes):
    rts.value(1)
    uart.write(data)
    while not uart.txdone():
        pass
    rts.value(0)


def send_ack(cmd_code: int):
    packet = bytes([STX, BLOCK_ID, cmd_code, 0])  # No payload
    packet += bytes([calc_checksum(packet)])
    send(packet)


def read_packet(timeout_ms=100):
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
        if uart.any():
            if uart.read(1) == b'\xAA':  # Start-of-frame
                header = uart.read(3)  # block_id, cmd, length
                if not header or len(header) < 3:
                    continue
                block_id, cmd, length = header
                payload = uart.read(length) or b''
                checksum = uart.read(1)
                full_packet = bytes([STX]) + header + (payload or b'')
                if checksum and checksum[0] == calc_checksum(full_packet):
                    return block_id, cmd, payload
    return None

# --- Command Handlers ---


def handle_ping(is_broadcast: bool):
    print("PING received")
    if not is_broadcast:
        send_ack(CMD_PING)


def handle_arm(is_broadcast: bool):
    global gun_sensor_type, current_gender
    fifo_comms.setup(gun_sensor_type, current_gender)
    send_ack(CMD_ARM)


def handle_set(is_broadcast: bool):
    return fifo_comms.start_loop()


def handle_dump(is_broadcast: bool):
    print("DUMP request received")
    dump("overall_buffer.bin")
    send_ack(CMD_DUMP)


def handle_set_sensor(is_broadcast: bool, payload: bytes):
    global gun_sensor_type
    try:
        s = payload.decode().strip()
        if s in ('NC', 'NO'):
            gun_sensor_type = s
            print(f"Sensor type set to: {s}")
            if not is_broadcast:
                send_ack(CMD_SET_SENSOR)
        else:
            print("Invalid sensor type payload")
    except Exception as e:
        print("Decode error:", e)


def handle_set_gender(is_broadcast: bool, payload: bytes):
    global current_gender
    try:
        s = payload.decode().strip()
        if s in ('M', 'F'):
            current_gender = s
            print(f"Gender set to: {s}")
            if not is_broadcast:
                send_ack(CMD_SET_GENDER)
        else:
            print("Invalid gender payload")
    except Exception as e:
        print("Decode error:", e)


def handle_send_rt_report():
    global rt_timestamp, gun_timestamp
    if rt_timestamp is not None and gun_timestamp is not None:
        # pre calculated 1/32768 for rtc timestamps in microseconds
        calculated_reaction = int(
            ((rt_timestamp - gun_timestamp) * 0.000030517578125 * 1_000_000))
        b = calculated_reaction.to_bytes(3, 'big')
        # CA (calc) + calculated reaction time
        packet = bytes(
            [STX, BLOCK_ID, CMD_SEND_RT_REPORT, 0x43, 0x41, len(b)]) + b
        packet += bytes([calc_checksum(packet)])
    elif rt_timestamp is not None and gun_timestamp is None:
        packet = bytes([STX, BLOCK_ID, CMD_SEND_RT_REPORT,
                       0x4E, 0x47])  # NG--no gun
        packet += bytes([calc_checksum(packet)])
    elif rt_timestamp is None and gun_timestamp is not None:
        packet = bytes([STX, BLOCK_ID, CMD_SEND_RT_REPORT,
                       0x4E, 0x52])  # NR--no reaction
        packet += bytes([calc_checksum(packet)])
    elif rt_timestamp is None and gun_timestamp is None:
        packet = bytes([STX, BLOCK_ID, CMD_SEND_RT_REPORT,
                       0x4E, 0x44])  # ND--no data
        packet += bytes([calc_checksum(packet)])
    send(packet)


def dump(filepath, chunk_size=256):
    try:
        with open(filepath, "rb") as f:
            while True:
                chunk = f.read(chunk_size)
                if not chunk:
                    break
                packet = bytes([STX, BLOCK_ID, CMD_DUMP, len(chunk)]) + chunk
                packet += bytes([calc_checksum(packet)])
                send(packet)
        print("File transmission complete.")
    except OSError as e:
        print("Failed to open file:", e)


# --- Main Loop ---

def listen():
    print("Listening (binary protocol)...")
    while True:
        is_broadcast = False
        result = read_packet()
        if not result:
            continue

        block_id, cmd, payload = result

        if block_id not in (BLOCK_ID, BROADCAST_ID):
            continue  # Not for this node, not broadcast
        if block_id == BROADCAST_ID:
            is_broadcast = True
        if cmd == CMD_PING:
            handle_ping(is_broadcast)
        elif cmd == CMD_ARM:
            handle_arm(is_broadcast)
        elif cmd == CMD_SET:
            handle_set(is_broadcast)
        elif cmd == CMD_DUMP:
            handle_dump(is_broadcast)
        elif cmd == CMD_SET_SENSOR:
            handle_set_sensor(is_broadcast, payload)
        elif cmd == CMD_SET_GENDER:
            handle_set_gender(is_broadcast, payload)
        elif cmd == CMD_SEND_RT_REPORT:
            handle_send_rt_report()
        else:
            print(f"Unknown command: {cmd}")


# --- Start ---
listen()
