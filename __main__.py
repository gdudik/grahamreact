import serial
import time
from typing import Literal

# --- COMMAND CODES ---
STX = 0xAA
CMD_PING = 0x01
CMD_ARM = 0x02
CMD_SET = 0x03
CMD_DUMP = 0x04
CMD_SET_SENSOR = 0x05
CMD_SET_GENDER = 0x06
CMD_SEND_RT_REPORT = 0x07


BLOCK_IDS = range(1, 11)  # block IDs 1 through 10
SERIAL_PORT = '/dev/ttyUSB0'  # Adjust if using onboard UART
BAUD = 1000000
TIMEOUT = 0.2  # seconds

def calc_checksum(data: bytes) -> int:
    return sum(data) % 256

def build_ping_packet(block_id: int) -> bytes:
    header = bytes([STX, block_id, CMD_PING, 0])  # 0-length payload
    return header + bytes([calc_checksum(header)])

def build_gender_packet(block_id: int, gender: Literal['M', 'F']) -> bytes:
    payload = gender.encode('utf-8')
    header = bytes([STX, block_id, CMD_SET_GENDER, 1])
    packet = header + payload
    return packet + bytes([calc_checksum(packet)])

def build_sensor_type_packet(block_id: int, sensor_type: Literal['NC', 'NO']) -> bytes:
    payload = sensor_type.encode('utf-8')
    header = bytes([STX, block_id, CMD_SET_SENSOR, 2])
    packet = header + payload
    return packet + bytes([calc_checksum(packet)])

def read_response(ser: serial.Serial, expected_block_id: int) -> bytes:
    start = time.time()
    while time.time() - start < TIMEOUT:
        if ser.read(1) == bytes([STX]):
            header = ser.read(3)
            if not header or len(header) < 3:
                continue
            block_id, cmd, length = header
            payload = ser.read(length) if length > 0 else b''
            checksum = ser.read(1)

            full = bytes([STX]) + header + payload
            if checksum and checksum[0] == calc_checksum(full):
                if block_id == expected_block_id and cmd == CMD_PING:
                    return full + checksum
    return b''

def ping_all_blocks():
    with serial.Serial(SERIAL_PORT, BAUD, timeout=0) as ser:
        for block_id in BLOCK_IDS:
            pkt = build_ping_packet(block_id)
            ser.write(pkt)
            print(f"Sent PING to block {block_id:02d}")

            response = read_response(ser, block_id)
            if response:
                print(f"Received valid response from block {block_id:02d}: {response.hex()}")
            else:
                print(f"No response from block {block_id:02d}")
            time.sleep(0.05)  # optional small gap between pings

# --- Run it ---
if __name__ == '__main__':
    ping_all_blocks()
