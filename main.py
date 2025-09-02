import serial
import time
from typing import Literal
from gpiozero import OutputDevice
from fastapi import FastAPI

app = FastAPI()

# --- COMMAND CODES ---
STX = 0xAA
CMD_PING = 0x01
CMD_ARM = 0x02
CMD_SET = 0x03
CMD_DUMP = 0x04
CMD_SET_SENSOR = 0x05
CMD_SET_GENDER = 0x06
CMD_SEND_RT_REPORT = 0x07

BROADCAST_ID = 0x99

abort_pin = OutputDevice(17)
abort_pin.off()

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


def build_arm_packet() -> bytes:
    header = bytes([STX, BROADCAST_ID, CMD_ARM, 0])
    return header + bytes([calc_checksum(header)])


def build_set_packet() -> bytes:
    header = bytes([STX, BROADCAST_ID, CMD_SET, 0])
    return header + bytes([calc_checksum(header)])


def build_sensor_type_packet(block_id: int, sensor_type: Literal['NC', 'NO']) -> bytes:
    payload = sensor_type.encode('utf-8')
    header = bytes([STX, block_id, CMD_SET_SENSOR, 2])
    packet = header + payload
    return packet + bytes([calc_checksum(packet)])


def build_send_report_packet(block_id: int) -> bytes:
    header = bytes([STX, block_id, CMD_SEND_RT_REPORT, 0])
    return header + bytes([calc_checksum(header)])


def build_dump_packet(block_id: int) -> bytes:
    header = bytes([STX, block_id, CMD_DUMP, 0])
    return header + bytes([calc_checksum(header)])


def read_response(ser: serial.Serial, expected_block_id: int, return_cmd) -> bytes:
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
                if block_id == expected_block_id and cmd == return_cmd:
                    return full + checksum
    return b''


def read_dump_chunks(ser: serial.Serial, expected_block_id: int, timeout_seconds: float = 5.0) -> bytes:
    """Read all chunks from a block's dump response and return the complete binary data."""
    file_data = b''
    start_time = time.time()
    
    while time.time() - start_time < timeout_seconds:
        if ser.read(1) == bytes([STX]):
            header = ser.read(3)
            if not header or len(header) < 3:
                continue
            
            block_id, cmd, length = header
            
            # Check if this is the expected block and command
            if block_id != expected_block_id or cmd != CMD_DUMP:
                continue
                
            # If length is 0, this is the ACK packet (end of transmission)
            if length == 0:
                # Read and verify checksum for the ACK packet
                checksum = ser.read(1)
                full = bytes([STX]) + header
                if checksum and checksum[0] == calc_checksum(full):
                    # ACK received, transmission complete
                    break
                continue
            
            # Read the chunk payload
            payload = ser.read(length)
            if not payload or len(payload) != length:
                continue
                
            # Read and verify checksum
            checksum = ser.read(1)
            full = bytes([STX]) + header + payload
            if checksum and checksum[0] == calc_checksum(full):
                file_data += payload
                # Reset timeout for next chunk
                start_time = time.time()
            
    return file_data


def dump_all_blocks():
    """Send dump command to all blocks and save received files."""
    abort_pin.off()
    results = []
    
    with serial.Serial(SERIAL_PORT, BAUD, timeout=0) as ser:
        for block_id in BLOCK_IDS:
            print(f"Requesting dump from block {block_id}...")
            
            # Send dump command
            pkt = build_dump_packet(block_id)
            ser.write(pkt)
            
            # Start reading file chunks immediately (no ACK wait needed)
            # The block sends data first, then ACK
            file_data = read_dump_chunks(ser, block_id)
            
            if file_data:
                # Save to file
                filename = f"block_{block_id}_dump.bin"
                try:
                    with open(filename, "wb") as f:
                        f.write(file_data)
                    
                    results.append({
                        "block_id": block_id,
                        "status": "success",
                        "filename": filename,
                        "bytes_received": len(file_data)
                    })
                    print(f"Saved {len(file_data)} bytes to {filename}")
                    
                except IOError as e:
                    results.append({
                        "block_id": block_id,
                        "status": f"file_write_error: {str(e)}",
                        "filename": filename,
                        "bytes_received": len(file_data)
                    })
            else:
                results.append({
                    "block_id": block_id,
                    "status": "no_data_received",
                    "filename": None,
                    "bytes_received": 0
                })
            
            time.sleep(0.1)  # Small delay between blocks
    
    return results


@app.get('/ping')
def ping_all_blocks():
    abort_pin.off()
    results = []

    with serial.Serial(SERIAL_PORT, BAUD, timeout=0) as ser:
        for block_id in BLOCK_IDS:
            pkt = build_ping_packet(block_id)
            ser.write(pkt)

            response = read_response(ser, block_id, CMD_PING)
            if response:
                results.append({
                    "block_id": block_id,
                    "status": "ok",
                })
            else:
                results.append({
                    "block_id": block_id,
                    "status": "no_response"
                })

            time.sleep(0.05)  # small gap between pings

    return {"results": results}


@app.get('/rt_report')
def get_reports():
    results = []

    with serial.Serial(SERIAL_PORT, BAUD, timeout=0) as ser:
        for block_id in BLOCK_IDS:
            pkt = build_send_report_packet(block_id)
            ser.write(pkt)

            response = read_response(ser, block_id, CMD_SEND_RT_REPORT)
            # response format [STX, BLOCK_ID, CMD_SEND_RT_REPORT, 0x43, 0x41, len(payload)]) + payload
            # in this case, payload is calculated_reaction.to_bytes(3, 'big') **in microseconds**
            # status codes: CA (calculated), NG (no gun), NR (no reaction), ND (no data)
            if response:
                status_code = response[3:5].decode()
                payload_len = response[5]
                reaction = int.from_bytes(
                    response[6:6+payload_len], byteorder='big', signed=True) / 1_000_000
                results.append({
                    "block_id": block_id,
                    "status": status_code,
                    "reaction": reaction
                })
            else:
                results.append({
                    "block_id": block_id,
                    "status": "no_response_from_block"
                })
            time.sleep(0.05)
    return {"results": results}


@app.get('/arm')
def arm():
    abort_pin.off()
    with serial.Serial(SERIAL_PORT, BAUD, timeout=0) as ser:
        pkt = build_arm_packet()
        ser.write(pkt)


@app.get('/set')
def set():
    with serial.Serial(SERIAL_PORT, BAUD, timeout=0) as ser:
        pkt = build_set_packet()
        ser.write(pkt)

@app.get('/dump')
def dump_blocks():
    """Dump binary files from all blocks and save to host machine."""
    results = dump_all_blocks()
    
    # Calculate summary statistics
    successful_dumps = [r for r in results if r["status"] == "success"]
    total_bytes = sum(r["bytes_received"] for r in results)
    
    return {
        "results": results,
        "summary": {
            "total_blocks": len(BLOCK_IDS),
            "successful_dumps": len(successful_dumps),
            "failed_dumps": len(results) - len(successful_dumps),
            "total_bytes_received": total_bytes
        }
    }


@app.get('/abort')
def abort_run():
    abort_pin.on()
