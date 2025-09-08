import serial
import time
from typing import Literal
from gpiozero import OutputDevice
from gpiozero import Device
from gpiozero.pins.pigpio import PiGPIOFactory
Device.pin_factory = PiGPIOFactory()
from fastapi import FastAPI

app = FastAPI()

def debug_packet(packet: bytes, label: str):
    """Debug helper to print packet contents in hex format."""
    hex_str = ' '.join(f'{b:02X}' for b in packet)
    print(f"[DEBUG] {label}: {hex_str} (len={len(packet)})")

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
REPLY_FLAG = 0x40

abort_pin = OutputDevice(17)
abort_pin.off()

rts_pin = OutputDevice(18)
rts_pin.off()

BLOCK_IDS = range(1, 11)  # block IDs 1 through 10
SERIAL_PORT = '/dev/ttyAMA0'  
BAUD = 1500000
TIMEOUT = 0.2  # seconds

def ser_write(ser: serial.Serial, packet: bytes):
    rts_pin.on()
    time.sleep(0.001)
    ser.write(packet)
    ser.flush()
    time.sleep(0.001)
    rts_pin.off()

def calc_checksum(data: bytes) -> int:
    return sum(data) % 256

def reply_cmd(cmd):
    return (cmd | REPLY_FLAG)

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
    expected_reply_cmd = reply_cmd(return_cmd)
    print(f"[DEBUG] Waiting for response: block_id={expected_block_id}, expected_cmd=0x{expected_reply_cmd:02X}")
    
    # Collect all bytes during the timeout period to handle timing variations
    raw_bytes_received = []
    start_time = time.time()
    while time.time() - start_time < TIMEOUT:
        byte_data = ser.read(1)
        if byte_data:
            raw_bytes_received.append(byte_data[0])
    
    if not raw_bytes_received:
        print(f"[DEBUG] No response received")
        return b''
    
    # Process the collected bytes for packet parsing
    all_raw_data = bytes(raw_bytes_received)
    
    # Look for STX and parse packets from the collected data
    i = 0
    while i < len(all_raw_data):
        if all_raw_data[i] == STX:
            # Check if we have enough bytes for header
            if i + 4 > len(all_raw_data):
                break
                
            header = all_raw_data[i+1:i+4]
            block_id, cmd, length = header
            
            # Check if we have enough bytes for payload and checksum
            packet_end = i + 4 + length + 1
            if packet_end > len(all_raw_data):
                break
                
            payload = all_raw_data[i+4:i+4+length] if length > 0 else b''
            checksum_byte = all_raw_data[i+4+length]
            
            full = all_raw_data[i:i+4+length]
            expected_checksum = calc_checksum(full)
            actual_checksum = checksum_byte
            
            if actual_checksum != expected_checksum:
                print(f"[DEBUG] Checksum mismatch for block {block_id}")
                i += 1
                continue
                
            if block_id != expected_block_id:
                i += 1
                continue
                
            if cmd != expected_reply_cmd:
                i += 1
                continue
                
            print(f"[DEBUG] Valid response received from block {block_id}")
            response_packet = full + bytes([checksum_byte])
            return response_packet
        
        i += 1
            
    print(f"[DEBUG] No valid response found in {len(all_raw_data)} bytes received")
    return b''


def read_exact_bytes(ser: serial.Serial, num_bytes: int, timeout_seconds: float = 1.0) -> bytes:
    """Read exactly num_bytes from serial port, blocking until complete or timeout."""
    data = b''
    start_time = time.time()
    
    while len(data) < num_bytes and (time.time() - start_time) < timeout_seconds:
        remaining = num_bytes - len(data)
        chunk = ser.read(remaining)
        if chunk:
            data += chunk
        else:
            # Small delay to prevent busy waiting
            time.sleep(0.001)
    
    return data


def read_dump_chunks(ser: serial.Serial, expected_block_id: int, timeout_seconds: float = 3.0) -> bytes:
    """Read all chunks from a block's dump response and return the complete binary data."""
    file_data = b''
    start_time = time.time()
    chunk_count = 0
    checksum_failures = 0
    incomplete_reads = 0
    wrong_packets = 0
    
    while time.time() - start_time < timeout_seconds:
        if ser.read(1) == bytes([STX]):
            # Read header with blocking read
            header = read_exact_bytes(ser, 3, 1.0)
            if not header or len(header) < 3:
                incomplete_reads += 1
                print(f"Incomplete header read (got {len(header)} bytes)")
                continue
            
            block_id, cmd, length = header
            
            # Check if this is the expected block and command
            if block_id != expected_block_id or cmd != reply_cmd(CMD_DUMP):
                wrong_packets += 1
                print(f"Wrong packet: block_id={block_id} (expected {expected_block_id}), cmd={cmd} (expected {CMD_DUMP})")
                continue
                
            # If length is 0, this is the ACK packet (end of transmission)
            if length == 0:
                print(f"Received ACK packet from block {expected_block_id}")
                # Read and verify checksum for the ACK packet
                checksum = read_exact_bytes(ser, 1, 1.0)
                full = bytes([STX]) + header
                if checksum and len(checksum) == 1 and checksum[0] == calc_checksum(full):
                    if checksum_failures > 0:
                        print(f"Block {expected_block_id}: {checksum_failures} checksum failures detected")
                    break
                else:
                    print(f"ACK checksum invalid: got {checksum[0] if checksum and len(checksum) > 0 else 'None'}, expected {calc_checksum(full)}")
                continue
            
            # Read the chunk payload with blocking read
            payload = read_exact_bytes(ser, length, 2.0)
            if not payload or len(payload) != length:
                incomplete_reads += 1
                print(f"Incomplete payload read: got {len(payload) if payload else 0} bytes, expected {length}")
                continue
                
            # Read and verify checksum with blocking read
            checksum = read_exact_bytes(ser, 1, 1.0)
            full = bytes([STX]) + header + payload
            expected_checksum = calc_checksum(full)
            
            if checksum and len(checksum) == 1 and checksum[0] == expected_checksum:
                chunk_count += 1
                file_data += payload
                if chunk_count % 50 == 0:  # Log every 50th chunk to reduce spam
                    print(f"Chunk {chunk_count}: {length} bytes received (total: {len(file_data)} bytes)")
                # Reset timeout for next chunk
                start_time = time.time()
            else:
                checksum_failures += 1
                if checksum_failures <= 5:  # Only show first 5 failures to avoid spam
                    actual_checksum = checksum[0] if checksum and len(checksum) > 0 else None
                    print(f"Block {expected_block_id}: Checksum failure #{checksum_failures} - got {actual_checksum}, expected {expected_checksum}")
            
    return file_data


def dump_all_blocks():
    """Send dump command to all blocks and save received files."""
    abort_pin.off()
    results = []
    
    with serial.Serial(SERIAL_PORT, BAUD, timeout=0) as ser:
        for block_id in BLOCK_IDS:
            print(f"Requesting dump from block {block_id}...")
            
            # Start timer
            start_time = time.time()
            
            # Send dump command
            pkt = build_dump_packet(block_id)
            ser_write(ser, pkt)
            
            # Start reading file chunks immediately (no ACK wait needed)
            # The block sends data first, then ACK
            file_data = read_dump_chunks(ser, block_id)
            
            # End timer and calculate duration
            end_time = time.time()
            duration = end_time - start_time
            
            if file_data:
                # Calculate throughput
                throughput = len(file_data) / duration / 1024  # KB/s
                print(f"Block {block_id}: {len(file_data)} bytes in {duration:.2f}s ({throughput:.1f} KB/s)")
                
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
                print(f"Block {block_id}: No data received in {duration:.2f}s")
                results.append({
                    "block_id": block_id,
                    "status": "no_data_received",
                    "filename": None,
                    "bytes_received": 0
                })
            
            time.sleep(0.1)  # Small delay between blocks
    
    return results


@app.post('/ping')
def ping_all_blocks():
    abort_pin.off()
    results = []

    with serial.Serial(SERIAL_PORT, BAUD, timeout=0) as ser:
        for block_id in BLOCK_IDS:
            print(f"\n[DEBUG] === PINGING BLOCK {block_id} ===")
            pkt = build_ping_packet(block_id)
            debug_packet(pkt, f"SENDING to block {block_id}")
            ser_write(ser, pkt)

            response = read_response(ser, block_id, CMD_PING)
            if response:
                debug_packet(response, f"RECEIVED from block {block_id}")
                results.append({
                    "block_id": block_id,
                    "status": "ok",
                })
            else:
                print(f"[DEBUG] No response received from block {block_id}")
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
            ser_write(ser, pkt)

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


@app.post('/arm')
def arm():
    results = []
    abort_pin.off()
    for block_id in BLOCK_IDS:
        with serial.Serial(SERIAL_PORT, BAUD, timeout=0) as ser:
            pkt = build_arm_packet()
            ser_write(ser,pkt)
            response = read_response(ser, block_id, CMD_ARM)
            if response:
                if response:
                    results.append({
                        "block_id": block_id,
                        "status": "armed",
                    })
            else:
                results.append({
                    "block_id": block_id,
                    "status": "no_response"
                })
    return {"results": results}


@app.post('/set')
def set():
    with serial.Serial(SERIAL_PORT, BAUD, timeout=0) as ser:
        pkt = build_set_packet()
        ser_write(ser, pkt)

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


@app.post('/abort')
def abort_run():
    abort_pin.on()
