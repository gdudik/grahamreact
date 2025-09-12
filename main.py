import serial
import serial.rs485
import time
from typing import Literal
import RPi.GPIO as GPIO
from fastapi import FastAPI
import checksum

app = FastAPI()


def debug_packet(packet: bytes, label: str):
    """Debug helper to print packet contents in hex format."""
    hex_str = ' '.join(f'{b:02X}' for b in packet)
    print(f"[DEBUG] {label}: {hex_str} (len={len(packet)})")


def _time_left(deadline: float) -> float:
    t = deadline - time.monotonic()
    return t if t > 0 else 0.0

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

active_blocks = []

# --- PINS ---
GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.OUT)


def abort_pin(state: bool):
    GPIO.output(27, state)


abort_pin(False)


BLOCK_IDS = range(1, 11)  # block IDs 1 through 10
SERIAL_PORT = '/dev/ttyAMA0'
BAUD = 1500000
TIMEOUT = 0.2  # seconds


def open_rs485():
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=TIMEOUT, rtscts=False)
    ser.rs485_mode = serial.rs485.RS485Settings(
        rts_level_for_tx=True,
        rts_level_for_rx=False,
    )
    return ser


def ser_write(ser: serial.Serial, packet: bytes):
    # ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()
    time.sleep(0.001)





def reply_cmd(cmd):
    return (cmd | REPLY_FLAG)




def read_one_packet(ser: serial.Serial, deadline: float):
    """
    Read exactly one framed packet: [STX][block_id][cmd][len][payload...][csum]
    Returns (block_id, cmd, payload) or None if timeout/invalid.
    """
    # 1) Hunt for STX within remaining time
    while _time_left(deadline) > 0:
        b = ser.read(1)
        if not b:
            continue
        if b[0] != STX:
            continue

        # 2) Fixed 3-byte header
        head = ser.read(3)
        if len(head) != 3:
            return None
        block_id, cmd, length = head

        # 3) Payload + checksum
        payload = ser.read(length)
        if len(payload) != length:
            return None
        csum = ser.read(1)
        if len(csum) != 1:
            return None

        full_wo = bytes([STX]) + head + payload
        if calc_checksum(full_wo) != csum[0]:
            continue  # bad frame; keep hunting

        return (block_id, cmd, payload)

    return None

def read_response(ser: serial.Serial, expected_block_id: int, return_cmd) -> bytes:
    """
    Wait for a reply packet from a specific block_id/cmd.
    Always prints whatever raw bytes were captured, even if invalid.
    Returns the full verified frame (with checksum) or b'' if not found.
    """
    expected_cmd = reply_cmd(return_cmd)
    deadline = time.time() + TIMEOUT
    raw = bytearray()

    # Collect whatever arrives until timeout
    while time.time() < deadline:
        b = ser.read(1)
        if b:
            raw.append(b[0])
        else:
            time.sleep(0.001)

    if not raw:
        print("[DEBUG] No bytes received at all")
        return b''

    # Show raw buffer in hex, even if incomplete
    hex_str = ' '.join(f'{x:02X}' for x in raw)
    print(f"[DEBUG] Raw buffer ({len(raw)} bytes): {hex_str}")

    # Try to parse frames from raw
    i = 0
    while i < len(raw):
        if raw[i] != STX:
            i += 1
            continue

        # header?
        if i + 4 > len(raw):
            print(f"[DEBUG] Incomplete header at pos {i}")
            break

        block_id, cmd, length = raw[i+1:i+4]
        packet_end = i + 4 + length + 1
        if packet_end > len(raw):
            print(f"[DEBUG] Incomplete packet at pos {i}, need {packet_end - len(raw)} more bytes")
            break

        payload = raw[i+4:i+4+length]
        checksum = raw[i+4+length]
        full_wo = raw[i:i+4+length]
        if calc_checksum(full_wo) != checksum:
            print(f"[DEBUG] Bad checksum at pos {i} (got {checksum:02X}, expected {calc_checksum(full_wo):02X})")
            i += 1
            continue

        if block_id != expected_block_id or cmd != expected_cmd:
            print(f"[DEBUG] Packet at pos {i} not for us (block={block_id}, cmd=0x{cmd:02X})")
            i += 1
            continue

        print(f"[DEBUG] Valid response from block {block_id}")
        return bytes(full_wo) + bytes([checksum])

    print("[DEBUG] No valid response frame parsed")
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
                print(
                    f"Wrong packet: block_id={block_id} (expected {expected_block_id}), cmd={cmd} (expected {CMD_DUMP})")
                continue

            # If length is 0, this is the ACK packet (end of transmission)
            if length == 0:
                print(f"Received ACK packet from block {expected_block_id}")
                # Read and verify checksum for the ACK packet
                checksum = read_exact_bytes(ser, 1, 1.0)
                full = bytes([STX]) + header
                if checksum and len(checksum) == 1 and checksum[0] == checksum.calc_checksum(full):
                    if checksum_failures > 0:
                        print(
                            f"Block {expected_block_id}: {checksum_failures} checksum failures detected")
                    break
                else:
                    print(
                        f"ACK checksum invalid: got {checksum[0] if checksum and len(checksum) > 0 else 'None'}, expected {checksum.calc_checksum(full)}")
                continue

            # Read the chunk payload with blocking read
            payload = read_exact_bytes(ser, length, 2.0)
            if not payload or len(payload) != length:
                incomplete_reads += 1
                print(
                    f"Incomplete payload read: got {len(payload) if payload else 0} bytes, expected {length}")
                continue

            # Read and verify checksum with blocking read
            checksum = read_exact_bytes(ser, 1, 1.0)
            full = bytes([STX]) + header + payload
            expected_checksum = checksum.calc_checksum(full)

            if checksum and len(checksum) == 1 and checksum[0] == expected_checksum:
                chunk_count += 1
                file_data += payload
                if chunk_count % 50 == 0:  # Log every 50th chunk to reduce spam
                    print(
                        f"Chunk {chunk_count}: {length} bytes received (total: {len(file_data)} bytes)")
                # Reset timeout for next chunk
                start_time = time.time()
            else:
                checksum_failures += 1
                if checksum_failures <= 5:  # Only show first 5 failures to avoid spam
                    actual_checksum = checksum[0] if checksum and len(
                        checksum) > 0 else None
                    print(
                        f"Block {expected_block_id}: Checksum failure #{checksum_failures} - got {actual_checksum}, expected {expected_checksum}")

    return file_data


def dump_all_blocks():
    """Send dump command to all blocks and save received files."""
    abort_pin(False)
    results = []

    with open_rs485() as ser:
        if not active_blocks:
            return 'No Active Blocks'
        for block_id in active_blocks:
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
                print(
                    f"Block {block_id}: {len(file_data)} bytes in {duration:.2f}s ({throughput:.1f} KB/s)")

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
    global active_blocks
    abort_pin(False)
    active_blocks = []
    results = []

    with open_rs485() as ser:
        for block_id in BLOCK_IDS:
            print(f"\n[DEBUG] === PINGING BLOCK {block_id} ===")
            pkt = build_ping_packet(block_id)
            debug_packet(pkt, f"SENDING to block {block_id}")
            ser_write(ser, pkt)

            response = read_response(ser, block_id, CMD_PING)
            if response:
                debug_packet(response, f"RECEIVED from block {block_id}")
                active_blocks.append(block_id)
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

    print("active", active_blocks)
    return {"results": results}


@app.get('/rt_report')
def get_reports():
    global active_blocks
    results = []
    with open_rs485() as ser:
        if not active_blocks:
            return 'No Active Blocks'
        for block_id in active_blocks:
            pkt = build_send_report_packet(block_id)
            ser_write(ser, pkt)
            response = read_response(ser, block_id, CMD_SEND_RT_REPORT)
            # response format [STX, BLOCK_ID, CMD_SEND_RT_REPORT, len(payload)], <<2 byte status code as part of payload>>) + payload
            # in this case, payload is calculated_reaction.to_bytes(3, 'big') **in microseconds**
            # status codes: CA (calculated), NG (no gun), NR (no reaction), ND (no data)
            if response:
                print(response)
                payload_len = response[3]
                status_code = response[4:6].decode()
                reaction = int.from_bytes(
                    response[6:6+payload_len], byteorder='big', signed=True) / 1_000_000
                if status_code and status_code == 'CA':
                    results.append({
                        "block_id": block_id,
                        "status": status_code,
                        "reaction": reaction
                    })
                elif status_code and status_code != 'CA':
                    results.append({
                        "block_id": block_id,
                        "status": status_code
                    })
                else:
                    results.append({
                        "block_id": block_id,
                        "status": "invalid_response"
                    })
            else:
                results.append({
                    "block_id": block_id,
                    "status": "no_response"
                })
        return {"results": results}



@app.post('/arm')
def arm():
    results = []
    abort_pin(False)
    with open_rs485() as ser:
        if not active_blocks:
            return 'No Active Blocks'
        for block_id in active_blocks:
            pkt = build_arm_packet()
            ser_write(ser, pkt)
            response = read_response(ser, block_id, CMD_ARM)
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
    with open_rs485() as ser:
        pkt = build_set_packet()
        ser_write(ser, pkt)


@app.get('/dump')
def dump_blocks():
    """Dump binary files from all blocks and save to host machine."""

    results = dump_all_blocks()
    if not (isinstance(results, list) and all(isinstance(r, dict) for r in results)):
            return {"results": [], "summary": {"note": "Invalid results"}}

    # Calculate summary statistics
    successful_dumps = [r for r in results if r["status"] == "success"]
    total_bytes = sum(r["bytes_received"] for r in results)

    return {
        "results": results,
        "summary": {
            "total_blocks": len(active_blocks),
            "successful_dumps": len(successful_dumps),
            "failed_dumps": len(results) - len(successful_dumps),
            "total_bytes_received": total_bytes
        }
    }


@app.post('/abort')
def abort_run():
    abort_pin(True)

@app.post('/set_gender/{gender}')
def set_gender(gender: str):
