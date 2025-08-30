
import struct
import csv
import sys
from pathlib import Path

PACKET_SIZE = 16
TS_MAX = 0x10000  # 16-bit timestamp rollover point

def to_int16(msb, lsb):
    val = (msb << 8) | lsb
    return val - 0x10000 if val & 0x8000 else val

def parse_fifo(input_file, output_file):
    with open(input_file, "rb") as f:
        data = f.read()

    if len(data) % PACKET_SIZE != 0:
        print(f"Warning: file size {len(data)} is not a multiple of {PACKET_SIZE}")

    packets = [data[i:i + PACKET_SIZE] for i in range(0, len(data), PACKET_SIZE)]
    rows = []

    rollover = 0
    last_ts = None

    for pkt in packets:
        if len(pkt) != PACKET_SIZE:
            continue  # skip incomplete packets

        header = pkt[0]
        ax = to_int16(pkt[1], pkt[2]) / 2048
        ay = to_int16(pkt[3], pkt[4]) / 2048
        az = to_int16(pkt[5], pkt[6]) / 2048
        gx = to_int16(pkt[7], pkt[8]) / 16.4
        gy = to_int16(pkt[9], pkt[10]) / 16.4
        gz = to_int16(pkt[11], pkt[12]) / 16.4
        temp = pkt[13] / 2.07 + 25
        ts16 = (pkt[14] << 8) | pkt[15]

        # Simple rollover logic: always increment if 16-bit ts goes backward
        if last_ts is not None and ts16 < last_ts:
            rollover += 1
        full_ts = (rollover << 16) | ts16
        last_ts = ts16

        rows.append([header, ax, ay, az, gx, gy, gz, temp, full_ts])

    with open(output_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["header", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "temp_C", "timestamp"])
        writer.writerows(rows)

    print(f"Wrote {len(rows)} packets to {output_file}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python convert_fifo_to_csv.py input.bin output.csv")
        sys.exit(1)

    parse_fifo(sys.argv[1], sys.argv[2])
