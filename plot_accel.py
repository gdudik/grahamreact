#!/usr/bin/env python3
import argparse
import struct
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import csv
from datetime import datetime

PACKET_SIZE = 16
RTC_HZ = 32_768.0
TS_MASK = (1 << 20) - 1
GUN_HEADER = 0x07
REACTION_HEADER = 0x21

def parse_packets(data: bytes):
    if len(data) % PACKET_SIZE != 0:
        data = data[: len(data) // PACKET_SIZE * PACKET_SIZE]
    n = len(data) // PACKET_SIZE
    ticks = np.empty(n, dtype=np.uint32)
    x_raw = np.empty(n, dtype=np.int16)
    mv = memoryview(data)

    for i in range(n):
        pkt = mv[i * PACKET_SIZE : (i + 1) * PACKET_SIZE]
        x_raw[i] = struct.unpack_from(">h", pkt, 1)[0]
        raw24 = (pkt[13] << 16) | (pkt[14] << 8) | pkt[15]
        ticks[i] = raw24 & TS_MASK
    return ticks, x_raw

def scale_to_g(x_raw: np.ndarray, fsr_g: int) -> np.ndarray:
    return x_raw.astype(np.float64) / (32768.0 / fsr_g)

def find_timestamp_ticks(data: bytes, header: int) -> int | None:
    n = len(data) // PACKET_SIZE
    mv = memoryview(data)
    for i in range(n - 1, -1, -1):
        pkt = mv[i * PACKET_SIZE : (i + 1) * PACKET_SIZE]
        if pkt[0] == header:
            raw24 = (pkt[13] << 16) | (pkt[14] << 8) | pkt[15]
            return raw24 & TS_MASK
    return None

def main():
    ap = argparse.ArgumentParser(description="Plot ICM-42688 X-axis accel with gun/reaction markers.")
    ap.add_argument("binfile", type=Path, help="Path to binary log")
    ap.add_argument("--fsr", type=int, default=16, choices=[2,4,8,16], help="Accel full-scale range in g (default: 16)")
    ap.add_argument("--out-png", type=Path, default=None, help="Save plot to PNG")
    ap.add_argument("--out-csv", nargs='?', const=True, default=None, help="Save CSV (time_s, accel_x_g). If no filename provided, uses current date/time")
    ap.add_argument("--out-bin", nargs='?', const=True, default=None, help="Save binary copy. If no filename provided, uses current date/time")
    ap.add_argument("--show", action="store_true", help="Show plot interactively")
    args = ap.parse_args()

    data = args.binfile.read_bytes()
    ticks, x_raw = parse_packets(data)

    gun_tick = find_timestamp_ticks(data, GUN_HEADER)
    t0_tick = gun_tick if gun_tick is not None else ticks[0]
    t_s = (ticks.astype(np.int32) - t0_tick) / RTC_HZ
    x_g = scale_to_g(x_raw, args.fsr)

    order = np.argsort(t_s, kind="mergesort")
    t_s = t_s[order]
    x_g = x_g[order]

    fig, ax = plt.subplots()
    ax.plot(t_s, x_g, linewidth=1.0)
    ax.set_xlabel("Time (s) [relative]")
    ax.set_ylabel(f"Accel X (g)  [±{args.fsr} g]")
    ax.set_title("ICM-42688 X-Axis Accel vs Time")
    ax.grid(True)

    # Gun marker
    if gun_tick is not None:
        ax.axvline(0.0, linestyle=":", color="red", linewidth=1.5)
        ax.text(0.0, ax.get_ylim()[1], "gun", rotation=90, va='top', ha='right', fontsize=8, color="red")
        ax.axvline(0.1, linestyle=":", color="black", linewidth=1.5)

    # Reaction marker
    reaction_tick = find_timestamp_ticks(data, REACTION_HEADER)
    if reaction_tick is not None:
        reaction_time = (reaction_tick - t0_tick) / RTC_HZ
        ax.axvline(reaction_time, linestyle="--", color="blue", linewidth=1.5)
        ax.text(reaction_time, ax.get_ylim()[1], "reaction", rotation=90, va='top', ha='right', fontsize=8, color="blue")

    plt.tight_layout()

    if args.out_png:
        plt.savefig(args.out_png, dpi=150)
    if args.out_csv:
        if args.out_csv is True:  # No filename provided
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_filename = f"{timestamp}.csv"
        else:  # Filename was provided
            csv_filename = args.out_csv
        
        with open(csv_filename, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "accel_x_g"])
            for t, x in zip(t_s, x_g):
                w.writerow([f"{t:.9f}", f"{x:.9f}"])
    if args.out_bin:
        if args.out_bin is True:  # No filename provided
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            bin_filename = f"{timestamp}.bin"
        else:  # Filename was provided
            bin_filename = args.out_bin
        
        with open(bin_filename, "wb") as f:
            f.write(data)
    if args.show or not args.out_png:
        plt.show()

if __name__ == "__main__":
    main()
