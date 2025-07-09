import struct
import matplotlib.pyplot as plt

# --- CONFIG ---
BIN_FILE = "block/accel_x_log.bin"   # path to your file
SAMPLE_RATE_HZ = 1344          # LIS3DH data rate
SCALE = 9.806 / 16380          # ±2g in high-res mode (12-bit)

# --- LOAD FILE ---
with open(BIN_FILE, "rb") as f:
    data = f.read()

# --- DECODE BINARY ---
samples = [x[0] for x in struct.iter_unpack('<h', data)]  # little-endian int16
accel_mps2 = [s * SCALE for s in samples]

# --- TIME AXIS ---
dt = 1 / SAMPLE_RATE_HZ
timestamps = [i * dt for i in range(len(accel_mps2))]

# --- PLOT ---
plt.figure(figsize=(12, 4))
plt.plot(timestamps, accel_mps2)
plt.title("LIS3DH X-Axis Acceleration")
plt.xlabel("Time (seconds)")
plt.ylabel("Acceleration (m/s²)")
plt.grid(True)
plt.tight_layout()
plt.show()
