import serial
import time

CHUNK_SIZE = 256  # data bytes (add 1 for checksum)
BAUD = 1000000
PORT = "/dev/tty.usbserial-BG012FG2"  # change to your actual port
timeout_after_idle = 0.5  # seconds to wait after last byte

def verify_checksum(packet: bytes) -> bool:
    data = packet[:-1]
    checksum = packet[-1]
    return (sum(data) % 256) == checksum

def main():
    with serial.Serial(PORT, BAUD, timeout=0) as ser:
        print(f"Waiting for transmission on {PORT} at {BAUD} baud...")

        total_chunks = 0
        good_chunks = 0
        bad_chunks = 0
        last_data_time = None
        start_time = None
        end_time = None
        buffer = bytearray()

        try:
            while True:
                if ser.in_waiting:
                    byte = ser.read(1)
                    buffer += byte
                    now = time.time()
                    
                    if start_time is None:
                        start_time = now
                        print("⏱️ Transmission started...")

                    last_data_time = now

                    if len(buffer) == CHUNK_SIZE + 1:
                        total_chunks += 1
                        if verify_checksum(buffer):
                            good_chunks += 1
                        else:
                            bad_chunks += 1
                            print(f"❌ Bad chunk #{total_chunks} (checksum mismatch)")

                        buffer.clear()
                else:
                    if last_data_time and (time.time() - last_data_time > timeout_after_idle):
                        end_time = time.time()
                        print("📭 Timed out waiting for new data.")
                        break
                    time.sleep(0.001)

        except KeyboardInterrupt:
            print("\n⏹️ Stopped by user.")

        print(f"\nTotal: {total_chunks}, Good: {good_chunks}, Bad: {bad_chunks}")
        if bad_chunks == 0 and total_chunks > 0:
            print("✅ Transfer verified with no errors.")
        elif total_chunks == 0:
            print("⚠️ No data received.")
        else:
            print("❌ Transfer had errors.")

        if start_time and end_time:
            duration = end_time - start_time
            print(f"⏲️ Transmission duration: {duration:.3f} seconds")

if __name__ == "__main__":
    main()
