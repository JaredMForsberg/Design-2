import serial
import struct

# ===== CONFIG =====
PORT = "COM6"          # Change this to your STM32 COM port
BAUD = 115200
TIMEOUT = 5            # Seconds
OUTPUT_FILE = "capture.jpg"
# ==================

def read_exact(ser, num_bytes):
    """
    Read exactly num_bytes from serial, or raise RuntimeError on timeout.
    """
    data = bytearray()
    while len(data) < num_bytes:
        chunk = ser.read(num_bytes - len(data))
        if not chunk:
            raise RuntimeError(f"Timeout while reading {num_bytes} bytes (got {len(data)})")
        data.extend(chunk)
    return bytes(data)


def main():
    print(f"Opening {PORT} @ {BAUD}...")
    with serial.Serial(PORT, BAUD, timeout=TIMEOUT) as ser:

        # Clear garbage
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Send single-char capture command
        print("Requesting capture...")
        ser.write(b"C")

        # Expect 3-byte magic ("IMG" or "ERR")
        magic = read_exact(ser, 3)

        if magic == b"ERR":
            raise RuntimeError("Device reported error (FIFO bad or timeout).")

        if magic != b"IMG":
            raise RuntimeError(f"Bad magic received: {magic!r}")

        # Read 4-byte little-endian image length
        length_bytes = read_exact(ser, 4)
        (img_len,) = struct.unpack("<I", length_bytes)
        print(f"Image length reported: {img_len} bytes")

        if img_len == 0 or img_len > 2_000_000:
            raise RuntimeError(f"Image length looks wrong: {img_len}")

        # Receive JPEG
        print("Receiving JPEG data...")
        jpeg_data = read_exact(ser, img_len)

        # Save file
        with open(OUTPUT_FILE, "wb") as f:
            f.write(jpeg_data)

        print(f"Saved JPEG to: {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
