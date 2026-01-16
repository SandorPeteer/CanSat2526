#!/usr/bin/env python3
"""
Debug tool: Raw serial port dumper with frame detection
Shows exactly what bytes the Heltec firmware sends
"""

import sys
import serial
import time
from typing import Optional

SYNC_FULL = 0xA5
SYNC_DELTA = 0xA4
META_H0 = 0xD3
META_H1 = 0xD4

def format_hex(data: bytes) -> str:
    """Format bytes as hex with ASCII representation"""
    hex_str = ' '.join(f'{b:02X}' for b in data)
    ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in data)
    return f"{hex_str:48s}  |{ascii_str}|"

def detect_frame_type(data: bytes) -> Optional[str]:
    """Detect what type of frame this might be"""
    if len(data) == 0:
        return None

    b0 = data[0]
    if b0 == SYNC_FULL:
        return "FULL_TELEM (0xA5)"
    elif b0 == SYNC_DELTA:
        return "DELTA_TELEM (0xA4)"
    elif len(data) >= 2 and b0 == META_H0 and data[1] == META_H1:
        return "META_FRAME (0xD3 0xD4)"
    elif 32 <= b0 < 127:
        return "ASCII_TEXT (boot msg?)"
    else:
        return "UNKNOWN"

def main():
    if len(sys.argv) < 2:
        print("Usage: python debug_serial_dump.py <serial_port>")
        print("Example: python debug_serial_dump.py /dev/cu.usbserial-0001")
        sys.exit(1)

    port = sys.argv[1]

    print(f"Opening {port} at 115200 baud...")
    print("Press Ctrl+C to stop\n")
    print("=" * 80)

    try:
        ser = serial.Serial(port, 115200, timeout=1)

        # Reset the ESP32 and capture boot messages
        print("\n*** Waiting for data (press RESET on Heltec now) ***\n")

        byte_count = 0
        line_buffer = bytearray()

        while True:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)

                for byte in data:
                    line_buffer.append(byte)
                    byte_count += 1

                    # Print in 16-byte chunks for readability
                    if len(line_buffer) >= 16:
                        frame_type = detect_frame_type(line_buffer)
                        timestamp = time.strftime("%H:%M:%S")
                        print(f"[{timestamp}] {byte_count-15:06d}-{byte_count:06d}: {format_hex(line_buffer[:16])}  <{frame_type}>")
                        line_buffer = line_buffer[16:]
            else:
                # Flush remaining bytes if any
                if line_buffer:
                    frame_type = detect_frame_type(line_buffer)
                    timestamp = time.strftime("%H:%M:%S")
                    padding = 16 - len(line_buffer)
                    padded = line_buffer + b'\x00' * padding
                    hex_part = format_hex(padded)
                    # Remove the padding from display
                    hex_display = ' '.join(f'{b:02X}' for b in line_buffer)
                    ascii_display = ''.join(chr(b) if 32 <= b < 127 else '.' for b in line_buffer)
                    print(f"[{timestamp}] {byte_count-len(line_buffer)+1:06d}-{byte_count:06d}: {hex_display:48s}  |{ascii_display}|  <{frame_type}>")
                    line_buffer.clear()

                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\nStopped by user")
        print(f"Total bytes captured: {byte_count}")

    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)

    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    main()
