#!/usr/bin/env python3
"""
Test script for STM32F769I-Discovery USB CDC serial communication.
Collects logs and tests connectivity to simplify feedback.
"""

import serial
import sys
import time
from datetime import datetime

# Default serial port - adjust for your system
PORT = "/dev/cu.usbserial-1410"
BAUD = 115200
LOG_FILE = "firmware_test.log"

def main():
    print(f"=== STM32F769I-Discovery Serial Test ===")
    print(f"Port: {PORT}")
    print(f"Baud: {BAUD}")
    print(f"Log file: {LOG_FILE}")
    print()

    try:
        # Open serial port
        ser = serial.Serial(PORT, BAUD, timeout=2, write_timeout=2)
        print(f"[{datetime.now().strftime('%H:%M:%S')}] Connected to {PORT}")

        with open(LOG_FILE, 'a') as log:
            log.write(f"\n=== Test started at {datetime.now()} ===\n")
            log.flush()

            # Send test commands and collect responses
            test_commands = [
                ("AT", "OK"),
                ("ATE0", "OK"),
                ("AT+GMR", ""),  # Get version
                ("AT+CIFSR", ""),  # Get connection status
            ]

            for cmd, expected in test_commands:
                print(f"\n--- Sending: {cmd} ---")
                ser.write((cmd + "\r\n").encode())

                # Read response with timeout
                start = time.time()
                response = b""
                while time.time() - start < 5:  # 5 second timeout
                    if ser.in_waiting():
                        chunk = ser.read(ser.in_waiting())
                        response += chunk
                        if b"OK" in response or b"ERROR" in response:
                            break
                    time.sleep(0.01)

                response_str = response.decode(errors='ignore').strip()
                print(f"Response: {response_str}")
                log.write(f"TX: {cmd}\nRX: {response_str}\n")
                log.flush()
                time.sleep(0.5)

        ser.close()
        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] Test complete, log saved to {LOG_FILE}")
        print(f"View log with: tail -20 {LOG_FILE}")

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        sys.exit(0)

if __name__ == "__main__":
    main()
