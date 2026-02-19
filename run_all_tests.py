#!/usr/bin/env python3
"""
Run all ATST tests on STM32F769I-Discovery and capture results.
Prompts user to rate screen appearance for display tests.
"""

import serial
import time
import sys
from datetime import datetime

# PORT = "/dev/cu.usbmodem3657395133351"
PORT = "/dev/cu.usbmodem3657395133351"
BAUD = 115200
TIMEOUT = 300  # 5 minutes per test

# Tests to run
TESTS = [
    ("ATST0", "ESP8266 Weather Test"),
    ("ATST1", "Display Test"),
    ("ATST2", "LED Test"),
    ("ATST3", "RTC Test"),
    ("ATST4", "WiFi Connection Test"),
    ("ATST5", "NTP Sync Test"),
    ("ATST6", "HTTP Request Test"),
    ("ATST7", "Audio Codec Test"),
    ("ATST8", "SDRAM Test"),
    ("ATST9", "Touchscreen Test"),
    ("ATST10", "Run All Tests"),
    # LTDC Timing Tests (display tests that may show visual artifacts)
    ("ATST141", "LTDC Timing: Current (HBP=34)"),
    ("ATST142", "LTDC Timing: OTM8009A Std"),
    ("ATST143", "LTDC Timing: NT35510 Ref"),
    ("ATST144", "LTDC Timing: Conservative"),
    ("ATST145", "LTDC Timing: Alternative"),
    ("ATST146", "LTDC Timing: Minimal"),
    ("ATST147", "Corner Number Display (Diagnostic)"),
]

# Display tests that need screen appearance rating
DISPLAY_TESTS = {
    "ATST1", "ATST141", "ATST142", "ATST143",
    "ATST144", "ATST145", "ATST146", "ATST147"
}

def run_test(ser, test_cmd, test_name, test_num):
    """Run a single test and capture output."""
    print(f"\n{'='*70}")
    print(f"Running Test {test_num}/{len(TESTS)}: {test_name}")
    print(f"Command: {test_cmd}")
    print(f"{'='*70}")

    # Clear any pending input
    ser.reset_input_buffer()

    # Send command
    ser.write((test_cmd + "\r\n").encode())
    ser.flush()

    # Capture output
    output = []
    start_time = time.time()
    complete_found = False
    timeout_found = False
    fail_found = False

    while time.time() - start_time < TIMEOUT:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    output.append(line)
                    # Check for completion indicators
                    if f"{test_cmd} complete" in line:
                        complete_found = True
                        break
                    if "[TIMEOUT]" in line:
                        timeout_found = True
                    if "[FAIL]" in line:
                        fail_found = True
                    # For weather/API tests, check for connection timeout
                    if "Connection timeout" in line:
                        timeout_found = True
            except:
                pass

    status = "COMPLETE" if complete_found else "TIMEOUT" if timeout_found else "FAIL" if fail_found else "UNKNOWN"

    # Print result summary
    print(f"\nStatus: {status}")
    if len(output) > 0:
        # Show last few lines of output
        print(f"Output preview (last 10 lines):")
        for line in output[-10:]:
            print(f"  {line}")
    else:
        print("No output captured")

    return status, output

def ask_screen_appearance(test_cmd):
    """Ask user to rate screen appearance for display tests."""
    if test_cmd not in DISPLAY_TESTS:
        return None

    print("\n" + "-"*70)
    print("SCREEN APPEARANCE RATING")
    print("-"*70)
    print("Please look at the screen and rate the appearance:")
    print("  1 - Correct (text is clear, properly sized, no distortion)")
    print("  0 - Incorrect (tilted, blurry, sparse, wrong size, artifacts)")
    print("  2 - Close/Unable to determine")
    print("-"*70)

    while True:
        try:
            response = input(f"Rate {test_cmd} screen (1/0/2, or Enter to skip): ").strip()
            if response == "":
                return None  # Skip rating
            if response in ["1", "0", "2"]:
                return int(response)
            print("Please enter 1, 0, 2, or press Enter to skip")
        except KeyboardInterrupt:
            print("\nSkipped")
            return None

def main():
    print("="*70)
    print("STM32F769I-Discovery Test Runner")
    print("="*70)
    print(f"Port: {PORT}")
    print(f"Baud: {BAUD}")
    print(f"Tests: {len(TESTS)}")
    print(f"Display tests needing rating: {len([t for t in TESTS if t[0] in DISPLAY_TESTS])}")
    print("="*70)

    try:
        # Open serial port
        ser = serial.Serial(PORT, BAUD, timeout=10)
        print(f"\nConnected to {PORT}")
        print("Waiting for device to stabilize...")

        # Clear buffer
        time.sleep(2)
        ser.reset_input_buffer()

        # Results storage
        results = []
        screen_ratings = {}

        # Run tests
        for i, (test_cmd, test_name) in enumerate(TESTS, 1):
            status, output = run_test(ser, test_cmd, test_name, i)
            results.append({
                'num': i,
                'cmd': test_cmd,
                'name': test_name,
                'status': status,
                'output_lines': len(output)
            })

            # Ask about screen appearance for display tests
            if test_cmd in DISPLAY_TESTS and status == "COMPLETE":
                time.sleep(1)  # Let user see the screen
                rating = ask_screen_appearance(test_cmd)
                if rating is not None:
                    screen_ratings[test_cmd] = rating

            # Small delay between tests
            time.sleep(2)

            # Allow user to quit early
            if i % 5 == 0:
                print("\n" + "-"*70)
                cont = input("Continue? (Y/n): ").strip().upper()
                if cont == 'N':
                    print("\nTest run interrupted by user.")
                    ser.close()
                    return

        # Save results to file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"test_results_{timestamp}.txt"

        with open(filename, 'w') as f:
            f.write("="*70 + "\n")
            f.write("STM32F769I-Discovery Test Results\n")
            f.write(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Firmware: v0.1.46\n")
            f.write(f"Total Tests: {len(results)}\n")
            f.write("="*70 + "\n\n")

            for r in results:
                f.write(f"Test {r['num']}/{len(TESTS)}: {r['name']}\n")
                f.write(f"Command: {r['cmd']}\n")
                f.write(f"Status: {r['status']}\n")
                if r['cmd'] in screen_ratings:
                    rating = screen_ratings[r['cmd']]
                    f.write(f"Screen Rating: {rating}\n")
                f.write("-"*70 + "\n")

        print(f"\n{'='*70}")
        print(f"Results saved to: {filename}")
        print(f"{'='*70}")

        # Print summary
        print("\n" + "="*70)
        print("TEST SUMMARY")
        print("="*70)

        complete = sum(1 for r in results if r['status'] == 'COMPLETE')
        timeout = sum(1 for r in results if r['status'] == 'TIMEOUT')
        fail = sum(1 for r in results if r['status'] == 'FAIL')

        print(f"  COMPLETE: {complete}/{len(results)}")
        print(f"  TIMEOUT: {timeout}/{len(results)}")
        print(f"  FAIL:     {fail}/{len(results)}")
        print(f"  UNKNOWN:  {len(results) - complete - timeout - fail}/{len(results)}")

        # Screen rating summary
        if screen_ratings:
            print("\n" + "-"*70)
            print("SCREEN RATING SUMMARY")
            print("-"*70)
            correct = sum(1 for r in screen_ratings.values() if r == 1)
            incorrect = sum(1 for r in screen_ratings.values() if r == 0)
            close = sum(1 for r in screen_ratings.values() if r == 2)
            total = correct + incorrect + close
            print(f"  Correct:   {correct}/{total}")
            print(f"  Incorrect: {incorrect}/{total}")
            print(f"  Close:     {close}/{total}")

        ser.close()

    except serial.SerialException as e:
        print(f"\nError opening serial port: {e}")
        print(f"Make sure {PORT} is available and not in use by another application.")
        print("\nAvailable ports:")
        try:
            from serial.tools import list_ports
            for port in list_ports.comports():
                print(f"  {port.device}: {port.description}")
        except ImportError:
            print("  (Unable to list ports - pyserial-tools not installed)")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nTest run interrupted by user.")
        try:
            ser.close()
        except:
            pass
        sys.exit(0)

if __name__ == "__main__":
    main()
