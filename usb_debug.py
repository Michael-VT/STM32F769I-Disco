#!/usr/bin/env python3
"""
USB CDC Debug Tool for STM32F769I-Discovery

This script helps diagnose USB CDC issues:
1. Lists all USB devices
2. Monitors for CDC device appearance
3. Tests serial communication
"""

import os
import sys
import time
import glob
import subprocess
from datetime import datetime

def print_header(text):
    """Print a formatted header"""
    print("\n" + "=" * 60)
    print(f" {text}")
    print("=" * 60)

def get_usb_devices():
    """Get list of all USB modem devices"""
    devices = glob.glob('/dev/cu.usbmodem*')
    return sorted(devices)

def get_usb_info():
    """Get detailed USB information using system_profiler on macOS"""
    try:
        result = subprocess.run(['system_profiler', 'USB', '-detail'],
                              capture_output=True, text=True)
        return result.stdout
    except Exception as e:
        return None

def find_stm32_devices():
    """Find STM32 devices in USB info"""
    usb_info = get_usb_info()
    if not usb_info:
        return []

    stm32_devices = []
    lines = usb_info.split('\n')
    current_device = None

    for line in lines:
        line = line.strip()
        if 'STM32' in line or 'ST-LINK' in line or 'STMicroelectronics' in line:
            if current_device:
                stm32_devices.append(current_device)
            current_device = [line]
        elif current_device is not None:
            current_device.append(line)
            if not line:  # Empty line ends device
                stm32_devices.append(current_device)
                current_device = None

    if current_device:
        stm32_devices.append(current_device)

    return stm32_devices

def monitor_devices(duration=30):
    """Monitor for USB device changes"""
    print_header("USB Device Monitoring")

    known_devices = set(get_usb_devices())
    print(f"\nKnown devices at start:")
    for dev in known_devices:
        print(f"  - {dev}")

    print(f"\nMonitoring for {duration} seconds...")
    print("Connect/disconnect USB cables to test.\n")

    start_time = time.time()
    last_count = len(known_devices)

    while time.time() - start_time < duration:
        current_devices = set(get_usb_devices())

        # Check for new devices
        new_devices = current_devices - known_devices
        if new_devices:
            for dev in new_devices:
                timestamp = datetime.now().strftime("%H:%M:%S")
                print(f"[{timestamp}] NEW DEVICE: {dev}")
                known_devices.update(new_devices)

        # Check for removed devices
        removed_devices = known_devices - current_devices
        if removed_devices:
            for dev in removed_devices:
                timestamp = datetime.now().strftime("%H:%M:%S")
                print(f"[{timestamp}] REMOVED: {dev}")
                known_devices.difference_update(removed_devices)

        time.sleep(0.5)

    print(f"\nFinal device list:")
    for dev in known_devices:
        print(f"  - {dev}")

def test_serial_connection(device, baudrate=115200):
    """Test serial connection to a device"""
    print(f"\nTesting connection to {device} at {baudrate} baud...")

    try:
        import serial
        ser = serial.Serial(device, baudrate, timeout=2)
        print(f"✓ Connected to {device}")
        print(f"  Port: {ser.name}")
        print(f"  Baudrate: {ser.baudrate}")

        # Try to read any available data
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(f"  Received: {data.decode('utf-8', errors='ignore')[:100]}")
        else:
            print(f"  No data waiting (normal if device just connected)")

        ser.close()
        return True

    except ImportError:
        print("  pyserial not installed. Install with: pip3 install pyserial")
        return None
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

def show_hardware_info():
    """Display hardware connection information"""
    print_header("STM32F769I-Discovery Hardware Info")

    print("""
Board Layout:
┌──────────────────────────────────────────────────────────┐
│                                                          │
│  ┌────────────┐                                         │
│  │   LCD      │                                         │
│  │  Display   │                                         │
│  └────────────┘                                         │
│                                                          │
│                 CN13 (USB User FS)                        │
│              ┌─────────┐                                 │
│              │  USB-C  │  ← Firmware CDC Virtual COM    │
│              └─────────┘                                 │
│                                                          │
│  ┌────────────────────────────────────────────────┐    │
│  │                                                │    │
│  │         Arduino Headers (CN3-CN6)             │    │
│  │         CN2: WiFi Module                      │    │
│  │                                                │    │
│  └────────────────────────────────────────────────┘    │
│                                                          │
│                 CN16 (ST-LINK)                          │
│              ┌─────────┐                                 │
│              │ Mini-USB │  ← Debugger + ST-LINK COM      │
│              └─────────┘                                 │
│                                                          │
└──────────────────────────────────────────────────────────┘

Required Connections:
1. CN16 (ST-LINK)  - For programming (REQUIRED)
2. CN13 (USB User) - For firmware CDC (REQUIRED for Virtual COM!)

The CN13 port is on the RIGHT side of the board.
""")

def main():
    """Main function"""
    print_header("STM32F769I-Discovery USB CDC Diagnostic Tool")
    print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

    # Show current devices
    devices = get_usb_devices()
    print(f"\nCurrent USB modem devices: {len(devices)}")
    for dev in devices:
        print(f"  - {dev}")

    # Show STM32 USB info
    print("\nSearching for STM32 USB devices...")
    stm32_devices = find_stm32_devices()
    if stm32_devices:
        for dev in stm32_devices[:3]:  # Show first 3
            print(f"\nFound: {dev[0][:60]}...")
    else:
        print("No STM32 devices found in system USB info.")

    # Show hardware info
    show_hardware_info()

    # Menu
    print("\nOptions:")
    print("1. Monitor for device changes (30 seconds)")
    print("2. Test serial connection to ST-LINK port")
    print("3. Show detailed USB info")
    print("4. List all USB devices")
    print("5. Exit")

    while True:
        try:
            choice = input("\nEnter choice [1-5]: ").strip()

            if choice == '1':
                monitor_devices()
            elif choice == '2':
                if devices:
                    test_serial_connection(devices[0])
                else:
                    print("No USB devices found!")
            elif choice == '3':
                stm32_devices = find_stm32_devices()
                for dev in stm32_devices:
                    print("\n--- Device ---")
                    for line in dev[:20]:  # Show first 20 lines
                        print(line)
            elif choice == '4':
                devices = get_usb_devices()
                print(f"\nFound {len(devices)} USB modem devices:")
                for dev in devices:
                    print(f"  - {dev}")
            elif choice == '5':
                print("Exiting...")
                break
            else:
                print("Invalid choice. Enter 1-5.")

        except KeyboardInterrupt:
            print("\n\nExiting...")
            break

if __name__ == '__main__':
    main()
