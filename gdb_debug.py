#!/usr/bin/env python3
"""
GDB Debug Script for STM32F769I-Discovery

This script automates GDB debugging to check:
1. Whether firmware reaches main()
2. Whether FreeRTOS starts
3. Whether USB CDC initializes
4. Register states and variables
"""

import subprocess
import time
import sys
import os

class GDBDebugger:
    def __init__(self, elf_file):
        self.elf_file = elf_file
        self.gdb = None
        self.gdb_path = "/usr/local/bin/arm-none-eabi-gdb"

    def send_command(self, cmd):
        """Send command to GDB"""
        if self.gdb:
            self.gdb.stdin.write(cmd + "\n")
            self.gdb.flush()
            time.sleep(0.1)

    def send_continue(self):
        """Send continue command"""
        self.send_command("continue")

    def send_interrupt(self):
        """Send interrupt command"""
        self.send_command("\003")  # Ctrl+C

    def get_output(self):
        """Read all output from GDB"""
        output = ""
        while True:
            try:
                line = self.gdb.stdout.readline()
                if not line:
                    break
                output += line
            except:
                break
        return output

    def connect(self):
        """Connect to ST-LINK GDB server"""
        print("Connecting to GDB server...")

        # Start GDB
        self.gdb = subprocess.Popen(
            [self.gdb_path, self.elf_file],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=0,
            text=True
        )

        # Connect to target
        self.send_command("target extended-remote :3333")
        time.sleep(0.5)

        # Check connection
        output = self.get_output()
        if "Remote debugging" in output or "connected" in output.lower():
            print("✓ Connected to GDB server")
            return True
        else:
            print("✗ Failed to connect")
            print(f"Output: {output}")
            return False

    def halt(self):
        """Halt the target"""
        self.send_interrupt()
        time.sleep(0.2)

    def reset_and_halt(self):
        """Reset and halt the target"""
        print("Resetting and halting...")
        self.send_command("monitor reset halt")
        time.sleep(1)

    def check_register(self, reg):
        """Check a register value"""
        self.send_command(f"info registers {reg}")
        time.sleep(0.2)
        output = self.get_output()
        return output

    def check_variable(self, var):
        """Check a variable value"""
        self.send_command(f"print {var}")
        time.sleep(0.1)
        output = self.get_output()
        return output

    def disassemble(self, addr, count=10):
        """Disassemble around address"""
        self.send_command(f"x/{count}i 0x{addr:x}")
        time.sleep(0.1)
        output = self.get_output()
        return output

    def read_memory(self, addr, size=4):
        """Read memory at address"""
        self.send_command(f"x/{size}x 0x{addr:x}")
        time.sleep(0.1)
        output = self.get_output()
        return output

    def set_breakpoint(self, location):
        """Set a breakpoint"""
        self.send_command(f"break {location}")
        time.sleep(0.2)

    def check_backtrace(self):
        """Get backtrace"""
        self.send_command("backtrace")
        time.sleep(0.2)
        output = self.get_output()
        return output

    def close(self):
        """Close GDB connection"""
        if self.gdb:
            self.send_command("quit")
            time.sleep(0.5)
            self.gdb.terminate()
            self.gdb.wait()

def print_section(title):
    """Print a section header"""
    print("\n" + "=" * 60)
    print(f" {title}")
    print("=" * 60)

def main():
    elf_file = "build/TestF.elf"

    if not os.path.exists(elf_file):
        print(f"Error: {elf_file} not found!")
        print("Please build the firmware first: make clean && make -j8")
        sys.exit(1)

    print_section("STM32F769I-Discovery GDB Diagnostic")
    print(f"ELF File: {elf_file}")
    print(f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}")

    # Check for OpenOCD or st-util
    print("\nChecking for GDB server...")
    try:
        result = subprocess.run(["pgrep", "-f", "openocd"], capture_output=True)
        if result.returncode == 0:
            print("✓ OpenOCD is running")
        else:
            print("OpenOCD not running. Starting...")
            print("Run this in another terminal:")
            print("  openocd -f interface/stlink.cfg -f target/stm32f7x.cfg")
            sys.exit(1)
    except:
        print("Could not check for OpenOCD")

    # Connect GDB
    debugger = GDBDebugger(elf_file)
    if not debugger.connect():
        sys.exit(1)

    # Reset and halt
    debugger.reset_and_halt()

    # Check PC (Program Counter)
    print_section("Program State Check")
    pc_output = debugger.check_register("pc")
    print("Program Counter (PC):")
    print(pc_output[:500])

    # Check LR (Link Register)
    lr_output = debugger.check_register("lr")
    print("Link Register (LR):")
    print(lr_output[:500])

    # Check SP (Stack Pointer)
    sp_output = debugger.check_register("sp")
    print("Stack Pointer (SP):")
    print(sp_output[:500])

    # Check if in main()
    print_section("Location Check")
    debugger.send_command("list")
    time.sleep(0.2)
    list_output = debugger.get_output()
    print("Current location (if in source):")
    print(list_output[:500])

    # Check USB OTG registers
    print_section("USB OTG HS Register Check")

    # GCCFG register (VBUS sensing disable)
    gccfg = debugger.read_memory(0x40040038, 4)
    print(f"USB GCCFG (0x40040038): {gccfg[:200]}")

    # Check GRSTCTL (Reset control)
    grstctl = debugger.read_memory(0x4004000C, 4)
    print(f"USB GRSTCTL (0x4004000C): {grstctl[:200]}")

    # Check GOTGCTL (OTG control)
    gotgctl = debugger.read_memory(0x40040000, 4)
    print(f"USB GOTGCTL (0x40040000): {gotgctl[:200]}")

    # Check DCFG (Device configuration)
    dcfg = debugger.read_memory(0x40040050, 4)
    print(f"USB DCFG (0x40040050): {dcfg[:200]}")

    # Set breakpoint at MX_USB_DEVICE_Init
    print_section("Setting Breakpoints")
    debugger.set_breakpoint("MX_USB_DEVICE_Init")
    print("Breakpoint set at MX_USB_DEVICE_Init")

    # Continue to breakpoint or for 2 seconds
    print("\nContinuing...")
    debugger.send_continue()

    # Wait a bit and check if we hit breakpoint
    time.sleep(3)

    # Halt and check where we are
    debugger.halt()

    pc_output = debugger.check_register("pc")
    print("\nAfter 3 seconds, PC is at:")
    print(pc_output[:500])

    # Check backtrace
    print_section("Backtrace")
    bt = debugger.check_backtrace()
    print(bt[:800])

    # Close connection
    print("\nClosing GDB...")
    debugger.close()

    print_section("Diagnostic Complete")
    print("\nIf the PC is at a low address (0x080xxxxx), firmware is running.")
    print("If PC is at 0x00000000 or similar, firmware may have crashed.")

if __name__ == "__main__":
    main()
