#!/usr/bin/env python3
"""
OAK-D Pro Connection Diagnostic Script
========================================
Quick sanity check: detects OAK-D Pro, reads USB speed,
product name, and calibration data.

Adapted from: https://github.com/nihatguness/camera_config

Prerequisites:
  pip install depthai --break-system-packages

Usage:
  python3 test_oakd_connection.py

First time usage - udev rules:
  echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger
"""

import depthai as dai
import sys


def main():
    print("=" * 50)
    print("OAK-D Pro Connection Diagnostic Tool")
    print("=" * 50)

    print("\nSearching for OAK-D Pro camera...")

    devices = dai.Device.getAllAvailableDevices()

    if not devices:
        print("\n  No devices found!")
        print("\n  Troubleshooting:")
        print("    1. Check the USB-C cable is fully connected")
        print("    2. Try a different USB 3.0 port (blue colored)")
        print("    3. Use the original USB-C cable")
        print("    4. Try: lsusb | grep Movidius")
        return 1

    print(f"\n  Found {len(devices)} device(s):")
    for i, dev in enumerate(devices):
        print(f"    [{i+1}] State: {dev.state.name}")

    print("\n  Attempting to connect to first device...")

    try:
        device = dai.Device()

        print(f"\n  Connected successfully!")
        print(f"    USB Speed: {device.getUsbSpeed().name}")

        try:
            calibration = device.readCalibration()
            eeprom = calibration.getEepromData()
            print(f"    Product:   {eeprom.productName}")
            print(f"    Board:     {eeprom.boardName}")
        except Exception:
            pass

        device.close()
        print("\n  Device is working correctly!")
        return 0

    except Exception as e:
        print(f"\n  Failed to connect: {e}")
        return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n  Interrupted.")
        sys.exit(0)
