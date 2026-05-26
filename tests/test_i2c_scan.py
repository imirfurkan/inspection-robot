"""
I2C Bus Scanner for RPi5
========================
Scans the I2C bus and lists all detected devices.
For this project, expects two INA226 modules for battery voltage monitoring.

Expected devices:
  0x40 — INA226 #1 (electronics battery, A0=GND A1=GND)
  0x41 — INA226 #2 (dynamixel battery,   A0=VS  A1=GND)

Wiring (same for both modules):
  VCC → RPi 3.3V (Pin 1)
  GND → RPi GND  (Pin 6) + central GND bus
  SDA → RPi GPIO2 (Pin 3)
  SCL → RPi GPIO3 (Pin 5)
  IN+ → battery positive (before switch)
  IN- → shorted to IN+ (voltage-only mode)

Prerequisites:
  sudo raspi-config → Interface Options → I2C → Enable
  pip install smbus2 --break-system-packages

Usage:
  python3 test_i2c_scan.py
"""

import sys

try:
    from smbus2 import SMBus
except ImportError:
    print("smbus2 not installed. Run:")
    print("  pip install smbus2 --break-system-packages")
    sys.exit(1)


# INA226 address table (determined by A0 and A1 pin connections)
# Your project uses 0x40 and 0x41.
INA226_ADDRESSES = {
    0x40: "INA226 — Electronics battery (A0=GND, A1=GND)",
    0x41: "INA226 — Dynamixel battery  (A0=VS,  A1=GND)",
    0x44: "INA226 (A0=GND, A1=VS)  — not used in this project",
    0x45: "INA226 (A0=VS,  A1=VS)  — not used in this project",
}

# The two addresses we expect to find
EXPECTED = {0x40, 0x41}

# INA226 register to verify identity
REG_MANUFACTURER = 0xFE  # should read 0x5449 (Texas Instruments)
REG_DIE_ID       = 0xFF  # upper 12 bits should be 0x226

BUS_NUMBER = 1  # RPi5 uses I2C bus 1 (GPIO2=SDA, GPIO3=SCL)


def scan_i2c(bus_number: int) -> list[int]:
    """Scan I2C bus and return list of detected addresses."""
    found = []
    with SMBus(bus_number) as bus:
        for addr in range(0x03, 0x78):  # valid 7-bit I2C range
            try:
                bus.read_byte(addr)
                found.append(addr)
            except OSError:
                pass
    return found


def verify_ina226(bus: SMBus, addr: int) -> bool:
    """Check if device at addr is actually an INA226 by reading ID registers."""
    try:
        # Read manufacturer ID (should be 0x5449 = Texas Instruments)
        data = bus.read_i2c_block_data(addr, REG_MANUFACTURER, 2)
        mfr = (data[0] << 8) | data[1]

        # Read die ID (upper 12 bits should be 0x226)
        data = bus.read_i2c_block_data(addr, REG_DIE_ID, 2)
        die = (data[0] << 8) | data[1]

        is_ti = (mfr == 0x5449)
        is_226 = ((die >> 4) == 0x226)

        return is_ti and is_226
    except OSError:
        return False


def main():
    print(f"Scanning I2C bus {BUS_NUMBER}...")
    print()

    devices = scan_i2c(BUS_NUMBER)

    if not devices:
        print("No devices found.")
        print()
        print("Check:")
        print("  - I2C enabled?  sudo raspi-config → Interface Options → I2C")
        print("  - SDA on GPIO2 (Pin 3), SCL on GPIO3 (Pin 5)?")
        print("  - Both INA226 modules powered? (VCC to 3.3V, GND to GND)")
        print("  - Common GND between batteries, INA226s, and RPi?")
        return

    print(f"Found {len(devices)} device(s):")
    print()

    with SMBus(BUS_NUMBER) as bus:
        for addr in devices:
            label = INA226_ADDRESSES.get(addr, "unknown device")
            verified = ""

            # If it's at an INA226 address, verify it's really an INA226
            if addr in INA226_ADDRESSES:
                if verify_ina226(bus, addr):
                    verified = "  [verified INA226]"
                else:
                    verified = "  [NOT an INA226 — wrong ID registers]"

            print(f"  0x{addr:02X}  ({addr:3d})  —  {label}{verified}")

    # Check if expected devices are present
    print()
    found_expected = EXPECTED.intersection(devices)
    missing = EXPECTED - found_expected

    if not missing:
        print("All expected INA226 modules found.")
    else:
        print("Missing expected devices:")
        for addr in sorted(missing):
            print(f"  0x{addr:02X} — {INA226_ADDRESSES[addr]}")
        print()
        print("Troubleshooting:")
        if 0x40 in missing and 0x41 in missing:
            print("  Both missing — likely an I2C wiring issue (SDA/SCL/power).")
        elif 0x40 in missing:
            print("  Electronics INA226 missing — check A0 pin is tied to GND.")
        elif 0x41 in missing:
            print("  Dynamixel INA226 missing — check A0 pin is tied to VS (3.3V).")
        print("  Verify with: sudo i2cdetect -y 1")


if __name__ == "__main__":
    main()
