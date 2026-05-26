"""
INA226 Battery Monitor for RPi5
================================
Reads bus voltage from two INA226 modules via I2C (voltage only).
No voltage divider needed — INA226 measures up to 36V directly.
IN+ and IN- are shorted on each module (no current measurement).

Board: CJMCU-226

Wiring per module (8-pin header J2):
  VCC  → RPi 3.3V (Pin 1)      — daisy-chained between both modules
  GND  → central GND bus        — daisy-chained between both modules
  SDA  → RPi GPIO2 (Pin 3)     — daisy-chained between both modules
  SCL  → RPi GPIO3 (Pin 5)     — daisy-chained between both modules
  IN+  → battery positive (before switch)
  IN-  → short to IN+ (solder bridge on the header pins)
  VBS  → leave unconnected
  ALE  → leave unconnected

Address configuration (3-pin header J3 on back of board):
  Module 1 (electronics battery): leave J3 untouched → 0x40
  Module 2 (dynamixel battery):   solder bridge A0 to VCC on J3 → 0x41

Prerequisites:
  pip install smbus2 --break-system-packages

Usage:
  python3 test_ina226.py
"""

import sys
import time

try:
    from smbus2 import SMBus
except ImportError:
    print("smbus2 not installed. Run:")
    print("  pip install smbus2 --break-system-packages")
    sys.exit(1)


# ── INA226 register map ──────────────────────────────────────────────

REG_CONFIG        = 0x00
REG_BUS_VOLTAGE   = 0x02
REG_MANUFACTURER  = 0xFE
REG_DIE_ID        = 0xFF

# ── Configuration ────────────────────────────────────────────────────

I2C_BUS = 1

INA226_ADDR_BATTERY1 = 0x40  # electronics battery (A0=GND, A1=GND)
INA226_ADDR_BATTERY2 = 0x41  # dynamixel battery   (A0=VS,  A1=GND)

# INA226 bus voltage LSB (from datasheet)
BUS_VOLTAGE_LSB_MV = 1.25  # 1.25 mV per bit

# 3S LiPo thresholds
VOLTAGE_FULL     = 12.6   # 4.20V/cell
VOLTAGE_NOMINAL  = 11.1   # 3.70V/cell
VOLTAGE_LOW      = 10.5   # 3.50V/cell
VOLTAGE_CRITICAL = 9.9    # 3.30V/cell


# ── INA226 Driver ────────────────────────────────────────────────────

class INA226:
    """Minimal INA226 driver — bus voltage only."""

    def __init__(self, bus: SMBus, address: int):
        self.bus = bus
        self.addr = address

        # Verify the chip is an INA226
        mfr = self._read_register(REG_MANUFACTURER)
        die = self._read_register(REG_DIE_ID)
        if mfr != 0x5449:  # Texas Instruments
            print(f"  Warning: unexpected manufacturer ID 0x{mfr:04X} (expected 0x5449)")
        if (die >> 4) != 0x226:
            print(f"  Warning: unexpected die ID 0x{die:04X}")

        # Configure: 16 averages, 1.1ms bus voltage conversion,
        # continuous bus voltage only mode
        #
        # Bits [11:9] AVG    = 010 (16 averages)
        # Bits [8:6]  VBUSCT = 100 (1.1ms)
        # Bits [5:3]  VSHCT  = 100 (1.1ms, ignored in bus-only mode)
        # Bits [2:0]  MODE   = 101 (continuous bus voltage only)
        config = 0x4525
        self._write_register(REG_CONFIG, config)

    def _read_register(self, reg: int) -> int:
        """Read a 16-bit register (big-endian)."""
        data = self.bus.read_i2c_block_data(self.addr, reg, 2)
        return (data[0] << 8) | data[1]

    def _write_register(self, reg: int, value: int):
        """Write a 16-bit register (big-endian)."""
        self.bus.write_i2c_block_data(self.addr, reg, [(value >> 8) & 0xFF, value & 0xFF])

    def read_bus_voltage(self) -> float:
        """Read bus voltage in volts."""
        raw = self._read_register(REG_BUS_VOLTAGE)
        return raw * BUS_VOLTAGE_LSB_MV / 1000.0


# ── Display helpers ──────────────────────────────────────────────────

def get_battery_status(voltage: float) -> str:
    if voltage < 1.0:
        return "DISCONNECTED"
    elif voltage <= VOLTAGE_CRITICAL:
        return "CRITICAL - STOP NOW"
    elif voltage <= VOLTAGE_LOW:
        return "LOW - STOP SOON"
    elif voltage <= VOLTAGE_NOMINAL:
        return "OK"
    else:
        return "GOOD"


def get_percentage(voltage: float) -> float:
    if voltage < VOLTAGE_CRITICAL:
        return 0.0
    elif voltage > VOLTAGE_FULL:
        return 100.0
    return round((voltage - VOLTAGE_CRITICAL) / (VOLTAGE_FULL - VOLTAGE_CRITICAL) * 100, 1)


# ── Main ─────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  INA226 Battery Monitor")
    print("  Direct bus voltage measurement (no voltage divider)")
    print("  Press Ctrl+C to stop")
    print("=" * 60)
    print()

    sensors = []

    with SMBus(I2C_BUS) as bus:
        # Try to initialize each sensor
        for name, addr in [("Electronics", INA226_ADDR_BATTERY1),
                           ("Dynamixel",   INA226_ADDR_BATTERY2)]:
            try:
                sensor = INA226(bus, addr)
                sensors.append((name, sensor))
                print(f"  Found INA226 at 0x{addr:02X} ({name})")
            except OSError:
                print(f"  INA226 at 0x{addr:02X} ({name}) not found, skipping")

        if not sensors:
            print("\n  No INA226 sensors found. Check wiring and I2C.")
            return

        print()
        print("-" * 50)

        try:
            while True:
                for name, sensor in sensors:
                    voltage = sensor.read_bus_voltage()
                    status = get_battery_status(voltage)
                    pct = get_percentage(voltage)

                    print(f"  {name:12s} | {voltage:5.2f}V | {pct:5.1f}% | {status}")

                print("-" * 50)

                # Critical battery warning
                for name, sensor in sensors:
                    v = sensor.read_bus_voltage()
                    if 1.0 < v <= VOLTAGE_CRITICAL:
                        print(f"  >>> {name.upper()} BATTERY CRITICAL! SHUT DOWN! <<<")

                time.sleep(2)

        except KeyboardInterrupt:
            print("\nMonitoring stopped.")


if __name__ == "__main__":
    main()
