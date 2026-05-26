"""
LED MOSFET Control Test for RPi5
=================================
Controls LEDs through a MOSFET connected to GPIO12.

Wiring:
  RPi GPIO12 (Pin 32)  → MOSFET gate (SIG)
  MOSFET drain         → LED strip negative
  MOSFET source        → GND
  Battery+ (after switch) → LED strip positive

Prerequisites:
  pip install gpiozero lgpio --break-system-packages

  On RPi5, gpiozero needs the lgpio backend. If you get a
  "No module named lgpio" error, install it:
    sudo apt install python3-lgpio

Usage:
  python3 test_led_mosfet.py
"""

import sys
import time

try:
    from gpiozero import PWMLED
except ImportError:
    print("gpiozero not installed. Run:")
    print("  pip install gpiozero lgpio --break-system-packages")
    sys.exit(1)

MOSFET_GPIO = 12


def main():
    led = PWMLED(MOSFET_GPIO)

    print("=" * 45)
    print("  LED MOSFET Test (GPIO12)")
    print("  Press Ctrl+C to stop")
    print("=" * 45)

    try:
        while True:
            print("\n  1. Toggle ON/OFF")
            print("  2. Brightness levels")
            print("  3. Breathing effect")
            print("  4. Strobe")
            print("  0. Turn off and exit")
            choice = input("\n  Select: ").strip()

            if choice == "1":
                print("  ON")
                led.value = 1.0
                time.sleep(2)
                print("  OFF")
                led.value = 0.0
                time.sleep(1)

            elif choice == "2":
                for pct in [0, 25, 50, 75, 100, 75, 50, 25, 0]:
                    led.value = pct / 100.0
                    print(f"  Brightness: {pct}%")
                    time.sleep(0.8)

            elif choice == "3":
                print("  Breathing... (Ctrl+C to stop)")
                try:
                    while True:
                        # Fade in
                        for i in range(0, 101, 2):
                            led.value = i / 100.0
                            time.sleep(0.02)
                        # Fade out
                        for i in range(100, -1, -2):
                            led.value = i / 100.0
                            time.sleep(0.02)
                except KeyboardInterrupt:
                    led.value = 0.0

            elif choice == "4":
                print("  Strobe... (Ctrl+C to stop)")
                try:
                    while True:
                        led.value = 1.0
                        time.sleep(0.05)
                        led.value = 0.0
                        time.sleep(0.05)
                except KeyboardInterrupt:
                    led.value = 0.0

            elif choice == "0":
                led.value = 0.0
                print("  LEDs off. Exiting.")
                break

    except KeyboardInterrupt:
        led.value = 0.0
        print("\n  LEDs off. Exiting.")
    finally:
        led.close()


if __name__ == "__main__":
    main()
