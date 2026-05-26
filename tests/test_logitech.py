"""
Logitech Extreme 3D Pro Joystick Test
=======================================
Reads and displays all joystick axes and button states in real-time.

*** RUN THIS ON THE OPERATOR COMPUTER, NOT THE RPi. ***
The joystick plugs into the operator computer via USB.
The RPi is 10m+ away on the robot — no USB cable reaches.

Extreme 3D Pro layout:
  Axes:    0=Stick X, 1=Stick Y, 2=Twist Z, 3=Throttle slider
           4=HAT X, 5=HAT Y (POV hat as axes)
  Buttons: 0=Trigger, 1-11=Base/top buttons (12 total)
  Raw axis range: -32767 to +32767 (pygame normalizes to -1.0 to +1.0)

Your previous robot mapping:
  Axis 0 (X)     → blocked motors/steering at 50% deflection
  Axis 1 (Y)     → motor speed (forward = negative raw, inverted in code)
  Axis 2 (Twist) → discrete steering (left/right past threshold)
  Button 3       → center steering servo

ROS2 integration:
  On the operator computer, run:
    ros2 run joy joy_node
  This publishes sensor_msgs/Joy to /joy topic over the network.
  The RPi's Dynamixel node subscribes to /joy — no custom networking
  code needed. Both machines must share the same ROS_DOMAIN_ID.

Setup:
  Joystick → USB → operator computer
  Operator computer → PoE ethernet → RPi (on the robot)

Prerequisites (on operator computer):
  sudo apt install -y joystick
  pip install pygame --break-system-packages

  Verify joystick is detected:
    ls /dev/input/js*
    jstest /dev/input/js0

Usage:
  python3 test_logitech.py
"""

import sys
import os

try:
    import pygame
except ImportError:
    print("pygame not installed. Run:")
    print("  pip install pygame --break-system-packages")
    sys.exit(1)


# Logitech Extreme 3D Pro — 12 buttons
# Numbering matches Linux /dev/input/js0 (and your previous working code)
BUTTON_NAMES = {
    0:  "Trigger",
    1:  "Thumb (side)",
    2:  "Button 3 (top left)",
    3:  "Button 4 (top right)",
    4:  "Button 5 (top far left)",
    5:  "Button 6 (top far right)",
    6:  "Base 7",
    7:  "Base 8",
    8:  "Base 9",
    9:  "Base 10",
    10: "Base 11",
    11: "Base 12",
}

# Logitech Extreme 3D Pro — 6 axes
# Axis 0: Stick X (left/right)    → -1.0 = left,  +1.0 = right
# Axis 1: Stick Y (forward/back)  → -1.0 = forward, +1.0 = back (inverted!)
# Axis 2: Twist Z (rotation)      → -1.0 = twist left, +1.0 = twist right
# Axis 3: Throttle slider          → -1.0 = full forward, +1.0 = full back
# Axis 4: HAT X (POV left/right)  → -1.0 = left, +1.0 = right
# Axis 5: HAT Y (POV up/down)     → -1.0 = up, +1.0 = down
AXIS_NAMES = {
    0: "Stick X",
    1: "Stick Y",
    2: "Twist Z",
    3: "Throttle",
    4: "HAT X",
    5: "HAT Y",
}

# Deadzone for analog sticks
DEADZONE = 0.1


def main():
    os.environ["SDL_VIDEODRIVER"] = "dummy"  # no display needed
    pygame.init()
    pygame.joystick.init()

    print("=" * 55)
    print("  Logitech Extreme 3D Pro Test")
    print("  Press Ctrl+C to stop")
    print("=" * 55)
    print()

    count = pygame.joystick.get_count()
    if count == 0:
        print("  No joystick detected.")
        print("  Check:")
        print("    - USB cable connected?")
        print("    - ls /dev/input/js*")
        print("    - Try: jstest /dev/input/js0")
        return

    # Use first joystick
    joy = pygame.joystick.Joystick(0)
    joy.init()

    print(f"  Controller: {joy.get_name()}")
    print(f"  Axes:       {joy.get_numaxes()}")
    print(f"  Buttons:    {joy.get_numbuttons()}")
    print()
    print("  Move stick, twist, throttle, and press buttons.")
    print("  Axes with values near 0 are filtered (deadzone = {:.1f})".format(DEADZONE))
    print("-" * 55)

    try:
        while True:
            pygame.event.pump()

            # Print active axes
            active_axes = []
            for i in range(joy.get_numaxes()):
                val = joy.get_axis(i)
                if abs(val) > DEADZONE:
                    name = AXIS_NAMES.get(i, f"Axis {i}")
                    active_axes.append(f"{name}: {val:+.3f}")

            # Print pressed buttons
            pressed = []
            for i in range(joy.get_numbuttons()):
                if joy.get_button(i):
                    name = BUTTON_NAMES.get(i, f"Button {i}")
                    pressed.append(name)

            # Build output line
            parts = []
            if active_axes:
                parts.append(" | ".join(active_axes))
            if pressed:
                parts.append("BTN: " + ", ".join(pressed))

            if parts:
                line = "  " + "  |  ".join(parts)
                print(f"\r{line:<100}", end="", flush=True)

            pygame.time.wait(50)  # 20Hz polling

    except KeyboardInterrupt:
        print("\n\n  Controller test stopped.")
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()
