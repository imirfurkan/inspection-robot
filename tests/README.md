# Robot Test Suite — Phase 1 (Hardware Verification)

Standalone test files for verifying each hardware component individually.
No ROS2 dependency — these are plain Python/C++ scripts.

## Testing Order

Run these in order. Each test verifies a layer of the system before
moving to the next.

### 1. I2C Bus Scanner (`test_i2c_scan.py`)
Detects all I2C devices. Run this first to confirm INA226 is visible.
```bash
pip install smbus2 --break-system-packages
python3 test_i2c_scan.py
# Expected: 0x40 (INA226 battery 1), optionally 0x41 (battery 2)
```

### 2. INA226 Battery Monitor (`test_ina226.py`)
Reads battery voltage and current. Confirms power system is wired correctly.
```bash
python3 test_ina226.py
# Expected: voltage readings matching multimeter, current if wired in-line
```

### 3. LED MOSFET Control (`test_led_mosfet.py`)
Toggles LEDs via MOSFET on GPIO12. Confirms MOSFET wiring and GPIO.
```bash
pip install gpiozero lgpio --break-system-packages
python3 test_led_mosfet.py
```

### 4. Rear Camera (`test_rear_camera.py`)
Live video from CSI camera (RPi Camera V2 NoIR / IMX219).
```bash
sudo apt install -y python3-picamera2 python3-opencv
python3 test_rear_camera.py              # GUI mode (needs display)
python3 test_rear_camera.py --headless   # saves frames to disk (SSH)
```

### 5. Front Camera — OAK-D Pro (`test_oakd_pro.py`)
Color + depth feed with IR projector/flood controls.
```bash
pip install depthai opencv-python --break-system-packages
python3 test_oakd_pro.py
```

### 6. Logitech Controller (`test_logitech.py`)
Reads all joystick axes, buttons, and D-pad in real-time.
```bash
sudo apt install -y joystick
pip install pygame --break-system-packages
jstest /dev/input/js0           # quick OS-level verify
python3 test_logitech.py        # detailed readout
```

### 7. U2D2 Link Test (`test_u2d2_link.cpp`)
Scans the Dynamixel bus and reports all discovered motors.
```bash
# Install Dynamixel SDK first:
cd ~ && git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd DynamixelSDK/c++/build/linux_aarch64
make && sudo make install

# Compile and run:
cd /path/to/robot/tests
g++ -o test_u2d2_link test_u2d2_link.cpp -ldxl_aarch64_cpp
sudo ./test_u2d2_link
```

### 8. Dynamixel Motor Test (`test_dynamixel.cpp`)
Velocity and position control for XH540-W140-R motors.
```bash
g++ -o test_dynamixel test_dynamixel.cpp -ldxl_aarch64_cpp
sudo ./test_dynamixel
# WARNING: Motors will spin. Elevate the robot or remove wheels.
```

## File Summary

| File | Language | Component | I/O |
|------|----------|-----------|-----|
| test_i2c_scan.py | Python | I2C bus | I2C scan |
| test_ina226.py | Python | INA226 | I2C (0x40, 0x41) |
| test_led_mosfet.py | Python | MOSFET + LEDs | GPIO12 |
| test_rear_camera.py | Python | RPi Camera V2 | CSI ribbon |
| test_oakd_pro.py | Python | OAK-D Pro | USB 3.0 |
| test_logitech.py | Python | Logitech F310/F710 | USB / Bluetooth |
| test_u2d2_link.cpp | C++ | U2D2 + Dynamixel bus | USB serial |
| test_dynamixel.cpp | C++ | XH540-W140-R motors | USB serial (via U2D2) |

## Phase 2 — ROS2 Integration

After all tests pass, these files become the basis for ROS2 nodes:
- test_ina226.py → battery_monitor_node (publishes sensor_msgs/BatteryState)
- test_rear_camera.py → rear_camera_node (publishes sensor_msgs/Image)
- test_oakd_pro.py → replaced by depthai_ros package
- test_logitech.py → replaced by ros2 run joy joy_node
- test_dynamixel.cpp → dynamixel_driver_node (subscribes to geometry_msgs/Twist)
- test_led_mosfet.py → led_controller_node (subscribes to std_msgs/Float32)

The digital servo (DS3235 steering) code you already have also becomes a node.
