# ROS2 Robot Workspace

## Architecture

```
 OPERATOR LAPTOP                    RASPBERRY PI 5 (Docker)
┌─────────────────┐               ┌──────────────────────────────┐
│                 │   ethernet    │                              │
│  joy_node       │──────────────▶│  joy_mapper_node             │
│  (publishes     │   /joy topic  │  (converts /joy → /cmd_vel)  │
│   /joy)         │   DDS over    │              │               │
│                 │   network     │              ▼               │
│  Logitech       │               │  dynamixel_driver_node       │
│  Extreme 3D Pro │               │  (converts /cmd_vel → motor  │
│  USB            │               │   commands via U2D2 serial)  │
└─────────────────┘               │              │               │
                                  │              ▼               │
                                  │  U2D2 ──▶ Motors             │
                                  │  ID:1(FL) ID:6(FR)           │
                                  │  ID:8(RL) ID:10(RR)          │
                                  └──────────────────────────────┘
```

## Quick Start

### 1. On the RPi — Build & Run

```bash
# Enter Docker container
docker run -it --rm --privileged --network=host \
  -v /dev:/dev -v /run/udev:/run/udev:ro \
  -v ~/ros2_ws:/root/ros2_ws \
  my_robot

# Inside Docker:
source /opt/ros/jazzy/setup.bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Run motor nodes
ros2 launch dynamixel_driver motor_control.launch.py
```

### 2. On the Operator Laptop — Run Joystick

```bash
# Make sure same ROS_DOMAIN_ID (default 0 is fine if both on same network)
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash

# Run joy_node (plug in Logitech first)
ros2 run joy joy_node
```

### 3. Verify Communication

```bash
# On either machine:
ros2 topic list          # should see /joy, /cmd_vel, /motor_status
ros2 topic echo /joy     # see joystick data
ros2 topic echo /cmd_vel # see velocity commands
```

## Network Setup

Both machines must be on the same subnet via ethernet. DDS (the
ROS2 middleware) uses multicast by default, which works over a
direct ethernet connection.

If multicast doesn't work, set both machines to use the same
DDS config with unicast peers. Create a file `cyclonedds.xml`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <NetworkInterfaceAddress>eth0</NetworkInterfaceAddress>
    </General>
    <Discovery>
      <Peers>
        <Peer address="OPERATOR_IP"/>
        <Peer address="RPI_IP"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

Then export on both machines:
```bash
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Package Structure

```
ros2_ws/
└── src/
    ├── dynamixel_driver/          # Motor control package
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── config/
    │   │   └── motor_params.yaml  # Motor IDs, baudrate, limits
    │   ├── launch/
    │   │   └── motor_control.launch.py
    │   └── src/
    │       ├── dynamixel_driver_node.cpp  # /cmd_vel → motors
    │       └── joy_mapper_node.cpp        # /joy → /cmd_vel
    │
    └── robot_bringup/             # Top-level launch (for later)
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/
        └── launch/
```

## Topics

| Topic          | Type                       | Publisher          | Subscriber            |
|---------------|----------------------------|--------------------|-----------------------|
| /joy          | sensor_msgs/Joy            | joy_node (laptop)  | joy_mapper_node (RPi) |
| /cmd_vel      | geometry_msgs/Twist        | joy_mapper_node    | dynamixel_driver_node |
| /motor_status | std_msgs/Float32MultiArray | dynamixel_driver   | (debug/monitoring)    |

## Tuning

Edit `config/motor_params.yaml`:
- `max_velocity`: max Dynamixel velocity units (100 ≈ 23 RPM, 161 ≈ 37 RPM max)
- `deadzone`: joystick deadzone (0.15 = 15%)
- `reverse_ids`: which motors spin in reverse for forward motion
- `cmd_timeout`: seconds without /cmd_vel before auto-stop (safety)

Edit joy_mapper parameters in `launch/motor_control.launch.py`:
- `axis_linear/angular/block`: which joystick axes do what
- `block_threshold`: how far sideways to block motors
- `enable_button`: set to 0 for trigger as deadman switch (-1 = disabled)

## Troubleshooting

**Motors don't move:**
- Check U2D2 port: `ls /dev/ttyUSB*` inside Docker
- Check motor power: U2D2 Power Hub LED should be on
- Verify motor IDs match config: `ros2 param get /dynamixel_driver_node motor_ids`
- Check /cmd_vel is being published: `ros2 topic echo /cmd_vel`

**No /joy topic on RPi:**
- Check both machines have same `ROS_DOMAIN_ID`
- Check ethernet connection: `ping RPI_IP` from laptop
- Try CycloneDDS unicast config (see Network Setup above)
- Inside Docker, make sure `--network=host` was used

**Wrong motor direction:**
- Add/remove motor IDs from `reverse_ids` in motor_params.yaml
