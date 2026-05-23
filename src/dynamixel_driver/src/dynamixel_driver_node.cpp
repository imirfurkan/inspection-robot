Duct Inspection Robot
/
Claude finished the response

I have created docker ros2 jazzy on pi5. Now I want to start the ros2 structure nodes etc transitioning from test codes. I would like to start with joystick and dynamixel motors
10:33 PM

Let me check if you have any previous conversations about this project for context.
Searched memory

Great, I have a full picture of your project now. You've got the Docker container running with ros:jazzy-ros-base, the test codes are written and some tested, and now you want to transition into proper ROS2 packages starting with joystick → Dynamixel motor control.

Let me create the full ROS2 package structure for you. Before I write the code, let me confirm a couple things from our previous conversations and ask what I'm not sure about:

From what I remember:

    Joystick: Logitech Extreme 3D Pro, joy_node handles input (no custom code needed)
    Motors: Dynamixel XH540-W140-R via U2D2, Protocol 2.0, baudrate 57600
    Architecture: joy_node → /joy topic → your dynamixel_driver_node
    Your old joystick mapping: axis 1 = speed (inverted), axis 0 = blocking at 50%, axis 2 = steering, button 3 = center servo

Their IDs are 1, 6, 8, 10. I have 4 motors.
10:34 PM

Got it — 4 motors with IDs 1, 6, 8, 10. And the joystick — is it plugged into the RPi directly, or into a separate operator laptop that communicates over the network?

Either way, let me start building the package. I'll assume the joystick is on the RPi for now (simpler starting point — you can always move it to a remote machine later by just running joy_node there with the same ROS_DOMAIN_ID).

Let me read the relevant skill first, then create your workspace structure.
Check frontend-design skill for any relevant info

Now let me create the full ROS2 package structure. This will be a C++ package with ament_cmake since the Dynamixel SDK is C++.
Created 2 files, ran a command
Claude's response was interrupted.

Joystick is plugged to the operator computer, which is connected to rpi via ethernet
10:34 PM

Good — that's the cleaner setup. So the architecture is:

Operator laptop (runs joy_node, publishes /joy) → Ethernet → RPi Docker (runs dynamixel_driver_node, subscribes to /joy, commands motors via U2D2)

Both machines need the same ROS_DOMAIN_ID and DDS needs to work over the ethernet link. Let me continue building the package files.
Created 7 files

Now let me package everything up for you to download:
Ran 2 commands, read a file

Here's your complete ROS2 workspace. Let me walk you through the design decisions:

Why two nodes instead of one? The joy_mapper_node sits between joy_node and the motor driver. It converts raw joystick axes into /cmd_vel (a standard geometry_msgs/Twist). This means later when you add autonomous navigation or keyboard control, they just publish to /cmd_vel too — the motor driver doesn't care where commands come from.

The data flow is:

Operator laptop              RPi Docker
joy_node → /joy ──ethernet──▶ joy_mapper_node → /cmd_vel → dynamixel_driver_node → U2D2 → motors

Key things in motor_params.yaml you should verify:

    reverse_ids: [6, 10] — I assumed IDs 6 and 10 are the right-side motors. If your motor layout is different, adjust this. Getting it wrong just means the robot spins instead of going straight.
    max_velocity: 100 — conservative starting point (~23 RPM). The XH540-W140 max is ~161 units. Increase once you're confident.
    baudrate: 57600 — factory default. If you changed it during testing, update this.

To get it running, copy the ros2_ws/src/ contents into ~/ros2_ws/src/ on your RPi, then follow the Quick Start in the README. The first thing you'll want to verify is that /joy messages from the laptop actually reach the RPi — ros2 topic echo /joy inside Docker. If DDS discovery doesn't work over your ethernet link, there's a CycloneDDS unicast config in the README.

What would you like to tackle next — testing the build, adding the DS3235 steering servo, or something else?
Readme
Document · MD 
Dynamixel driver node
CPP 
Joy mapper node
CPP 
Motor params
YAML 
Motor control.launch
PY 
Cmakelists
TXT 
Package
XML 
Cmakelists
TXT 
Package
XML 

How to set up both ros domain ids same
10:40 PM

It's just an environment variable you export on both machines. Pick any number 0–232 (default is 0).

On the operator laptop:
bash

export ROS_DOMAIN_ID=5
source /opt/ros/jazzy/setup.bash
ros2 run joy joy_node

On the RPi (inside Docker):
bash

export ROS_DOMAIN_ID=5
source /opt/ros/jazzy/setup.bash
ros2 launch dynamixel_driver motor_control.launch.py

That's it. Both machines with the same ROS_DOMAIN_ID on the same ethernet subnet will automatically discover each other via DDS multicast.

To avoid typing it every time, add it to your shell config:

On the laptop:
bash

echo "export ROS_DOMAIN_ID=5" >> ~/.bashrc

On the RPi (add it to your Dockerfile so it's baked into the image):
dockerfile

ENV ROS_DOMAIN_ID=5

Or pass it when running the container:
bash

docker run -it --rm --privileged --network=host \
  -e ROS_DOMAIN_ID=5 \
  -v /dev:/dev -v /run/udev:/run/udev:ro \
  -v ~/ros2_ws:/root/ros2_ws \
  my_robot

Why not just use 0 (the default)? You can — it works fine if you're on your own network. Using a non-zero ID avoids collisions if someone else on the same network is also running ROS2 (like in a lab or shared workspace). If it's just you at home, 0 is fine and you don't need to export anything.

Quick test once both are running:
bash

# On the RPi, inside Docker:
ros2 topic list

If you see /joy in the list, the laptop's joy_node is visible. If not, the issue is network/DDS, not the domain ID.

Do I ned ros localhost only 0
10:43 PM

ROS_LOCALHOST_ONLY controls whether ROS2 traffic stays on the local machine or goes out to the network. For your setup:

On the RPi (Docker): must be 0 (or unset) On the operator laptop: must be 0 (or unset)
bash

export ROS_LOCALHOST_ONLY=0

If either machine has it set to 1, DDS won't send or receive messages over the network — the laptop's /joy topic would never reach the RPi.

Since 0 is the default, you probably don't need to set it at all. But if you previously set it to 1 (some tutorials do this), make sure to unset it:
bash

unset ROS_LOCALHOST_ONLY

Or check what it's currently set to:
bash

echo $ROS_LOCALHOST_ONLY

If it prints nothing or 0, you're fine. If it prints 1, that's your problem.

what is DDS, we previously said UDS is good
10:46 PM

You might be mixing up two different things:

DDS (Data Distribution Service) is the communication middleware that ROS2 uses under the hood. When you publish to /joy on the laptop, DDS is what actually sends those messages over the ethernet to the RPi. You don't interact with it directly — ROS2 handles it. CycloneDDS is the default DDS implementation in Jazzy. This isn't something you chose; it comes with ROS2.

UDS — I'm not sure we discussed this specifically. Are you thinking of U2D2? That's the Robotis USB-to-Dynamixel adapter that connects your RPi to the motors via serial. Completely different layer — U2D2 is hardware for motor communication, DDS is software for ROS2 node-to-node communication.

The stack looks like this:

Joystick → USB → Laptop
  → joy_node → DDS (over ethernet) → RPi Docker
    → joy_mapper_node → dynamixel_driver_node
      → USB serial → U2D2 → Dynamixel motors

DDS handles the laptop↔RPi network part. U2D2 handles the RPi↔motors serial part. They don't overlap.

Is there something else you were referring to with UDS?

I am unsure about the file structure. Where should my tests files sit, seperate from ros2 ws?
10:48 PM

Good question. You have two options, and I'd recommend keeping them inside the workspace but separate from the ROS2 packages:

~/ros2_ws/
├── src/
│   ├── dynamixel_driver/       # ROS2 package (what we just made)
│   ├── robot_bringup/          # ROS2 package
│   └── README.md
│
├── tests/                      # Your standalone test scripts
│   ├── test_i2c_scan.py
│   ├── test_ina226.py
│   ├── test_led_mosfet.py
│   ├── test_rear_camera.py
│   ├── test_oakd_pro.py
│   ├── test_logitech.py
│   ├── test_u2d2_link.cpp
│   ├── test_dynamixel.cpp
│   └── README.md
│
└── install/                    # auto-generated by colcon build
    build/
    log/

Why this works well:

The tests/ folder sits at the workspace root, not inside src/. colcon build only looks inside src/ for ROS2 packages, so it completely ignores tests/. Your test scripts stay standalone — python3 test_ina226.py or g++ test_dynamixel.cpp — no ROS2 needed to run them.

They're still in the same volume mount (~/ros2_ws) so they're accessible inside Docker. And they're right next to the ROS2 code they evolved from, which makes referencing them easy.

The alternative — putting tests in a completely separate folder like ~/robot_tests/ — works too but means an extra volume mount in your docker run command and splits related code across two locations for no real benefit.

One thing to not do: don't put them inside src/dynamixel_driver/ or any ROS2 package folder. colcon would try to interpret them and you'd get confusing build errors.

So my project files should be in ros2 w in raspberry pi and not robot_docker?
10:49 PM

Right. Here's how the two folders relate:

~/robot_docker/
└── Dockerfile              # just the recipe to build the image
                            # nothing else goes here

~/ros2_ws/
├── src/                    # your actual project code
│   ├── dynamixel_driver/
│   └── robot_bringup/
└── tests/                  # standalone test scripts

robot_docker/ only holds the Dockerfile. It's a build tool, not a workspace. You built your image from it once with docker build -t my_robot . and you only touch it again when you need to add a system package.

ros2_ws/ is your actual project — all your code, configs, launch files, and tests live here. It gets mounted into the container with -v ~/ros2_ws:/root/ros2_ws, so any changes you make on the RPi (outside Docker) are immediately visible inside Docker, and vice versa.

Think of it this way: the Dockerfile defines what's installed (Ubuntu, ROS2, dynamixel_sdk, etc.). The workspace contains what you write (your nodes, launch files, configs). They're separate concerns.

you sent me 9 files earlier, where should they sit in the project folder
11:06 PM

All 9 files go into ~/ros2_ws/ on the RPi. Here's exactly where each one lands:

~/ros2_ws/
├── src/
│   ├── README.md
│   │
│   ├── dynamixel_driver/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── config/
│   │   │   └── motor_params.yaml
│   │   ├── launch/
│   │   │   └── motor_control.launch.py
│   │   └── src/
│   │       ├── dynamixel_driver_node.cpp
│   │       └── joy_mapper_node.cpp
│   │
│   └── robot_bringup/
│       ├── CMakeLists.txt
│       └── package.xml
│
└── tests/
    ├── test_i2c_scan.py          ← your old test files
    ├── test_ina226.py               go here
    ├── test_dynamixel.cpp
    └── ...

To put them in place on the RPi:
bash

# Create the folder structure
mkdir -p ~/ros2_ws/src/dynamixel_driver/{src,config,launch}
mkdir -p ~/ros2_ws/src/robot_bringup/{config,launch}
mkdir -p ~/ros2_ws/tests

# Then copy/paste each file into its location
# or transfer the downloaded files via scp from your laptop:
scp -r ros2_ws/src/* pi@RPI_IP:~/ros2_ws/src/

The files I sent you are the 9 in src/. Your old test scripts from the previous conversation go into tests/ alongside them.

I have downloaded and put the codes as you suggested. Now, go through each file, what it does, why it is placed there. Then, show how to builld the project file. Then, as how to sync ros2_ws insides to github. previously, /tests were synced to github but as I moved those from inspection-robot, things changed
11:28 PM

Let's go through everything.
What each file does and why it's there

src/ — This is where colcon build looks for ROS2 packages. Every subfolder here with a package.xml is a package.

src/README.md — Project documentation. The architecture diagram, build instructions, topic table, troubleshooting. Lives at the top of src/ because it describes the whole project.

src/dynamixel_driver/ — Your main package. It contains two nodes and all their supporting files.

    package.xml — The package's identity card. Tells ROS2 the package name, what it depends on (rclcpp, sensor_msgs, dynamixel_sdk, etc.), and that it uses ament_cmake as the build system. colcon reads this first to know what to install before building.
    CMakeLists.txt — The build recipe. Tells CMake which .cpp files to compile into which executables, what libraries to link, and what folders to install. Without this, colcon build wouldn't know what to do with your source files.
    config/motor_params.yaml — Runtime parameters loaded by the launch file. Motor IDs [1, 6, 8, 10], which ones are reversed [6, 10], baudrate, serial port, velocity limits. You edit this file to tune behavior without recompiling.
    launch/motor_control.launch.py — Starts both nodes with one command. Loads the YAML parameters into dynamixel_driver_node and passes joystick mapping parameters to joy_mapper_node. Instead of running two separate ros2 run commands, you do ros2 launch dynamixel_driver motor_control.launch.py.
    src/joy_mapper_node.cpp — Subscribes to /joy (raw joystick axes/buttons from the operator laptop), applies your deadzone, X-axis blocking, and inversion logic, then publishes a clean /cmd_vel (Twist message with linear.x and angular.z). This is the "brain" that interprets your Logitech stick movements. It's a separate node so that later, autonomous navigation can also publish /cmd_vel and the motor driver doesn't need to change.
    src/dynamixel_driver_node.cpp — Subscribes to /cmd_vel, does differential drive math (left wheels = linear - angular, right wheels = linear + angular), reverses the right-side motor directions, and writes velocity commands to the 4 Dynamixels via the U2D2. Also has a safety watchdog — if no /cmd_vel arrives for 0.5 seconds, it stops all motors. Publishes /motor_status with velocity, temperature, and voltage readings for debugging.

src/robot_bringup/ — An empty shell for now. When you add more nodes later (camera, battery monitor, LEDs), this package will hold the top-level launch file that starts everything. Its package.xml and CMakeLists.txt are minimal — they just install launch and config folders.
How to build

Inside Docker on the RPi:
bash

# Enter the container
docker run -it --rm --privileged --network=host \
  -v /dev:/dev -v /run/udev:/run/udev:ro \
  -v ~/ros2_ws:/root/ros2_ws \
  -e ROS_DOMAIN_ID=5 \
  my_robot

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Go to workspace root (not src/)
cd /root/ros2_ws

# Build
colcon build --symlink-install

# Source your built packages
source install/setup.bash

# Run
ros2 launch dynamixel_driver motor_control.launch.py

--symlink-install means config and launch files are symlinked rather than copied, so you can edit them without rebuilding. You only need to rebuild when you change .cpp files.

If the build fails complaining about dynamixel_sdk, make sure your Dockerfile has ros-jazzy-dynamixel-sdk installed. Check your Dockerfile has:
dockerfile

RUN apt update && apt install -y \
    ros-jazzy-joy \
    ros-jazzy-dynamixel-sdk \
    && rm -rf /var/lib/apt/lists/*

If you changed the Dockerfile, rebuild the image:
bash

# On the RPi, outside Docker
cd ~/robot_docker
docker build -t my_robot .

GitHub sync

Your old repo was probably called inspection-robot with just the tests/ folder. Now the structure has changed. Here's how to set it up:
bash

# On the RPi, in the ros2_ws folder
cd ~/ros2_ws

# Initialize git if not already
git init

# Create .gitignore — critical, don't commit build artifacts
cat > .gitignore << 'EOF'
build/
install/
log/
.colcon/
EOF

# Add everything
git add .
git commit -m "ROS2 workspace: dynamixel driver + joy mapper"

# Create a new repo on GitHub (or rename the old one), then:
git remote add origin git@github.com:YOURUSERNAME/inspection-robot.git
git branch -M main
git push -u origin main

The key is the .gitignore — build/, install/, and log/ are generated by colcon build and should never be committed. Only src/ and tests/ go to GitHub.

If you want to keep the old test files in the same repo, move them into ~/ros2_ws/tests/ first:
bash

# If they're still in the old location
cp ~/robot/tests/* ~/ros2_ws/tests/

Your repo will then look like:

inspection-robot/          (GitHub)
├── .gitignore
├── src/
│   ├── README.md
│   ├── dynamixel_driver/
│   └── robot_bringup/
└── tests/
    ├── test_dynamixel.cpp
    ├── test_logitech.py
    └── ...

Want to try the build now, or set up GitHub first?

will the github rpeo name be ros2ws
11:31 PM

Up to you, but I'd recommend keeping it as inspection-robot (or whatever your project name is) rather than ros2_ws.

ros2_ws is a generic workspace name — it describes the ROS2 convention, not your project. If you ever have a second robot or project, you'd want another ros2_ws and the names would collide. The repo name should describe what's inside, not the folder convention.

On the RPi, the folder can still be called ~/ros2_ws/ — that's just a local path. The GitHub repo name is independent:
bash

cd ~/ros2_ws
git remote add origin git@github.com:YOURUSERNAME/inspection-robot.git

So locally it's ~/ros2_ws/, on GitHub it's inspection-robot. No conflict.

Your repo will then look like:

inspection-robot/          (GitHub)
├── .gitignore
├── src/
│   ├── README.md

11:46 PM

You're right to question that — the GitHub repo contents won't have an inspection-robot/ wrapper folder. What's on GitHub is the contents of ~/ros2_ws/, not a folder named inspection-robot.

When someone clones it, git creates the folder from the repo name automatically:
bash

git clone git@github.com:YOURUSERNAME/inspection-robot.git
# creates: ./inspection-robot/src/dynamixel_driver/...

But on your RPi, the structure is:

~/ros2_ws/                  ← git root is

Claude's response was interrupted.

my previous messsage was a mistake. admin@admin:~/ros2_ws $ docker run -it --rm --privileged --network=host\  -v /dev:/dev -v /run/udev:/run/udev:ro \  -v ~/ros2_ws:/root/ros2_ws \  my_robot root@admin:~/ros2_ws# source /opt/ros/jazzy/setup.bash cd /root/ros2_ws colcon build --symlink-install Starting >>> dynamixel_driver Finished <<< dynamixel_driver [15.1s]                        Starting >>> robot_bringup --- stderr: robot_bringup                           CMake Error at ament_cmake_symlink_install/ament_cmake_symlink_install.cmake:105 (message):  ament_cmake_symlink_install_directory() can't find  '/root/ros2_ws/src/robot_bringup/launch' Call Stack (most recent call first):  ament_cmake_symlink_install/ament_cmake_symlink_install.cmake:319 (ament_cmake_symlink_install_directory)  cmake_install.cmake:46 (include) --- Failed   <<< robot_bringup [1.03s, exited with code 1] Summary: 1 package finished [16.3s]  1 package failed: robot_bringup  1 package had stderr output: robot_bringup root@admin:~/ros2_ws#
11:46 PM

The robot_bringup CMakeLists.txt tries to install launch/ and config/ directories, but you haven't created them yet (the package is an empty shell). Quick fix — create the empty folders on the RPi:
bash

mkdir -p ~/ros2_ws/src/robot_bringup/launch
mkdir -p ~/ros2_ws/src/robot_bringup/config

Then rebuild inside Docker:
bash

colcon build --symlink-install

Both packages should pass now. The dynamixel_driver already built successfully, so this is just fixing robot_bringup.

Some stuff I want you to change:

Thever_node]: Motor ID 1: ping failed — [TxRxResult] There is no status packet! [dynamixel_driver_node-2] [INFO] [1779569954.825341972] [dynamixel_driver_node]: Port /dev/ttyUSB0 opened at 57600 baud [dynamixel_driver_node-2] [ERROR] [1779569954.861355888] [dynamixel_driver_node]: Motor ID 1: ping failed — [TxRxResult] There is no status packet! [dynamixel_driver_node-2] [INFO] [1779569954.862243148] [d

This should instead output the motor IDs it detected, and IDs it did not detected.

  # Right-side motors spin in reverse direction for forward motion
    reverse_ids: [6, 10]

There is no such thing as reverse ids for forward motion. for forward, everything just goes forward.

I do not need a cmd timeout     cmd_timeout: 0.5
12:02 AM
View current dynamixel_driver_node.cpp

Let me apply all three changes:
Edited 5 files

Now update the cmdVelCallback — remove reverse logic, all motors get the same velocity:
Viewed 2 files, edited a file

Now remove the cmd_timeout watchdog from timerCallback and update shutdown/status to use active_ids_:
Remove timeout watchdog from timerCallback, keep just retry and status

Now update shutdown and publishStatus to use active_ids_:
Edited a file, viewed a file

The shutdown loop body got lost. Let me fix that:
Fix shutdown loop — add back the stop and torque disable commands

Now update publishStatus to use active_ids_:
Edited 2 files

Now clean up the member variables — remove reverse_ids_, cmd_timeout_, last_linear_, last_angular_, last_cmd_time_, and add active_ids_:
Edited 2 files

Remove unused last_cmd_time_ init and <set> include:
Edited 2 files

Now update the YAML config and the header comment:
Viewed a file, ran a command

Let me verify the final state of the cpp file compiles cleanly in my head:
Final review of the complete file

Clean. Let me fix the stale header comment too:
Fix header comment — remove reverse motor mention

Now copy the updated files to output:
Ran a command, read a file

Here are the two updated files. Summary of changes:

Initialization now scans and reports — instead of failing on the first missing motor, it pings all 4 IDs and prints something like:

Motors FOUND:   [1 6 10 ]
Motors MISSING: [8 ]
Initialized 3 / 4 motors

It then operates with only the motors that responded (active_ids_), so you can test with 1, 2, or 3 motors connected.

Removed reverse_ids — all motors get the same velocity value. No direction flipping.

Removed cmd_timeout — no auto-stop watchdog. Motors hold their last commanded velocity until you send a new command or kill the node (which triggers shutdown → stop all + torque off).

Replace these two files on the RPi and rebuild with colcon build --symlink-install. The YAML change won't need a rebuild since --symlink-install symlinks config files, but the .cpp change does.
Dynamixel driver node
CPP 
Motor params
YAML 
Want to be notified when Claude responds?


Claude is AI and can make mistakes. Please double-check responses.
Dynamixel driver node · CPP
/*
 * Dynamixel Driver Node
 * =====================
 * Subscribes to /cmd_vel (geometry_msgs/Twist)
 * Controls 4x Dynamixel XH540-W140-R motors via U2D2
 *
 * Architecture:
 *   joy_node (operator laptop)
 *     → /joy
 *   joy_mapper_node (RPi)
 *     → /cmd_vel
 *   dynamixel_driver_node (RPi, this node)
 *     → U2D2 serial → motors
 *
 * Motor IDs: 1, 6, 8, 10
 * All motors receive the same velocity command for forward/backward motion.
 *
 * Dynamixel XH540-W140-R control table (Protocol 2.0):
 *   Addr  Size  Name
 *   11    1     Operating Mode (1=velocity, 3=position, 4=ext.position)
 *   64    1     Torque Enable
 *   104   4     Goal Velocity (velocity mode)
 *   112   4     Goal Position (position mode)
 *   128   4     Present Velocity
 *   132   4     Present Position
 *   144   2     Present Input Voltage (unit: 0.1V)
 *   146   1     Present Temperature (unit: 1°C)
 */
 
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
 
#include <vector>
#include <string>
#include <chrono>
#include <cmath>
#include <algorithm>
 
// XH540-W140-R control table addresses
constexpr uint16_t ADDR_OPERATING_MODE   = 11;
constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
constexpr uint16_t ADDR_GOAL_VELOCITY    = 104;
constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;
constexpr uint16_t ADDR_PRESENT_POSITION = 132;
constexpr uint16_t ADDR_PRESENT_VOLTAGE  = 144;
constexpr uint16_t ADDR_PRESENT_TEMP     = 146;
 
// Operating modes
constexpr uint8_t MODE_VELOCITY = 1;
constexpr uint8_t MODE_POSITION = 3;
 
// Protocol
constexpr float PROTOCOL_VERSION = 2.0;
 
class DynamixelDriverNode : public rclcpp::Node
{
public:
    DynamixelDriverNode()
    : Node("dynamixel_driver_node"),
      port_handler_(nullptr),
      packet_handler_(nullptr),
      motors_initialized_(false)
    {
        // Declare parameters
        this->declare_parameter("port_name", "/dev/ttyUSB0");
        this->declare_parameter("baudrate", 57600);
        this->declare_parameter("motor_ids", std::vector<int64_t>{1, 6, 8, 10});
        this->declare_parameter("operating_mode", "velocity");
        this->declare_parameter("max_velocity", 100);
        this->declare_parameter("deadzone", 0.15);
        this->declare_parameter("loop_rate", 50.0);
 
        // Read parameters
        port_name_    = this->get_parameter("port_name").as_string();
        baudrate_     = this->get_parameter("baudrate").as_int();
        max_velocity_ = this->get_parameter("max_velocity").as_int();
        double loop_rate = this->get_parameter("loop_rate").as_double();
 
        auto id_vec = this->get_parameter("motor_ids").as_integer_array();
        for (auto id : id_vec) motor_ids_.push_back(static_cast<uint8_t>(id));
 
        // Initialize Dynamixel SDK
        if (!initMotors()) {
            RCLCPP_ERROR(this->get_logger(), "Motor init failed — node will retry every 3s");
        }
 
        // Subscriber: /cmd_vel
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&DynamixelDriverNode::cmdVelCallback, this, std::placeholders::_1));
 
        // Publisher: motor status (optional, for debugging)
        status_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/motor_status", 10);
 
        // Timer: safety watchdog + status publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / loop_rate)),
            std::bind(&DynamixelDriverNode::timerCallback, this));
 
        RCLCPP_INFO(this->get_logger(),
            "Dynamixel driver started — port:%s baud:%d motors:[%d,%d,%d,%d] max_vel:%d",
            port_name_.c_str(), baudrate_,
            motor_ids_[0], motor_ids_[1], motor_ids_[2], motor_ids_[3],
            max_velocity_);
    }
 
    ~DynamixelDriverNode()
    {
        shutdown();
    }
 
private:
    // ─── Initialization ────────────────────────────────────────────
    bool initMotors()
    {
        port_handler_ = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
        packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
 
        if (!port_handler_->openPort()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open port %s", port_name_.c_str());
            return false;
        }
 
        if (!port_handler_->setBaudRate(baudrate_)) {
            RCLCPP_ERROR(this->get_logger(), "Cannot set baudrate %d", baudrate_);
            return false;
        }
 
        RCLCPP_INFO(this->get_logger(), "Port %s opened at %d baud", port_name_.c_str(), baudrate_);
 
        // Scan all motor IDs — report which are found and which are missing
        std::vector<uint8_t> found_ids;
        std::vector<uint8_t> missing_ids;
 
        RCLCPP_INFO(this->get_logger(), "Scanning for motors...");
 
        for (uint8_t id : motor_ids_) {
            uint8_t dxl_error = 0;
            int result = packet_handler_->ping(port_handler_, id, &dxl_error);
            if (result != COMM_SUCCESS) {
                missing_ids.push_back(id);
                continue;
            }
            if (dxl_error != 0) {
                RCLCPP_WARN(this->get_logger(), "Motor ID %d: hardware error — %s",
                    id, packet_handler_->getRxPacketError(dxl_error));
            }
            found_ids.push_back(id);
        }
 
        // Report results
        std::string found_str, missing_str;
        for (auto id : found_ids)  found_str  += std::to_string(id) + " ";
        for (auto id : missing_ids) missing_str += std::to_string(id) + " ";
 
        if (!found_ids.empty())
            RCLCPP_INFO(this->get_logger(), "Motors FOUND:   [%s]", found_str.c_str());
        if (!missing_ids.empty())
            RCLCPP_WARN(this->get_logger(), "Motors MISSING: [%s]", missing_str.c_str());
 
        if (found_ids.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No motors detected — check wiring and power");
            return false;
        }
 
        // Initialize only the motors that responded
        active_ids_ = found_ids;
        for (uint8_t id : active_ids_) {
            // Disable torque before changing mode
            writeByteRegister(id, ADDR_TORQUE_ENABLE, 0);
 
            // Set velocity mode
            writeByteRegister(id, ADDR_OPERATING_MODE, MODE_VELOCITY);
 
            // Enable torque
            writeByteRegister(id, ADDR_TORQUE_ENABLE, 1);
 
            RCLCPP_INFO(this->get_logger(), "Motor ID %d: velocity mode, torque ON", id);
        }
 
        RCLCPP_INFO(this->get_logger(), "Initialized %zu / %zu motors",
            active_ids_.size(), motor_ids_.size());
 
        motors_initialized_ = true;
        return true;
    }
 
    void shutdown()
    {
        if (!motors_initialized_) return;
 
        // Stop all motors and disable torque
        for (uint8_t id : active_ids_) {
            writeDwordRegister(id, ADDR_GOAL_VELOCITY, 0);
            writeByteRegister(id, ADDR_TORQUE_ENABLE, 0);
        }
        port_handler_->closePort();
        motors_initialized_ = false;
        RCLCPP_INFO(this->get_logger(), "Motors stopped, torque OFF, port closed");
    }
 
    // ─── Callbacks ─────────────────────────────────────────────────
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!motors_initialized_) return;
 
        double speed = msg->linear.x;
 
        // Convert normalized speed (-1.0 to 1.0) to Dynamixel velocity units
        int32_t vel = static_cast<int32_t>(std::clamp(speed, -1.0, 1.0) * max_velocity_);
 
        // Send same velocity to all active motors
        for (uint8_t id : active_ids_) {
            writeDwordRegister(id, ADDR_GOAL_VELOCITY, vel);
        }
    }
 
    void timerCallback()
    {
        // Retry init if motors aren't connected yet
        if (!motors_initialized_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "Retrying motor initialization...");
            initMotors();
            return;
        }
 
        // Publish motor status every ~1 second (throttled)
        publishStatus();
    }
 
    void publishStatus()
    {
        // Throttle to ~2 Hz
        static int counter = 0;
        if (++counter % 25 != 0) return;
 
        auto status_msg = std_msgs::msg::Float32MultiArray();
        // Layout: [id1_vel, id1_temp, id1_voltage, id2_vel, id2_temp, ...]
        for (uint8_t id : active_ids_) {
            int32_t  vel  = readDwordRegister(id, ADDR_PRESENT_VELOCITY);
            uint16_t volt = readWordRegister(id, ADDR_PRESENT_VOLTAGE);
            uint8_t  temp = readByteRegister(id, ADDR_PRESENT_TEMP);
 
            status_msg.data.push_back(static_cast<float>(vel));
            status_msg.data.push_back(static_cast<float>(temp));
            status_msg.data.push_back(static_cast<float>(volt) / 10.0f);  // convert to volts
        }
        status_pub_->publish(status_msg);
    }
 
    // ─── Dynamixel register helpers ────────────────────────────────
    void writeByteRegister(uint8_t id, uint16_t addr, uint8_t value)
    {
        uint8_t dxl_error = 0;
        int result = packet_handler_->write1ByteTxRx(port_handler_, id, addr, value, &dxl_error);
        if (result != COMM_SUCCESS) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Write1Byte ID %d addr %d failed: %s", id, addr,
                packet_handler_->getTxRxResult(result));
        }
    }
 
    void writeDwordRegister(uint8_t id, uint16_t addr, int32_t value)
    {
        uint8_t dxl_error = 0;
        int result = packet_handler_->write4ByteTxRx(
            port_handler_, id, addr, static_cast<uint32_t>(value), &dxl_error);
        if (result != COMM_SUCCESS) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Write4Byte ID %d addr %d failed: %s", id, addr,
                packet_handler_->getTxRxResult(result));
        }
    }
 
    uint8_t readByteRegister(uint8_t id, uint16_t addr)
    {
        uint8_t value = 0, dxl_error = 0;
        packet_handler_->read1ByteTxRx(port_handler_, id, addr, &value, &dxl_error);
        return value;
    }
 
    uint16_t readWordRegister(uint8_t id, uint16_t addr)
    {
        uint16_t value = 0;
        uint8_t dxl_error = 0;
        packet_handler_->read2ByteTxRx(port_handler_, id, addr, &value, &dxl_error);
        return value;
    }
 
    int32_t readDwordRegister(uint8_t id, uint16_t addr)
    {
        uint32_t value = 0;
        uint8_t dxl_error = 0;
        packet_handler_->read4ByteTxRx(port_handler_, id, addr, &value, &dxl_error);
        return static_cast<int32_t>(value);
    }
 
    // ─── Members ───────────────────────────────────────────────────
    dynamixel::PortHandler*   port_handler_;
    dynamixel::PacketHandler* packet_handler_;
    bool motors_initialized_;
 
    std::vector<uint8_t> motor_ids_;
    std::vector<uint8_t> active_ids_;    // motors that actually responded to ping
    std::string port_name_;
    int baudrate_;
    int max_velocity_;
 
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamixelDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
 

