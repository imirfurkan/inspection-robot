/*
 * Dynamixel Driver Node (ROS2)
 * =============================
 * Subscribes to /cmd_vel (geometry_msgs/Twist)
 * Subscribes to /drive_mode (std_msgs/String)
 * Controls Dynamixel XH540-W140-R motors via U2D2.
 *
 * Motor layout (looking from the front):
 *   Front-Left: ID 10    Front-Right: ID 6
 *   Rear-Left:  ID 1     Rear-Right:  ID 8
 *
 * Five drive modes:
 *   "drive_all"          — all 4 motors velocity (default, normal driving)
 *   "drive_rear_assist"  — rear velocity + front current (compliant assist)
 *   "drive_front_nudge"  — rear hold (vel=0) + front current (fix buckling)
 *   "drive_front_only"   — rear hold (vel=0) + front velocity (front maneuver)
 *   "drive_rear_only"    — rear velocity + front hold (vel=0) (rear maneuver)
 *
 * Rear motors ALWAYS stay in velocity mode. Only front motors switch
 * between velocity and current mode. This means rear wheels never lose
 * torque during a mode switch — no rollback on slopes.
 *
 * Publishes /motor_status (Float32MultiArray) with telemetry.
 *
 * e-manual: https://emanual.robotis.com/docs/en/dxl/x/xw540-t140/
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp> // standard ROS2 message for velocity commands
                                       // linear.x = forward/backward, angular.z = yaw rotation
                                       // using Twist means any standard ROS2 tool can drive the robot
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp> // for drive mode
#include <vector>
#include <set>
#include <string>
#include <algorithm>
#include <cstdint>

#include <dynamixel_sdk/dynamixel_sdk.h>

// ── XH540-W140 control table ────────────────────────────────────
// The numbers here are the register addresses for the Dynamixel motors.
// You can find these in the Dynamixel manual for your specific model.
// They are used to read/write data to the motors, such as setting velocity or reading temperature.
constexpr uint16_t ADDR_OPERATING_MODE   = 11;
constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
constexpr uint16_t ADDR_GOAL_CURRENT     = 102;
constexpr uint16_t ADDR_GOAL_VELOCITY    = 104;
constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;
constexpr uint16_t ADDR_PRESENT_TEMP     = 146;
constexpr uint16_t ADDR_PRESENT_VOLTAGE  = 144;

constexpr uint8_t MODE_CURRENT  = 0;
constexpr uint8_t MODE_VELOCITY = 1;

constexpr int PROTOCOL_VERSION  = 2;

constexpr float CURRENT_UNIT_MA = 2.69f;

class DynamixelDriverNode : public rclcpp::Node
{
public:
    DynamixelDriverNode() : Node("dynamixel_driver_node")
    {
        // ── Declare parameters ──
        // YAML values override these defaults.
        this->declare_parameter("port_name", "/dev/ttyUSB0");
        this->declare_parameter("baudrate", 57600);
        this->declare_parameter("motor_ids", std::vector<int64_t>{1, 6, 8, 10});
        this->declare_parameter("reverse_ids", std::vector<int64_t>{});
        this->declare_parameter("front_ids", std::vector<int64_t>{10, 6});
        this->declare_parameter("rear_ids", std::vector<int64_t>{1, 8});
        this->declare_parameter("operating_mode", "velocity");
        this->declare_parameter("max_velocity", 100);
        this->declare_parameter("max_current_ma", 400.0);
        this->declare_parameter("deadzone", 0.15);
        this->declare_parameter("loop_rate", 50.0);

        // ── Read parameters ──
        port_name_ = this->get_parameter("port_name").as_string();
        baudrate_  = this->get_parameter("baudrate").as_int();

        // motor_ids_ is a vector (ordered list, you iterate in order), reverse_ids_ is a set
        // (unordered collection where .count(id) is fast lookup — O(log n) vs O(n) for vector).
        // We use set for reverse/front/rear IDs because we only ever ask "is this ID in the set?"
        // — never "what's the third reverse ID?"
        auto id_vec = this->get_parameter("motor_ids").as_integer_array();
        for (auto id : id_vec) motor_ids_.push_back(static_cast<uint8_t>(id));

        auto rev_vec = this->get_parameter("reverse_ids").as_integer_array();
        for (auto id : rev_vec) reverse_ids_.insert(static_cast<uint8_t>(id));

        auto front_vec = this->get_parameter("front_ids").as_integer_array();
        for (auto id : front_vec) front_ids_.insert(static_cast<uint8_t>(id));

        auto rear_vec = this->get_parameter("rear_ids").as_integer_array();
        for (auto id : rear_vec) rear_ids_.insert(static_cast<uint8_t>(id));

        max_velocity_ = this->get_parameter("max_velocity").as_int();
        max_current_ma_ = this->get_parameter("max_current_ma").as_double();
        max_current_units_ = static_cast<int16_t>(max_current_ma_ / CURRENT_UNIT_MA); // Dynamixel current is set in discrete units, so we convert from mA to units here.
        double loop_rate = this->get_parameter("loop_rate").as_double();

        // ── Initialize Dynamixel SDK ──
        port_handler_   = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
        packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if (!port_handler_->openPort()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open port %s", port_name_.c_str());
            return;
        }
        if (!port_handler_->setBaudRate(baudrate_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set baud rate %d", baudrate_);
            return;
        }

        RCLCPP_INFO(this->get_logger(),
            "Port %s opened at %d baud", port_name_.c_str(), baudrate_);

        // Start in drive_all mode
        drive_mode_ = DriveMode::DRIVE_ALL;
        initMotors();

        // ── ROS2 pub/sub ──
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&DynamixelDriverNode::cmdVelCallback, this, std::placeholders::_1));

        mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/drive_mode", 10,
            std::bind(&DynamixelDriverNode::modeCallback, this, std::placeholders::_1));

        status_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/motor_status", 10);

        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / loop_rate)),
            std::bind(&DynamixelDriverNode::publishStatus, this));

        RCLCPP_INFO(this->get_logger(),
            "Dynamixel driver ready. Max velocity: %d units, Max current: %.0f mA (%d units)",
            max_velocity_, max_current_ma_, max_current_units_);
    }

    // The constructor runs when the object is created, the destructor runs when it's destroyed.
    // In ROS2, when you press Ctrl+C, rclcpp::shutdown() triggers the node's destruction, which calls the destructor.
    // That's where you stop motors and close the port — guaranteed cleanup even if the shutdown is unexpected.
    // Without it, killing the node would leave motors spinning with torque enabled.
    ~DynamixelDriverNode() // destructor
    {
        // Stop all motors and disable torque
        for (uint8_t id : active_ids_) {
            writeDwordRegister(id, ADDR_GOAL_VELOCITY, 0);
            writeWordRegister(id, ADDR_GOAL_CURRENT, 0);
            writeByteRegister(id, ADDR_TORQUE_ENABLE, 0);
        }

        if (port_handler_) {
            port_handler_->closePort();
        }
        RCLCPP_INFO(this->get_logger(), "Motors stopped, port closed.");
    }

private:
    enum class DriveMode {
        DRIVE_ALL,          // all velocity
        DRIVE_REAR_ASSIST,  // rear velocity, front current (compliant)
        DRIVE_FRONT_NUDGE,  // rear hold (vel=0), front current (nudge)
        DRIVE_FRONT_ONLY,   // rear hold (vel=0), front velocity
        DRIVE_REAR_ONLY     // rear velocity, front hold (vel=0)
    };

    // ── Helper: do front motors need current mode in this drive mode? ──
    // Only DRIVE_REAR_ASSIST and DRIVE_FRONT_NUDGE use current mode for front.
    // All other modes keep front in velocity mode (either joystick-driven or held at 0).
    bool frontNeedsCurrent(DriveMode mode) const
    {
        return mode == DriveMode::DRIVE_REAR_ASSIST || mode == DriveMode::DRIVE_FRONT_NUDGE;
    }

    // ── Helper: mode name string for logging ──
    const char* modeName(DriveMode mode) const
    {
        switch (mode) {
            case DriveMode::DRIVE_ALL:         return "DRIVE_ALL";
            case DriveMode::DRIVE_REAR_ASSIST: return "DRIVE_REAR_ASSIST";
            case DriveMode::DRIVE_FRONT_NUDGE: return "DRIVE_FRONT_NUDGE";
            case DriveMode::DRIVE_FRONT_ONLY:  return "DRIVE_FRONT_ONLY";
            case DriveMode::DRIVE_REAR_ONLY:   return "DRIVE_REAR_ONLY";
            default:                           return "UNKNOWN";
        }
    }

    // ── Motor initialization ──────────────────────────────────────
    bool initMotors()
    {
        active_ids_.clear();
        std::string found_str, missing_str;

        for (uint8_t id : motor_ids_) {
            uint8_t dxl_error = 0;
            int result = packet_handler_->ping(port_handler_, id, &dxl_error);

            if (result != COMM_SUCCESS) {
                missing_str += std::to_string(id) + " ";
                continue;
            }

            if (dxl_error != 0) {
                RCLCPP_WARN(this->get_logger(), "Motor ID %d: hardware error — %s",
                    id, packet_handler_->getRxPacketError(dxl_error));
            }

            active_ids_.push_back(id);
            found_str += std::to_string(id) + " ";
        }

        RCLCPP_INFO(this->get_logger(), "Motors FOUND:   [%s]", found_str.c_str());
        if (!missing_str.empty()) {
            RCLCPP_WARN(this->get_logger(), "Motors MISSING: [%s]", missing_str.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "Initialized %zu / %zu motors",
            active_ids_.size(), motor_ids_.size());

        motors_initialized_ = !active_ids_.empty();

        if (motors_initialized_) {
            // Set all motors to velocity mode on startup
            for (uint8_t id : active_ids_) {
                writeByteRegister(id, ADDR_TORQUE_ENABLE, 0);
                writeByteRegister(id, ADDR_OPERATING_MODE, MODE_VELOCITY);
                writeByteRegister(id, ADDR_TORQUE_ENABLE, 1);
                RCLCPP_INFO(this->get_logger(), "Motor ID %d: VELOCITY mode, torque ON", id);
            }
        }

        return motors_initialized_;
    }

    // ── Switch front motors between velocity and current mode ─────
    // Only touches front motors. Rear motors keep running undisturbed.
    void switchFrontMode(uint8_t new_mode)
    {
        for (uint8_t id : active_ids_) {
            if (!front_ids_.count(id)) continue;  // skip rear motors

            // Zero out front motor commands before switching
            writeWordRegister(id, ADDR_GOAL_CURRENT, 0);
            writeDwordRegister(id, ADDR_GOAL_VELOCITY, 0);

            // Torque off → change mode → torque on
            writeByteRegister(id, ADDR_TORQUE_ENABLE, 0);
            writeByteRegister(id, ADDR_OPERATING_MODE, new_mode);
            writeByteRegister(id, ADDR_TORQUE_ENABLE, 1);

            const char* mode_name = (new_mode == MODE_CURRENT) ? "CURRENT" : "VELOCITY";
            RCLCPP_INFO(this->get_logger(),
                "Motor ID %d: switched to %s mode", id, mode_name);
        }
    }

    // ── Callbacks ─────────────────────────────────────────────────
    void modeCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (!motors_initialized_) return;

        DriveMode new_mode;
        if (msg->data == "drive_all") {
            new_mode = DriveMode::DRIVE_ALL;
        } else if (msg->data == "drive_rear_assist") {
            new_mode = DriveMode::DRIVE_REAR_ASSIST;
        } else if (msg->data == "drive_front_nudge") {
            new_mode = DriveMode::DRIVE_FRONT_NUDGE;
        } else if (msg->data == "drive_front_only") {
            new_mode = DriveMode::DRIVE_FRONT_ONLY;
        } else if (msg->data == "drive_rear_only") {
            new_mode = DriveMode::DRIVE_REAR_ONLY;
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown drive mode: %s", msg->data.c_str());
            return;
        }

        if (new_mode == drive_mode_) return;  // already in this mode

        // Determine if front motors need an operating mode change.
        // Front motors use current mode for DRIVE_REAR_ASSIST and DRIVE_FRONT_NUDGE,
        // velocity mode for everything else.
        bool old_front_current = frontNeedsCurrent(drive_mode_);
        bool new_front_current = frontNeedsCurrent(new_mode);

        if (old_front_current != new_front_current) {
            // Front motor operating mode actually changes (~24ms, rear undisturbed)
            uint8_t front_mode = new_front_current ? MODE_CURRENT : MODE_VELOCITY;
            switchFrontMode(front_mode);
        }

        // If entering a mode where rear holds, set rear velocity to 0 immediately.
        // Rear stays in velocity mode with goal=0 — actively resists rotation (powered brake).
        if (new_mode == DriveMode::DRIVE_FRONT_NUDGE || new_mode == DriveMode::DRIVE_FRONT_ONLY) {
            for (uint8_t id : active_ids_) {
                if (rear_ids_.count(id)) {
                    writeDwordRegister(id, ADDR_GOAL_VELOCITY, 0);
                }
            }
        }

        // If entering a mode where front holds (vel=0), set front velocity to 0 immediately.
        // This only applies when front is in velocity mode (DRIVE_REAR_ONLY).
        if (new_mode == DriveMode::DRIVE_REAR_ONLY) {
            for (uint8_t id : active_ids_) {
                if (front_ids_.count(id)) {
                    writeDwordRegister(id, ADDR_GOAL_VELOCITY, 0);
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Drive mode: %s → %s",
            modeName(drive_mode_), modeName(new_mode));

        drive_mode_ = new_mode;
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!motors_initialized_) return;

        double speed = msg->linear.x;
        double clamped = std::clamp(speed, -1.0, 1.0);

        // Pre-compute commands
        int32_t vel = static_cast<int32_t>(clamped * max_velocity_);
        int16_t cur = static_cast<int16_t>(clamped * max_current_units_);

        for (uint8_t id : active_ids_) {
            bool is_front = front_ids_.count(id) > 0;
            bool is_rear  = rear_ids_.count(id) > 0;
            int sign = reverse_ids_.count(id) ? -1 : 1;

            switch (drive_mode_) {
                case DriveMode::DRIVE_ALL:
                    // All motors: velocity from joystick
                    writeDwordRegister(id, ADDR_GOAL_VELOCITY, vel * sign);
                    break;

                case DriveMode::DRIVE_REAR_ASSIST:
                    if (is_front) {
                        // Front: current mode (compliant assist, proportional to joystick)
                        writeWordRegister(id, ADDR_GOAL_CURRENT,
                            static_cast<int16_t>(cur * sign));
                    } else {
                        // Rear: velocity from joystick (primary drive)
                        writeDwordRegister(id, ADDR_GOAL_VELOCITY, vel * sign);
                    }
                    break;

                case DriveMode::DRIVE_FRONT_NUDGE:
                    if (is_front) {
                        // Front: current mode (joystick-controlled nudge)
                        writeWordRegister(id, ADDR_GOAL_CURRENT,
                            static_cast<int16_t>(cur * sign));
                    } else if (is_rear) {
                        // Rear: hold position (velocity = 0, active brake)
                        writeDwordRegister(id, ADDR_GOAL_VELOCITY, 0);
                    }
                    break;

                case DriveMode::DRIVE_FRONT_ONLY:
                    if (is_front) {
                        // Front: velocity from joystick
                        writeDwordRegister(id, ADDR_GOAL_VELOCITY, vel * sign);
                    } else if (is_rear) {
                        // Rear: hold position (velocity = 0, active brake)
                        writeDwordRegister(id, ADDR_GOAL_VELOCITY, 0);
                    }
                    break;

                case DriveMode::DRIVE_REAR_ONLY:
                    if (is_front) {
                        // Front: hold position (velocity = 0, active brake)
                        writeDwordRegister(id, ADDR_GOAL_VELOCITY, 0);
                    } else if (is_rear) {
                        // Rear: velocity from joystick
                        writeDwordRegister(id, ADDR_GOAL_VELOCITY, vel * sign);
                    }
                    break;
            }
        }
    }

    void publishStatus()
    {
        if (!motors_initialized_) return;

        auto msg = std_msgs::msg::Float32MultiArray();
        for (uint8_t id : active_ids_) {
            uint32_t raw_vel = 0;
            packet_handler_->read4ByteTxRx(port_handler_, id,
                ADDR_PRESENT_VELOCITY, &raw_vel);
            int32_t vel = static_cast<int32_t>(raw_vel);

            uint8_t temp = 0;
            packet_handler_->read1ByteTxRx(port_handler_, id,
                ADDR_PRESENT_TEMP, &temp);

            uint16_t raw_v = 0;
            packet_handler_->read2ByteTxRx(port_handler_, id,
                ADDR_PRESENT_VOLTAGE, &raw_v);

            msg.data.push_back(static_cast<float>(vel));
            msg.data.push_back(static_cast<float>(temp));
            msg.data.push_back(static_cast<float>(raw_v) / 10.0f);
        }
        status_pub_->publish(msg);
    }

    // ── Dynamixel register helpers ────────────────────────────────
    void writeByteRegister(uint8_t id, uint16_t addr, uint8_t value)
    {
        uint8_t dxl_error = 0;
        packet_handler_->write1ByteTxRx(port_handler_, id, addr, value, &dxl_error);
    }

    void writeWordRegister(uint8_t id, uint16_t addr, int16_t value)
    {
        uint8_t dxl_error = 0;
        packet_handler_->write2ByteTxRx(port_handler_, id, addr,
            static_cast<uint16_t>(value), &dxl_error);
    }

    void writeDwordRegister(uint8_t id, uint16_t addr, int32_t value)
    {
        uint8_t dxl_error = 0;
        packet_handler_->write4ByteTxRx(port_handler_, id, addr,
            static_cast<uint32_t>(value), &dxl_error);
    }

    // ── Member variables ──────────────────────────────────────────
    std::vector<uint8_t> motor_ids_;
    std::vector<uint8_t> active_ids_;
    std::set<uint8_t>    reverse_ids_;
    std::set<uint8_t>    front_ids_;
    std::set<uint8_t>    rear_ids_;
    std::string port_name_;
    int baudrate_;
    int max_velocity_;
    double max_current_ma_;
    int16_t max_current_units_;

    DriveMode drive_mode_ = DriveMode::DRIVE_ALL;
    bool motors_initialized_ = false;

    dynamixel::PortHandler*   port_handler_   = nullptr;
    dynamixel::PacketHandler* packet_handler_ = nullptr;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamixelDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}