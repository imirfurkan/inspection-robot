/*
 * Dynamixel Driver Node (ROS2) — Refactored
 * ==========================================
 * Generic motor driver that reads mode descriptors from drive_modes.h/cpp.
 * The driver itself has NO mode-specific logic — it just executes whatever
 * the active DriveModeDef says to do for each motor.
 *
 * To add a new drive mode, edit drive_modes.cpp only.
 *
 * Motor layout (looking from the front):
 *   Front-Left: ID 10    Front-Right: ID 6
 *   Rear-Left:  ID 1     Rear-Right:  ID 8
 *
 * Subscribes to /cmd_vel       (geometry_msgs/Twist)
 * Subscribes to /drive_mode    (std_msgs/String)
 * Subscribes to /steering_position (std_msgs/Float32) — normalized: -1.0=full left, 0.0=center, +1.0=full right
 * Publishes  /motor_status      (Float32MultiArray) with telemetry.
 *
 * Ackermann K interpolation (drive_all mode only):
 *   The inner wheels on a turn travel a shorter arc than the outer wheels,
 *   so their velocity is scaled down proportionally. K values are linearly
 *   interpolated between center (all 1.0) and the full-lock endpoints:
 *
 *   Full left  (position -1.0): FL=0.85, FR=1.00, RL=0.79, RR=0.94
 *   Center     (position  0.0): FL=1.00, FR=1.00, RL=1.00, RR=1.00
 *   Full right (position +1.0): FL=1.00, FR=0.85, RL=0.94, RR=0.79
 *
 * e-manual: https://emanual.robotis.com/docs/en/dxl/x/xw540-t140/
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <algorithm>
#include <cstdint>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include "drive_modes.h"

// ── XH540-W140 control table ────────────────────────────────────
constexpr uint16_t ADDR_OPERATING_MODE   = 11;
constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
constexpr uint16_t ADDR_GOAL_CURRENT     = 102;
constexpr uint16_t ADDR_GOAL_VELOCITY    = 104;
constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;
constexpr uint16_t ADDR_PRESENT_TEMP     = 146;
constexpr uint16_t ADDR_PRESENT_VOLTAGE  = 144;

constexpr uint8_t DXL_MODE_CURRENT  = 0;
constexpr uint8_t DXL_MODE_VELOCITY = 1;

constexpr int PROTOCOL_VERSION = 2;
constexpr float CURRENT_UNIT_MA = 2.69f;

// ── Ackermann K values at full-lock endpoints ────────────────────
// These describe the velocity ratio of each wheel relative to the
// fastest (outer) wheel at maximum steering lock.
//
// At full LEFT lock:
//   FL (inner front) = 0.85,  FR (outer front) = 1.00
//   RL (inner rear)  = 0.79,  RR (outer rear)  = 0.94
//
// At full RIGHT lock (mirror of left):
//   FL (outer front) = 1.00,  FR (inner front) = 0.85
//   RL (outer rear)  = 0.94,  RR (inner rear)  = 0.79
struct AckermannKEndpoint {
    float fl, fr, rl, rr;
};

static constexpr AckermannKEndpoint K_CENTER    = {1.00f, 1.00f, 1.00f, 1.00f};
static constexpr AckermannKEndpoint K_FULL_LEFT = {0.85f, 1.00f, 0.79f, 0.94f};
static constexpr AckermannKEndpoint K_FULL_RIGHT= {1.00f, 0.85f, 0.94f, 0.79f};


class DynamixelDriverNode : public rclcpp::Node
{
public:
    DynamixelDriverNode() : Node("dynamixel_driver_node")
    {
        // ── Declare parameters ──
        this->declare_parameter("port_name", "/dev/ttyUSB0");
        this->declare_parameter("baudrate", 57600);
        this->declare_parameter("motor_ids", std::vector<int64_t>{1, 6, 8, 10});
        this->declare_parameter("reverse_ids", std::vector<int64_t>{});
        this->declare_parameter("front_left_id", 10);
        this->declare_parameter("front_right_id", 6);
        this->declare_parameter("rear_left_id", 1);
        this->declare_parameter("rear_right_id", 8);
        this->declare_parameter("max_velocity", 100);
        this->declare_parameter("max_current_ma", 400.0);
        this->declare_parameter("deadzone", 0.15);
        this->declare_parameter("loop_rate", 50.0);
        this->declare_parameter("default_mode", std::string("drive_all"));

        // ── Read parameters ──
        port_name_ = this->get_parameter("port_name").as_string();
        baudrate_  = this->get_parameter("baudrate").as_int();

        auto id_vec = this->get_parameter("motor_ids").as_integer_array();
        for (auto id : id_vec) motor_ids_.push_back(static_cast<uint8_t>(id));

        auto rev_vec = this->get_parameter("reverse_ids").as_integer_array();
        for (auto id : rev_vec) reverse_ids_.insert(static_cast<uint8_t>(id));

        layout_.front_left  = static_cast<uint8_t>(this->get_parameter("front_left_id").as_int());
        layout_.front_right = static_cast<uint8_t>(this->get_parameter("front_right_id").as_int());
        layout_.rear_left   = static_cast<uint8_t>(this->get_parameter("rear_left_id").as_int());
        layout_.rear_right  = static_cast<uint8_t>(this->get_parameter("rear_right_id").as_int());

        max_velocity_      = this->get_parameter("max_velocity").as_int();
        max_current_ma_    = this->get_parameter("max_current_ma").as_double();
        max_current_units_ = static_cast<int16_t>(max_current_ma_ / CURRENT_UNIT_MA);
        double loop_rate   = this->get_parameter("loop_rate").as_double();
        std::string default_mode = this->get_parameter("default_mode").as_string();

        // ── Build mode table ──
        mode_table_ = buildDefaultModes(layout_);

        // Log available modes
        std::string mode_list;
        for (const auto& [name, _] : mode_table_) mode_list += name + " ";
        RCLCPP_INFO(this->get_logger(), "Available modes: [%s]", mode_list.c_str());

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

        RCLCPP_INFO(this->get_logger(), "Port %s opened at %d baud", port_name_.c_str(), baudrate_);

        initMotors();

        // Set initial mode
        if (mode_table_.count(default_mode)) {
            active_mode_ = &mode_table_[default_mode];
            for (uint8_t id : active_ids_) current_op_mode_[id] = DXL_MODE_VELOCITY;
            applyOperatingModeChanges(mode_table_[default_mode], true);
            RCLCPP_INFO(this->get_logger(), "Starting in mode: %s", default_mode.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "Default mode '%s' not found in mode table!", default_mode.c_str());
            return;
        }

        // ── ROS2 pub/sub ──
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&DynamixelDriverNode::cmdVelCallback, this, std::placeholders::_1));

        mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/drive_mode", 10,
            std::bind(&DynamixelDriverNode::modeCallback, this, std::placeholders::_1));

        // Steering position — normalized [-1,1], used for Ackermann K interpolation in drive_all
        steering_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/steering_position", 10,
            std::bind(&DynamixelDriverNode::steeringCallback, this, std::placeholders::_1));

        position_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_position", 10,
            std::bind(&DynamixelDriverNode::positionCallback, this, std::placeholders::_1));

        status_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/motor_status", 10);

        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / loop_rate)),
            std::bind(&DynamixelDriverNode::publishStatus, this));

        RCLCPP_INFO(this->get_logger(),
            "Dynamixel driver ready. Max velocity: %d units, Max current: %.0f mA (%d units)",
            max_velocity_, max_current_ma_, max_current_units_);
        RCLCPP_INFO(this->get_logger(),
            "Ackermann interpolation active for drive_all — subscribed to /steering_position");
    }

    ~DynamixelDriverNode()
    {
        for (uint8_t id : active_ids_) {
            writeDwordRegister(id, ADDR_GOAL_VELOCITY, 0);
            writeWordRegister(id, ADDR_GOAL_CURRENT, 0);
            writeByteRegister(id, ADDR_TORQUE_ENABLE, 0);
        }
        if (port_handler_) port_handler_->closePort();
        RCLCPP_INFO(this->get_logger(), "Motors stopped, port closed.");
    }

private:
    // ── Ackermann K interpolation ─────────────────────────────────
    // pos: -1.0 = full left, 0.0 = center, +1.0 = full right
    // t = abs(pos), linearly interpolated from center (t=0) to endpoint (t=1).
    struct PerMotorK {
        float fl, fr, rl, rr;
    };

    PerMotorK computeAckermannK(float pos) const
    {
        pos = std::clamp(pos, -1.0f, 1.0f);
        float t = std::abs(pos);
        PerMotorK k;

        if (pos <= 0.0f) {
            // Turning left
            k.fl = K_CENTER.fl + t * (K_FULL_LEFT.fl - K_CENTER.fl);
            k.fr = K_CENTER.fr + t * (K_FULL_LEFT.fr - K_CENTER.fr);
            k.rl = K_CENTER.rl + t * (K_FULL_LEFT.rl - K_CENTER.rl);
            k.rr = K_CENTER.rr + t * (K_FULL_LEFT.rr - K_CENTER.rr);
        } else {
            // Turning right
            k.fl = K_CENTER.fl + t * (K_FULL_RIGHT.fl - K_CENTER.fl);
            k.fr = K_CENTER.fr + t * (K_FULL_RIGHT.fr - K_CENTER.fr);
            k.rl = K_CENTER.rl + t * (K_FULL_RIGHT.rl - K_CENTER.rl);
            k.rr = K_CENTER.rr + t * (K_FULL_RIGHT.rr - K_CENTER.rr);
        }

        return k;
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
            // All motors start in velocity mode
            for (uint8_t id : active_ids_) {
                writeByteRegister(id, ADDR_TORQUE_ENABLE, 0);
                writeByteRegister(id, ADDR_OPERATING_MODE, DXL_MODE_VELOCITY);
                writeByteRegister(id, ADDR_TORQUE_ENABLE, 1);
                RCLCPP_INFO(this->get_logger(), "Motor ID %d: VELOCITY mode, torque ON", id);
            }
        }
        return motors_initialized_;
    }

    // ── Generic operating mode switch ────────────────────────────
    void switchMotorOperatingMode(uint8_t id, uint8_t target_mode)
    {
        if (current_op_mode_.count(id) && current_op_mode_[id] == target_mode) return;

        // Zero commands before switching
        writeWordRegister(id, ADDR_GOAL_CURRENT, 0);
        writeDwordRegister(id, ADDR_GOAL_VELOCITY, 0);

        // Torque off → change → torque on
        writeByteRegister(id, ADDR_TORQUE_ENABLE, 0);
        writeByteRegister(id, ADDR_OPERATING_MODE, target_mode);
        writeByteRegister(id, ADDR_TORQUE_ENABLE, 1);

        current_op_mode_[id] = target_mode;

        const char* mode_name = (target_mode == DXL_MODE_CURRENT) ? "CURRENT" : "VELOCITY";
        RCLCPP_INFO(this->get_logger(), "Motor ID %d: switched to %s mode", id, mode_name);
    }

    // ── Apply operating modes for one specific profile ──────────
    // Sets each motor to the Dynamixel operating mode that its
    // ControlType requires:
    //   VELOCITY → velocity mode
    //   CURRENT  → current mode
    //   HOLD     → velocity mode (goal_velocity = 0 for active brake)
    //
    // Called on mode switch (for the current direction's profile)
    // and on zero-crossing (when direction flips to the other profile).
    void applyProfileOperatingModes(const std::map<uint8_t, MotorCommand>& profile)
    {
        for (uint8_t id : active_ids_) {
            MotorCommand cmd = getCommand(profile, id);
            uint8_t target = requiredOperatingMode(cmd.type);
            switchMotorOperatingMode(id, target);

            // If entering HOLD, zero velocity immediately
            if (cmd.type == ControlType::HOLD) {
                writeDwordRegister(id, ADDR_GOAL_VELOCITY, 0);
            }
        }
    }

    // ── Apply operating mode changes for a new drive mode ─────────
    // Uses the current direction to pick which profile to apply.
    void applyOperatingModeChanges(const DriveModeDef& mode, bool force = false)
    {
        (void)force;  // switchMotorOperatingMode already skips no-ops
        const auto& profile = going_forward_
            ? mode.forward_profile
            : mode.reverse_profile;
        applyProfileOperatingModes(profile);
    }

    // ── Steering position callback ────────────────────────────────
    void steeringCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        current_steering_position_ = msg->data;
    }

    // ── Mode switch callback ──────────────────────────────────────
    void modeCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (!motors_initialized_) return;

        auto it = mode_table_.find(msg->data);
        if (it == mode_table_.end()) {
            RCLCPP_WARN(this->get_logger(), "Unknown drive mode: '%s'", msg->data.c_str());

            // List available modes to help the user
            std::string avail;
            for (const auto& [name, _] : mode_table_) avail += name + " ";
            RCLCPP_WARN(this->get_logger(), "Available modes: [%s]", avail.c_str());
            return;
        }

        if (active_mode_ && active_mode_->name == msg->data) {
            return;  // already in this mode
        }

        const DriveModeDef& new_mode = it->second;
        RCLCPP_INFO(this->get_logger(), "Drive mode: %s → %s",
            active_mode_ ? active_mode_->name.c_str() : "NONE",
            new_mode.name.c_str());

        // Switch operating modes for motors that need it
        applyOperatingModeChanges(new_mode);

        active_mode_ = &new_mode;
    }

    void positionCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        current_robot_position_ = msg->data;
    }
    // ── Command callback ──────────────────────────────────────────
    // Generic — no mode-specific logic. Handles direction changes by
    // switching motor operating modes at the zero crossing so HOLD
    // always uses velocity mode with goal = 0.
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!motors_initialized_ || !active_mode_) return;

        double speed   = msg->linear.x;
        double clamped = std::clamp(speed, -1.0, 1.0);
        bool now_forward = (clamped <= 0.0); // TODO important
        // RCLCPP_INFO(this->get_logger(), "clamped=%.3f, now_forward=%d, going_forward_=%d", clamped, now_forward, going_forward_);

        // ── Zero-crossing detection ──
        // When direction flips, some motors may need their Dynamixel
        // operating mode switched (e.g. a motor that was CURRENT going
        // forward but becomes HOLD going reverse needs velocity mode).
        // This costs ~24ms per motor that switches, but only happens
        // at the zero crossing — not every tick.
        if (now_forward != going_forward_) {
            going_forward_ = now_forward;
            const auto& new_profile = now_forward
                ? active_mode_->forward_profile
                : active_mode_->reverse_profile;
            applyProfileOperatingModes(new_profile);
        }

        // Pick the active profile
        const auto& profile = now_forward
            ? active_mode_->forward_profile
            : active_mode_->reverse_profile;

        // Compute per-motor K for drive_all (Ackermann interpolation).
        // For all other modes K comes directly from the profile struct.
        const bool is_drive_all = (active_mode_->name == "drive_all");
        // ── Buckling compensation (horizontal_buckling state only) ──
        // Reduces drive wheel speeds to prevent over-compression during buckling.
        // Forward: rear wheels at 0.7x  |  Reverse: front wheels at 0.7x
        const bool apply_buckling_comp = (is_drive_all
            && current_robot_position_ == "horizontal_buckling"
            && std::abs(clamped) > 0.0
            && !(std::abs(current_steering_position_) > 0.1f && now_forward)); // && (std::abs(current_steering_position_) < 0.05f

        PerMotorK ak{};
        if (is_drive_all) {
            ak = computeAckermannK(current_steering_position_);

            // Print current K proportions when actively steering (position non-zero)
            if (std::abs(current_steering_position_) > 0.01f) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                    "[drive_all] steering=%.2f  FL=%.3f  FR=%.3f  RL=%.3f  RR=%.3f",
                    current_steering_position_, ak.fl, ak.fr, ak.rl, ak.rr);
            }
        }

        for (uint8_t id : active_ids_) {
            MotorCommand cmd = getCommand(profile, id);
            int sign = reverse_ids_.count(id) ? -1 : 1;

            // Override K with Ackermann value for drive_all VELOCITY motors
            float effective_k = cmd.k;
            if (is_drive_all && cmd.type == ControlType::VELOCITY) {
                if      (id == layout_.front_left)  effective_k = ak.fl;
                else if (id == layout_.front_right) effective_k = ak.fr;
                else if (id == layout_.rear_left)   effective_k = ak.rl;
                else if (id == layout_.rear_right)  effective_k = ak.rr;

                // ── Dynamic inner-wheel current mode at steering threshold ──
                // When steering exceeds threshold, override the inner front wheel
                // to CURRENT mode (compliant) — mirrors steer_trial_left/right behavior.
                // Threshold sourced from parameter (default 0.5).
                const float steer_threshold = 0.2f; // TODO: make this a parameter
                if (current_steering_position_ < -steer_threshold && now_forward && id == layout_.front_left)
                {
                    // Turning left: FL is inner front
                    cmd.type = ControlType::CURRENT;
                    effective_k = ak.fl;
                } else if (current_steering_position_ > steer_threshold && now_forward && id == layout_.front_right)
                {
                    // Turning right: FR is inner front
                    cmd.type = ControlType::CURRENT;
                    effective_k = ak.fr;
                }
            }

            // ── Ensure Dynamixel operating mode matches what cmd.type needs ──
            // This is a no-op if the motor is already in the correct mode.
            if (is_drive_all) {
                switchMotorOperatingMode(id, requiredOperatingMode(cmd.type));
            }

            // Apply buckling compensation multiplier if active
            if (apply_buckling_comp) {
                bool is_rear  = (id == layout_.rear_left  || id == layout_.rear_right);
                bool is_front = (id == layout_.front_left || id == layout_.front_right);
                if (now_forward && is_rear)   effective_k *= 0.8f;
                if (!now_forward && is_front) effective_k *= 0.8f;
            }

            switch (cmd.type) {
                case ControlType::VELOCITY: {
                    int flip = cmd.reverse ? -1 : 1;
                    int32_t vel = static_cast<int32_t>(clamped * effective_k * max_velocity_);
                    writeDwordRegister(id, ADDR_GOAL_VELOCITY, vel * sign * flip);
                    break;
                }
                case ControlType::CURRENT: {
                    int flip = cmd.reverse ? -1 : 1;
                    int16_t cur = static_cast<int16_t>(clamped * effective_k * max_current_units_);
                    writeWordRegister(id, ADDR_GOAL_CURRENT, static_cast<int16_t>(cur * sign * flip));
                    break;
                }
                case ControlType::HOLD: {
                    writeDwordRegister(id, ADDR_GOAL_VELOCITY, 0);
                    break;
                }
            }
        }
    }

    // ── Status publisher ──────────────────────────────────────────
    void publishStatus()
    {
        if (!motors_initialized_) return;

        auto msg = std_msgs::msg::Float32MultiArray();
        for (uint8_t id : active_ids_) {
            uint32_t raw_vel = 0;
            packet_handler_->read4ByteTxRx(port_handler_, id, ADDR_PRESENT_VELOCITY, &raw_vel);
            int32_t vel = static_cast<int32_t>(raw_vel);

            uint8_t temp = 0;
            packet_handler_->read1ByteTxRx(port_handler_, id, ADDR_PRESENT_TEMP, &temp);

            uint16_t raw_v = 0;
            packet_handler_->read2ByteTxRx(port_handler_, id, ADDR_PRESENT_VOLTAGE, &raw_v);

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
    MotorLayout          layout_;

    std::string port_name_;
    int         baudrate_;
    int         max_velocity_;
    double      max_current_ma_;
    int16_t     max_current_units_;
    // Robot position state — updated by /robot_position subscription
    std::string current_robot_position_ = "unknown";

    // Normalized steering position in [-1, 1] — updated by /steering_position subscription
    float current_steering_position_ = 0.0f;

    // Mode system
    std::map<std::string, DriveModeDef> mode_table_;
    const DriveModeDef* active_mode_ = nullptr;
    std::map<uint8_t, uint8_t> current_op_mode_;
    bool going_forward_ = true;

    bool motors_initialized_ = false;

    dynamixel::PortHandler*   port_handler_   = nullptr;
    dynamixel::PacketHandler* packet_handler_ = nullptr;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr position_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamixelDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}