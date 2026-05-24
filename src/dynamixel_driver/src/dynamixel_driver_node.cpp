/*
 * Dynamixel Driver Node (ROS2)
 * =============================
 * Subscribes to /cmd_vel (geometry_msgs/Twist)
 * Controls Dynamixel XH540-W140-R motors via U2D2.
 *
 * Motor IDs: 1, 6, 8, 10
 * Motor layout (looking from the front):
 *   Front-Left: ID 10    Front-Right: ID 6
 *   Rear-Left:  ID 1     Rear-Right:  ID 8
 *
 * Motors listed in reverse_ids have their velocity negated so that
 * positive cmd_vel.linear.x drives all wheels forward regardless of
 * physical mounting direction.
 *
 * Publishes /motor_status (Float32MultiArray) with telemetry.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include <set>
#include <string>
#include <algorithm>
#include <cstdint>

#include <dynamixel_sdk/dynamixel_sdk.h>

// ── XH540-W140 control table ────────────────────────────────────
constexpr uint16_t ADDR_OPERATING_MODE   = 11;
constexpr uint16_t ADDR_TORQUE_ENABLE    = 64;
constexpr uint16_t ADDR_GOAL_VELOCITY    = 104;
constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;
constexpr uint16_t ADDR_PRESENT_TEMP     = 146;
constexpr uint16_t ADDR_PRESENT_VOLTAGE  = 144;

constexpr uint8_t MODE_VELOCITY = 1;
constexpr int PROTOCOL_VERSION  = 2;


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
        this->declare_parameter("operating_mode", "velocity");
        this->declare_parameter("max_velocity", 100);
        this->declare_parameter("deadzone", 0.15);
        this->declare_parameter("loop_rate", 50.0);

        // ── Read parameters ──
        port_name_ = this->get_parameter("port_name").as_string();
        baudrate_  = this->get_parameter("baudrate").as_int();

        auto id_vec = this->get_parameter("motor_ids").as_integer_array();
        for (auto id : id_vec) motor_ids_.push_back(static_cast<uint8_t>(id));

        auto rev_vec = this->get_parameter("reverse_ids").as_integer_array();
        for (auto id : rev_vec) reverse_ids_.insert(static_cast<uint8_t>(id));

        max_velocity_ = this->get_parameter("max_velocity").as_int();
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

        // Log reverse_ids
        if (!reverse_ids_.empty()) {
            std::string rev_str;
            for (auto id : reverse_ids_) rev_str += std::to_string(id) + " ";
            RCLCPP_INFO(this->get_logger(), "Reverse IDs: [%s]", rev_str.c_str());
        }

        initMotors();

        // ── ROS2 pub/sub ──
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&DynamixelDriverNode::cmdVelCallback, this, std::placeholders::_1));

        status_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/motor_status", 10);

        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / loop_rate)),
            std::bind(&DynamixelDriverNode::publishStatus, this));

        RCLCPP_INFO(this->get_logger(), "Dynamixel driver node ready.");
    }

    ~DynamixelDriverNode()
    {
        // Stop all motors and disable torque
        for (uint8_t id : active_ids_) {
            writeDwordRegister(id, ADDR_GOAL_VELOCITY, 0);
            writeByteRegister(id, ADDR_TORQUE_ENABLE, 0);
        }

        if (port_handler_) {
            port_handler_->closePort();
        }
        RCLCPP_INFO(this->get_logger(), "Motors stopped, port closed.");
    }

private:
    // ── Motor initialization ──────────────────────────────────────
    bool initMotors()
    {
        // Scan all motor IDs — report found and missing
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

            // Disable torque before changing mode
            writeByteRegister(id, ADDR_TORQUE_ENABLE, 0);

            // Set velocity mode
            writeByteRegister(id, ADDR_OPERATING_MODE, MODE_VELOCITY);

            // Enable torque
            writeByteRegister(id, ADDR_TORQUE_ENABLE, 1);

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
        return motors_initialized_;
    }

    // ── Callbacks ─────────────────────────────────────────────────
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!motors_initialized_) return;

        double speed = msg->linear.x;

        // Convert normalized speed (-1.0 to 1.0) to Dynamixel velocity units
        int32_t vel = static_cast<int32_t>(std::clamp(speed, -1.0, 1.0) * max_velocity_);

        // Send velocity to all active motors, negating for reverse_ids
        for (uint8_t id : active_ids_) {
            int32_t motor_vel = vel;
            if (reverse_ids_.count(id)) {
                motor_vel = -vel;
            }
            writeDwordRegister(id, ADDR_GOAL_VELOCITY, motor_vel);
        }
    }

    void publishStatus()
    {
        if (!motors_initialized_) return;

        auto msg = std_msgs::msg::Float32MultiArray();
        // Layout: [id1_vel, id1_temp, id1_voltage, id2_vel, id2_temp, ...]
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

    void writeDwordRegister(uint8_t id, uint16_t addr, int32_t value)
    {
        uint8_t dxl_error = 0;
        packet_handler_->write4ByteTxRx(port_handler_, id, addr,
            static_cast<uint32_t>(value), &dxl_error);
    }

    // ── Member variables ──────────────────────────────────────────
    std::vector<uint8_t> motor_ids_;
    std::vector<uint8_t> active_ids_;    // motors that actually responded to ping
    std::set<uint8_t>    reverse_ids_;   // motors that need velocity negated
    std::string port_name_;
    int baudrate_;
    int max_velocity_;

    bool motors_initialized_ = false;

    dynamixel::PortHandler*   port_handler_   = nullptr;
    dynamixel::PacketHandler* packet_handler_ = nullptr;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
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