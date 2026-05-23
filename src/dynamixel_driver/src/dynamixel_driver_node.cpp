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