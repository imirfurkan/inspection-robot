/*
 * Joy Mapper Node
 * ===============
 * Subscribes to /joy (sensor_msgs/Joy) from joy_node
 * Publishes /cmd_vel (geometry_msgs/Twist)
 * Publishes /drive_mode (std_msgs/String)
 *
 * Logitech Extreme 3D Pro axis mapping:
 *   axis 0 = stick X  (left/right)
 *   axis 1 = stick Y  (forward/back, INVERTED: push forward = negative)
 *   axis 2 = stick twist Z (rotation)
 *   axis 3 = throttle slider
 *
 * Mode-to-button mapping is read entirely from YAML parameters:
 *   mode_names:   ["drive_all", "drive_rear_assist", "drive_pivot_left", ...]
 *   mode_buttons: [7, 8, 4, ...]
 *
 * To add a new mode:
 *   1. Add the behavior in drive_modes.cpp
 *   2. Add the name + button number to YAML
 *   No C++ changes needed here.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <vector>
#include <string>

class JoyMapperNode : public rclcpp::Node
{
public:
    JoyMapperNode()
    : Node("joy_mapper_node")
    {
        // ── Standard parameters ──
        this->declare_parameter("deadzone", 0.15);
        this->declare_parameter("max_linear_speed", 1.0);
        this->declare_parameter("max_angular_speed", 1.0);
        this->declare_parameter("axis_linear", 1);
        this->declare_parameter("axis_angular", 2);
        this->declare_parameter("enable_button", -1);

        // ── Mode mapping from YAML ──
        // Two parallel arrays: mode_names[i] is triggered by mode_buttons[i].
        // If these aren't set, fall back to empty (no mode buttons).
        this->declare_parameter("mode_names", std::vector<std::string>{});
        this->declare_parameter("mode_buttons", std::vector<int64_t>{});

        deadzone_          = this->get_parameter("deadzone").as_double();
        max_linear_speed_  = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        axis_linear_       = this->get_parameter("axis_linear").as_int();
        axis_angular_      = this->get_parameter("axis_angular").as_int();
        enable_button_     = this->get_parameter("enable_button").as_int();

        // Build mode button table from the parallel arrays
        auto names   = this->get_parameter("mode_names").as_string_array();
        auto buttons = this->get_parameter("mode_buttons").as_integer_array();

        if (names.size() != buttons.size()) {
            RCLCPP_ERROR(this->get_logger(),
                "mode_names (%zu) and mode_buttons (%zu) must be the same length!",
                names.size(), buttons.size());
        }

        size_t count = std::min(names.size(), buttons.size());
        for (size_t i = 0; i < count; ++i) {
            int btn = static_cast<int>(buttons[i]);
            if (btn >= 0) {
                mode_buttons_.push_back({btn, names[i]});
            }
        }

            
        // ── Publishers / Subscribers ──
        cmd_pub_  = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        mode_pub_ = this->create_publisher<std_msgs::msg::String>("/drive_mode", 10);

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoyMapperNode::joyCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Joy mapper ready. Mode buttons:");
        for (const auto& [btn, name] : mode_buttons_) {
            RCLCPP_INFO(this->get_logger(), "  Button %d → %s", btn, name.c_str());
        }
        if (mode_buttons_.empty()) {
            RCLCPP_WARN(this->get_logger(),
                "No mode buttons configured. Set mode_names and mode_buttons in YAML.");
        }
    }

private:
    double applyDeadzone(double val) const
    {
        if (std::abs(val) < deadzone_) return 0.0;
        double sign = (val > 0) ? 1.0 : -1.0;
        return sign * (std::abs(val) - deadzone_) / (1.0 - deadzone_);
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // ── Drive mode buttons ──
        for (const auto& [btn, mode_name] : mode_buttons_) {
            if (btn < static_cast<int>(msg->buttons.size()) && msg->buttons[btn]) {
                auto mode_msg = std_msgs::msg::String();
                mode_msg.data = mode_name;
                mode_pub_->publish(mode_msg);
                RCLCPP_INFO(this->get_logger(), "Mode: %s", mode_name.c_str());
            }
        }

        // ── Deadman switch check ──
        if (enable_button_ >= 0) {
            if (enable_button_ >= static_cast<int>(msg->buttons.size()) ||
                !msg->buttons[enable_button_])
            {
                auto twist = geometry_msgs::msg::Twist();
                cmd_pub_->publish(twist);
                return;
            }
        }

        // ── Axis mapping ──
        double linear = 0.0, angular = 0.0;

        if (axis_linear_ < static_cast<int>(msg->axes.size())) {
            linear = -applyDeadzone(msg->axes[axis_linear_]) * max_linear_speed_;
        }

        if (axis_angular_ < static_cast<int>(msg->axes.size())) {
            angular = applyDeadzone(msg->axes[axis_angular_]) * max_angular_speed_;
        }

        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = linear;
        twist.angular.z = angular;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "Publishing linear.x=%.3f, angular.z=%.3f", twist.linear.x, twist.angular.z);
        cmd_pub_->publish(twist);
    }

    // Parameters
    double deadzone_;
    double max_linear_speed_;
    double max_angular_speed_;
    int axis_linear_;
    int axis_angular_;
    int enable_button_;

    // Mode button table: built from YAML, each entry is (button_index, mode_name)
    std::vector<std::pair<int, std::string>> mode_buttons_;

    // ROS2
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyMapperNode>());
    rclcpp::shutdown();
    return 0;
}