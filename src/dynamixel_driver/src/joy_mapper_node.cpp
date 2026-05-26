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
 * Button mapping:
 *   button 0 = trigger
 *   button 3 = center servo (used for steering)
 *   button 7  = drive_all (all velocity, normal driving)
 *   button 8  = drive_rear_assist (rear vel + front current)
 *   button 9  = drive_front_nudge (rear hold + front current nudge)
 *   button 10 = drive_front_only (rear hold + front velocity)
 *   button 11 = drive_rear_only (rear velocity + front hold)
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
        // Declare parameters
        this->declare_parameter("deadzone", 0.15);
        this->declare_parameter("max_linear_speed", 1.0);
        this->declare_parameter("max_angular_speed", 1.0);
        this->declare_parameter("axis_linear", 1);
        this->declare_parameter("axis_angular", 2);
        this->declare_parameter("enable_button", -1);

        // Drive mode button assignments (Extreme 3D Pro has buttons 0-11)
        this->declare_parameter("button_drive_all", 7);
        this->declare_parameter("button_drive_rear_assist", 8);
        this->declare_parameter("button_drive_front_nudge", 9);
        this->declare_parameter("button_drive_front_only", 10);
        this->declare_parameter("button_drive_rear_only", 11);

        deadzone_          = this->get_parameter("deadzone").as_double();
        max_linear_speed_  = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        axis_linear_       = this->get_parameter("axis_linear").as_int();
        axis_angular_      = this->get_parameter("axis_angular").as_int();
        enable_button_     = this->get_parameter("enable_button").as_int();

        // Build mode button table: pairs of (button_index, mode_string)
        // This avoids repeating the same if-block 5 times
        auto addMode = [&](const std::string& param, const std::string& mode_name) {
            int btn = this->get_parameter(param).as_int();
            if (btn >= 0) {
                mode_buttons_.push_back({btn, mode_name});
            }
        };
        addMode("button_drive_all",          "drive_all");
        addMode("button_drive_rear_assist",  "drive_rear_assist");
        addMode("button_drive_front_nudge",  "drive_front_nudge");
        addMode("button_drive_front_only",   "drive_front_only");
        addMode("button_drive_rear_only",    "drive_rear_only");

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        mode_pub_ = this->create_publisher<std_msgs::msg::String>("/drive_mode", 10);

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoyMapperNode::joyCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Joy mapper ready. Mode buttons:");
        for (const auto& [btn, name] : mode_buttons_) {
            RCLCPP_INFO(this->get_logger(), "  Button %d → %s", btn, name.c_str());
        }
    }

private:
    double applyDeadzone(double val) const
    {
        if (std::abs(val) < deadzone_) return 0.0;
        // Rescale so output starts from 0 after deadzone
        double sign = (val > 0) ? 1.0 : -1.0;
        return sign * (std::abs(val) - deadzone_) / (1.0 - deadzone_);
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // ── Drive mode buttons ──
        // Check all registered mode buttons. If pressed, publish the mode string.
        for (const auto& [btn, mode_name] : mode_buttons_) {
            if (btn < static_cast<int>(msg->buttons.size()) && msg->buttons[btn]) {
                auto mode_msg = std_msgs::msg::String();
                mode_msg.data = mode_name;
                mode_pub_->publish(mode_msg);
                RCLCPP_INFO(this->get_logger(), "Mode: %s", mode_name.c_str());
            }
        }

        // ── Deadman switch check ──
        // If enable_button is set (>= 0), the button must be held for commands to pass.
        // If not held, publish zero velocity and skip axis reading.
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
            // Invert Y axis — push forward gives negative raw value
            linear = -applyDeadzone(msg->axes[axis_linear_]) * max_linear_speed_;
        }

        if (axis_angular_ < static_cast<int>(msg->axes.size())) {
            angular = applyDeadzone(msg->axes[axis_angular_]) * max_angular_speed_;
        }

        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = linear;
        twist.angular.z = angular;
        cmd_pub_->publish(twist);
    }

    // Parameters
    double deadzone_;
    double max_linear_speed_;
    double max_angular_speed_;
    int axis_linear_;
    int axis_angular_;
    int enable_button_;

    // Mode button table: each entry is (button_index, mode_string_to_publish)
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