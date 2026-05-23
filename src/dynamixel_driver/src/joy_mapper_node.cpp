/*
 * Joy Mapper Node
 * ===============
 * Subscribes to /joy (sensor_msgs/Joy) from joy_node
 * Publishes /cmd_vel (geometry_msgs/Twist)
 *
 * This node translates your Logitech Extreme 3D Pro axes/buttons
 * into velocity commands. Keeping this separate from the motor driver
 * means you can swap joysticks or add autonomous control later
 * without touching motor code.
 *
 * Logitech Extreme 3D Pro axis mapping:
 *   axis 0 = stick X  (left/right)
 *   axis 1 = stick Y  (forward/back, INVERTED: push forward = negative)
 *   axis 2 = stick twist Z (rotation)
 *   axis 3 = throttle slider
 *
 * Button mapping:
 *   button 0 = trigger
 *   button 3 = center servo (used later for steering)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class JoyMapperNode : public rclcpp::Node
{
public:
    JoyMapperNode()
    : Node("joy_mapper_node")
    {
        // Declare parameters
        this->declare_parameter("deadzone", 0.15);
        this->declare_parameter("max_linear_speed", 1.0);   // normalized 0-1
        this->declare_parameter("max_angular_speed", 1.0);   // normalized 0-1
        this->declare_parameter("axis_linear", 1);           // stick Y
        this->declare_parameter("axis_angular", 2);          // stick twist
        this->declare_parameter("axis_block", 0);            // stick X
        this->declare_parameter("block_threshold", 0.5);     // X-axis blocking
        this->declare_parameter("enable_button", -1);        // -1 = no deadman switch

        deadzone_         = this->get_parameter("deadzone").as_double();
        max_linear_speed_  = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        axis_linear_      = this->get_parameter("axis_linear").as_int();
        axis_angular_     = this->get_parameter("axis_angular").as_int();
        axis_block_       = this->get_parameter("axis_block").as_int();
        block_threshold_  = this->get_parameter("block_threshold").as_double();
        enable_button_    = this->get_parameter("enable_button").as_int();

        // Publishers and subscribers
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoyMapperNode::joyCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "Joy mapper started — linear:axis[%d] angular:axis[%d] block:axis[%d] deadzone:%.2f",
            axis_linear_, axis_angular_, axis_block_, deadzone_);
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        geometry_msgs::msg::Twist twist;

        // Safety: check array bounds
        if (msg->axes.size() <= static_cast<size_t>(std::max({axis_linear_, axis_angular_, axis_block_}))) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Joy message has fewer axes than expected");
            cmd_pub_->publish(twist);  // publish zero
            return;
        }

        // Deadman switch: if configured, require button to be held
        if (enable_button_ >= 0) {
            if (msg->buttons.size() <= static_cast<size_t>(enable_button_) ||
                msg->buttons[enable_button_] == 0) {
                cmd_pub_->publish(twist);  // publish zero
                return;
            }
        }

        // Read raw axes
        double linear_raw  = -msg->axes[axis_linear_];   // invert: push forward = positive
        double angular_raw =  msg->axes[axis_angular_];
        double block_raw   =  msg->axes[axis_block_];

        // X-axis blocking: if stick pushed too far sideways, kill motors
        if (std::abs(block_raw) > block_threshold_) {
            cmd_pub_->publish(twist);  // publish zero
            return;
        }

        // Apply deadzone
        linear_raw  = applyDeadzone(linear_raw);
        angular_raw = applyDeadzone(angular_raw);

        // Scale to max speeds
        twist.linear.x  = linear_raw  * max_linear_speed_;
        twist.angular.z = angular_raw * max_angular_speed_;

        cmd_pub_->publish(twist);
    }

    double applyDeadzone(double value)
    {
        if (std::abs(value) < deadzone_) {
            return 0.0;
        }
        // Rescale so output starts from 0 after deadzone
        double sign = (value > 0) ? 1.0 : -1.0;
        return sign * (std::abs(value) - deadzone_) / (1.0 - deadzone_);
    }

    // Parameters
    double deadzone_;
    double max_linear_speed_;
    double max_angular_speed_;
    int    axis_linear_;
    int    axis_angular_;
    int    axis_block_;
    double block_threshold_;
    int    enable_button_;

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyMapperNode>());
    rclcpp::shutdown();
    return 0;
}
