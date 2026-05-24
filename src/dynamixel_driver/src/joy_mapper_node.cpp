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
 *   button 7 = all velocity mode
 *   button 8 = split mode (rear velocity, front current)
 *   button 9 = hold front drive (rear holds, front nudges)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
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
        this->declare_parameter("enable_button", -1);        // -1 = no deadman switch
        this->declare_parameter("button_all_velocity", 7);
        this->declare_parameter("button_split_mode", 8);
        this->declare_parameter("button_hold_front_drive", 9);

        deadzone_          = this->get_parameter("deadzone").as_double();
        max_linear_speed_  = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        axis_linear_       = this->get_parameter("axis_linear").as_int();
        axis_angular_      = this->get_parameter("axis_angular").as_int();
        enable_button_     = this->get_parameter("enable_button").as_int();
        btn_all_velocity_  = this->get_parameter("button_all_velocity").as_int();
        btn_split_mode_    = this->get_parameter("button_split_mode").as_int();

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        mode_pub_ = this->create_publisher<std_msgs::msg::String>("/drive_mode", 10);

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoyMapperNode::joyCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "Joy mapper ready. Button %d=all velocity, Button %d=split mode",
            btn_all_velocity_, btn_split_mode_);
    }

private:
    double applyDeadzone(double val) const // method being const shows it will not modify any member variables of the class.
    {
        if (std::abs(val) < deadzone_) return 0.0;
        // Rescale so output starts from 0 after deadzone
        double sign = (val > 0) ? 1.0 : -1.0;
        return sign * (std::abs(val) - deadzone_) / (1.0 - deadzone_);
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // ── Drive mode buttons ──
        if (btn_all_velocity_ >= 0 &&
            btn_all_velocity_ < static_cast<int>(msg->buttons.size()) &&
            msg->buttons[btn_all_velocity_])
        {
            auto mode_msg = std_msgs::msg::String();
            mode_msg.data = "all_velocity";
            mode_pub_->publish(mode_msg);
            RCLCPP_INFO(this->get_logger(), "Mode switch: ALL VELOCITY");
        }

        if (btn_split_mode_ >= 0 &&
            btn_split_mode_ < static_cast<int>(msg->buttons.size()) &&
            msg->buttons[btn_split_mode_])
        {
            auto mode_msg = std_msgs::msg::String();
            mode_msg.data = "split_mode";
            mode_pub_->publish(mode_msg);
            RCLCPP_INFO(this->get_logger(), "Mode switch: SPLIT (rear vel, front current)");
        }

        if (btn_hold_front_ >= 0 &&
            btn_hold_front_ < static_cast<int>(msg->buttons.size()) &&
            msg->buttons[btn_hold_front_])
        {
            auto mode_msg = std_msgs::msg::String();
            mode_msg.data = "hold_front_drive";
            mode_pub_->publish(mode_msg);
            RCLCPP_INFO(this->get_logger(), "Mode switch: HOLD_FRONT_DRIVE (rear hold, front nudge)");
        }

        // ── Deadman switch check ── // #TODO understand this, how do buttons work, just change of state makes it postiive etc?
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
    int btn_all_velocity_;
    int btn_split_mode_;
    int btn_hold_front_;

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