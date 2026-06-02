/*
 * Joy Mapper Node
 * ===============
 * Subscribes to /joy (sensor_msgs/Joy) from joy_node
 * Publishes /cmd_vel (geometry_msgs/Twist)
 * Publishes /drive_mode (std_msgs/String)
 *
 * Logitech Extreme 3D Pro axis mapping:
 *   axis 0 = stick X  (left/right)   → proportional steering (servo)
 *   axis 1 = stick Y  (forward/back, INVERTED: push forward = negative)
 *   axis 2 = stick twist Z (rotation) → tank turn speed + trigger
 *   axis 3 = throttle slider
 *
 * Tank turn (button 1 held + axis 2):
 *   Button 1 must be held. X/Y axes must be within tank_xy_deadzone_.
 *   twist > +threshold  → "tank_turn_right" mode, twist value → linear.x speed
 *   twist < -threshold  → "tank_turn_left"  mode, twist value → linear.x speed
 *   released/center     → restores tank_turn_restore_mode_
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
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
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
        this->declare_parameter("axis0_deadzone", 0.2);


        // ── Tank turn parameters ──
        this->declare_parameter("tank_turn_threshold", 0.5);
        this->declare_parameter("tank_turn_restore_mode", std::string("drive_all"));
        this->declare_parameter("tank_turn_button", 1);         // button that must be held

        // ── Mode mapping from YAML ──
        // Two parallel arrays: mode_names[i] is triggered by mode_buttons[i].
        this->declare_parameter("mode_names", std::vector<std::string>{});
        this->declare_parameter("mode_buttons", std::vector<int64_t>{});

        deadzone_          = this->get_parameter("deadzone").as_double();
        max_linear_speed_  = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        axis_linear_       = this->get_parameter("axis_linear").as_int();
        axis_angular_      = this->get_parameter("axis_angular").as_int();
        enable_button_     = this->get_parameter("enable_button").as_int();
        tank_threshold_    = this->get_parameter("tank_turn_threshold").as_double();
        tank_restore_mode_ = this->get_parameter("tank_turn_restore_mode").as_string();
        tank_button_       = this->get_parameter("tank_turn_button").as_int();
        axis0_deadzone_    = this->get_parameter("axis0_deadzone").as_double();


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
        imu_zero_pub_  = this->create_publisher<std_msgs::msg::Empty>("/imu/zero", 10);
        imu_reset_pub_ = this->create_publisher<std_msgs::msg::Empty>("/imu/reset", 10);
        vel_limit_pub_ = this->create_publisher<std_msgs::msg::Bool>("/velocity_limit", 10);


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

    void publishMode(const std::string& mode_name)
    {
        auto m = std_msgs::msg::String();
        m.data = mode_name;
        mode_pub_->publish(m);
        RCLCPP_INFO(this->get_logger(), "Mode: %s", mode_name.c_str());
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // ── Drive mode buttons ──
        for (const auto& [btn, mode_name] : mode_buttons_) {
            if (btn < static_cast<int>(msg->buttons.size()) && msg->buttons[btn]) {
                publishMode(mode_name);
                tank_active_ = false;  // clear tank state on explicit mode button
            }
        }

        // // ── IMU zero/reset buttons ──
        // if (msg->buttons.size() > 4 && msg->buttons[4] && !imu_zero_prev_) {
        //     imu_zero_pub_->publish(std_msgs::msg::Empty());
        // }
        // if (msg->buttons.size() > 5 && msg->buttons[5] && !imu_reset_prev_) {
        //     imu_reset_pub_->publish(std_msgs::msg::Empty());
        // }

        // ── Velocity limit button ──
        if (msg->buttons.size() > 4 && msg->buttons[4] && !vel_limit_prev_) {
            auto m = std_msgs::msg::Bool();
            m.data = !vel_limit_active_;          // toggle
            vel_limit_active_ = m.data;
            vel_limit_pub_->publish(m);
        }
        vel_limit_prev_ = msg->buttons.size() > 4 && msg->buttons[4];

        // imu_zero_prev_  = msg->buttons.size() > 4 && msg->buttons[4];
        // imu_reset_prev_ = msg->buttons.size() > 5 && msg->buttons[5];

        // ── Axis 2 — tank turn (button 1 held + X/Y within deadzone) ──
        // Speed comes from the twist axis value, written into linear.x.
        if (msg->axes.size() > 2) {
            bool button_held = (tank_button_ < static_cast<int>(msg->buttons.size()) &&
                                msg->buttons[tank_button_]);
            bool xy_clear = (std::abs(msg->axes[0]) < axis0_deadzone_ &&
                             applyDeadzone(msg->axes[1]) == 0.0);
            double twist     = msg->axes[2];

            if (button_held && xy_clear && std::abs(twist) > tank_threshold_) {
                if (twist > tank_threshold_) {
                    if (!tank_active_ || tank_direction_ != 1) {
                        publishMode("tank_turn_left");
                        tank_active_    = true;
                        tank_direction_ = 1;                    }
                } else {
                    if (!tank_active_ || tank_direction_ != -1) {
                        publishMode("tank_turn_right");
                        tank_active_    = true;
                        tank_direction_ = -1;
                    }
                }

                // Publish twist axis as linear.x speed (ignore Y stick during tank)
                auto cmd = geometry_msgs::msg::Twist();
                cmd.linear.x = std::abs(twist) * max_linear_speed_;  // always positive; mode handles direction
                RCLCPP_INFO(this->get_logger(), "cmd_vel: linear=%.3f", cmd.linear.x);
                cmd_pub_->publish(cmd);
                return;  // skip normal axis mapping below

            } else if (tank_active_) {
                publishMode(tank_restore_mode_);
                tank_active_    = false;
                tank_direction_ = 0;
            }
        }

        // ── Enable toggle (rising edge) ──
        if (enable_button_ >= 0) {
            bool pressed = (enable_button_ < static_cast<int>(msg->buttons.size()) &&
                            msg->buttons[enable_button_]);
            if (pressed && !enable_btn_prev_) {
                joy_enabled_ = !joy_enabled_;
                RCLCPP_INFO(this->get_logger(), "Joy input %s", joy_enabled_ ? "ENABLED" : "DISABLED");
            }
            enable_btn_prev_ = pressed;

            if (!joy_enabled_) {
                cmd_pub_->publish(geometry_msgs::msg::Twist());
                return;
            }
        }

        // ── Normal axis mapping ──
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
        cmd_pub_->publish(twist);
    }

    // Parameters
    double deadzone_;
    double max_linear_speed_;
    double max_angular_speed_;
    int axis_linear_;
    int axis_angular_;
    int enable_button_;
    double tank_threshold_;
    std::string tank_restore_mode_;
    int tank_button_;
    double axis0_deadzone_;
    bool imu_zero_prev_  = false;
    bool imu_reset_prev_ = false;

    // Tank turn state
    bool tank_active_    = false;
    int  tank_direction_ = 0;  // -1 = left, 0 = none, 1 = right

    bool vel_limit_prev_ = false;
    bool vel_limit_active_ = false;

    bool joy_enabled_     = false;  // toggle state
    bool enable_btn_prev_ = false;

    // Mode button table: built from YAML, each entry is (button_index, mode_name)
    std::vector<std::pair<int, std::string>> mode_buttons_;

    // ROS2
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr imu_zero_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr imu_reset_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vel_limit_pub_;

    // Empty is a ROS2 message type with no fields at all. Used purely as a signal/trigger when you only care
    // that a message was sent, not what it contains. Like a doorbell — you don't need data, just the ring.
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyMapperNode>());
    rclcpp::shutdown();
    return 0;
}