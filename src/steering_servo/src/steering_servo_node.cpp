/*
 * Steering Servo Node
 * ====================
 * Subscribes to /joy (sensor_msgs/Joy) directly
 * Drives a DS3235-180 servo via Raspberry Pi 5 hardware PWM
 *
 * TWO steering modes on separate axes:
 *
 * Axis 2 (twist) — DISCRETE:
 *   > +0.8  → snap to full right (162°), latches
 *   < -0.8  → snap to full left (10°), latches
 *   Button 3 → snap to center (90°)
 *
 * Axis 0 (stick X) — PROPORTIONAL (experimental):
 *   Maps -1.0..+1.0 proportionally to servo range
 *   Rate-limited: commands sent at most every 50ms
 *   Slew-limited: max 5° change per update (prevents slamming)
 *   Deadband: ignores changes < 2° (prevents chasing jitter)
 *
 * DS3235-180 specs:
 *   500µs  = 0°,  1500µs = 90° (center),  2500µs = 180°
 *   Physical hard-stops: [10°, 162°]
 *
 * Pi 5 hardware PWM via sysfs:
 *   /sys/class/pwm/pwmchip0/pwm0/  (GPIO 12, channel 0)
 *
 * Requires dtoverlay=pwm in /boot/firmware/config.txt:
 *   dtoverlay=pwm,pin=12,func=4
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <fstream>
#include <string>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>

class SteeringServoNode : public rclcpp::Node
{
public:
    SteeringServoNode()
    : Node("steering_servo_node"),
      pwm_initialized_(false),
      steer_triggered_left_(false),
      steer_triggered_right_(false)
    {
        // Declare parameters
        this->declare_parameter("pwm_chip", 0);
        this->declare_parameter("pwm_channel", 0);
        this->declare_parameter("servo_min_deg", 10.0);
        this->declare_parameter("servo_max_deg", 162.0);
        this->declare_parameter("servo_center_deg", 90.0);
        this->declare_parameter("pulse_min_us", 500);
        this->declare_parameter("pulse_max_us", 2500);
        this->declare_parameter("period_ns", 20000000);

        // Discrete steering (axis 2)
        this->declare_parameter("steer_threshold", 0.8);
        this->declare_parameter("axis_steer", 2);
        this->declare_parameter("button_center", 3);

        // Proportional steering (axis 0) — experimental
        this->declare_parameter("axis_proportional", 0);
        this->declare_parameter("proportional_deadzone", 0.1);   // joystick deadzone
        this->declare_parameter("angle_deadband", 2.0);          // ignore changes < 2°
        this->declare_parameter("slew_rate_deg", 5.0);           // max degrees per update
        this->declare_parameter("update_interval_ms", 50);       // min ms between commands

        // Read parameters
        pwm_chip_       = this->get_parameter("pwm_chip").as_int();
        pwm_channel_    = this->get_parameter("pwm_channel").as_int();
        servo_min_deg_  = this->get_parameter("servo_min_deg").as_double();
        servo_max_deg_  = this->get_parameter("servo_max_deg").as_double();
        servo_center_   = this->get_parameter("servo_center_deg").as_double();
        pulse_min_us_   = this->get_parameter("pulse_min_us").as_int();
        pulse_max_us_   = this->get_parameter("pulse_max_us").as_int();
        period_ns_      = this->get_parameter("period_ns").as_int();
        steer_threshold_ = this->get_parameter("steer_threshold").as_double();
        axis_steer_     = this->get_parameter("axis_steer").as_int();
        button_center_  = this->get_parameter("button_center").as_int();

        axis_proportional_    = this->get_parameter("axis_proportional").as_int();
        proportional_deadzone_ = this->get_parameter("proportional_deadzone").as_double();
        angle_deadband_       = this->get_parameter("angle_deadband").as_double();
        slew_rate_deg_        = this->get_parameter("slew_rate_deg").as_double();
        update_interval_ms_   = this->get_parameter("update_interval_ms").as_int();

        // Build sysfs path
        pwm_path_ = "/sys/class/pwm/pwmchip" + std::to_string(pwm_chip_) +
                     "/pwm" + std::to_string(pwm_channel_) + "/";

        // Initialize hardware PWM
        if (!initPWM()) {
            RCLCPP_ERROR(this->get_logger(), "PWM init failed — check dtoverlay and permissions");
        }

        // Move to center on startup
        if (pwm_initialized_) {
            setServoDegrees(servo_center_);
            RCLCPP_INFO(this->get_logger(), "Servo centered at %.1f°", servo_center_);
        }

        // Subscribe to /joy
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&SteeringServoNode::joyCallback, this, std::placeholders::_1));

        last_proportional_update_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(this->get_logger(),
            "Steering servo started — discrete:axis[%d] proportional:axis[%d] "
            "range:[%.0f°–%.0f°] center:%.0f°",
            axis_steer_, axis_proportional_,
            servo_min_deg_, servo_max_deg_, servo_center_);
    }

    ~SteeringServoNode()
    {
        if (pwm_initialized_) {
            setServoDegrees(servo_center_);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            disablePWM();
        }
    }

private:
    // ─── Hardware PWM via sysfs ────────────────────────────────────
    bool initPWM()
    {
        std::string export_path = "/sys/class/pwm/pwmchip" +
                                   std::to_string(pwm_chip_) + "/export";
        {
            std::ifstream test(pwm_path_ + "period");
            if (!test.good()) {
                std::ofstream export_file(export_path);
                if (!export_file.is_open()) {
                    RCLCPP_ERROR(this->get_logger(),
                        "Cannot open %s — is dtoverlay=pwm enabled?", export_path.c_str());
                    return false;
                }
                export_file << pwm_channel_;
                export_file.close();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        if (!writeSysfs(pwm_path_ + "period", std::to_string(period_ns_)))
            return false;

        int center_ns = degreesToNs(servo_center_);
        if (!writeSysfs(pwm_path_ + "duty_cycle", std::to_string(center_ns)))
            return false;

        if (!writeSysfs(pwm_path_ + "enable", "1"))
            return false;

        pwm_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Hardware PWM initialized at %s", pwm_path_.c_str());
        return true;
    }

    void disablePWM()
    {
        writeSysfs(pwm_path_ + "enable", "0");
        RCLCPP_INFO(this->get_logger(), "PWM disabled");
    }

    bool writeSysfs(const std::string& path, const std::string& value)
    {
        std::ofstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Cannot write to %s", path.c_str());
            return false;
        }
        file << value;
        return file.good();
    }

    // ─── Servo math ───────────────────────────────────────────────
    int degreesToNs(double degrees)
    {
        degrees = std::clamp(degrees, servo_min_deg_, servo_max_deg_);
        double pulse_us = pulse_min_us_ + (degrees / 180.0) * (pulse_max_us_ - pulse_min_us_);
        return static_cast<int>(pulse_us * 1000.0);
    }

    void setServoDegrees(double degrees)
    {
        if (!pwm_initialized_) return;
        degrees = std::clamp(degrees, servo_min_deg_, servo_max_deg_);
        int duty_ns = degreesToNs(degrees);
        writeSysfs(pwm_path_ + "duty_cycle", std::to_string(duty_ns));
        current_angle_ = degrees;
    }

    double applyJoystickDeadzone(double value)
    {
        if (std::abs(value) < proportional_deadzone_) return 0.0;
        double sign = (value > 0) ? 1.0 : -1.0;
        return sign * (std::abs(value) - proportional_deadzone_) / (1.0 - proportional_deadzone_);
    }

    // ─── ROS callback ─────────────────────────────────────────────
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (!pwm_initialized_) return;

        // === Button 3: center servo (overrides everything) ===
        if (msg->buttons.size() > static_cast<size_t>(button_center_) &&
            msg->buttons[button_center_] == 1) {
            setServoDegrees(servo_center_);
            steer_triggered_left_  = false;
            steer_triggered_right_ = false;
            RCLCPP_INFO(this->get_logger(), "Steering centered (button %d)", button_center_);
            return;
        }

        // === Axis 2 — Discrete steering (unchanged) ===
        if (msg->axes.size() > static_cast<size_t>(axis_steer_)) {
            double twist_val = msg->axes[axis_steer_];

            if (twist_val > steer_threshold_) {
                if (!steer_triggered_right_) {
                    setServoDegrees(servo_max_deg_);
                    steer_triggered_right_ = true;
                    steer_triggered_left_  = false;
                    RCLCPP_INFO(this->get_logger(), "Discrete RIGHT → %.0f°", servo_max_deg_);
                    return;  // discrete takes priority this callback
                }
            }
            else if (twist_val < -steer_threshold_) {
                if (!steer_triggered_left_) {
                    setServoDegrees(servo_min_deg_);
                    steer_triggered_left_  = true;
                    steer_triggered_right_ = false;
                    RCLCPP_INFO(this->get_logger(), "Discrete LEFT → %.0f°", servo_min_deg_);
                    return;
                }
            }
        }

        // === Axis 0 — Proportional steering (experimental) ===
        if (msg->axes.size() <= static_cast<size_t>(axis_proportional_)) return;

        // Rate-limit: skip if not enough time has passed
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_proportional_update_).count();
        if (elapsed < update_interval_ms_) return;

        double stick_val = msg->axes[axis_proportional_];

        // Apply joystick deadzone
        stick_val = applyJoystickDeadzone(stick_val);

        // Map stick (-1.0..+1.0) to target angle
        double target_deg;
        if (stick_val >= 0.0) {
            target_deg = servo_center_ + stick_val * (servo_max_deg_ - servo_center_);
        } else {
            target_deg = servo_center_ + stick_val * (servo_center_ - servo_min_deg_);
        }

        // Deadband: ignore if target is too close to current position
        if (std::abs(target_deg - current_angle_) < angle_deadband_) return;

        // Slew rate limit: move toward target by at most slew_rate_deg_ per update
        double diff = target_deg - current_angle_;
        if (std::abs(diff) > slew_rate_deg_) {
            target_deg = current_angle_ + ((diff > 0) ? slew_rate_deg_ : -slew_rate_deg_);
        }

        setServoDegrees(target_deg);
        last_proportional_update_ = now;
    }

    // ─── Members ──────────────────────────────────────────────────
    bool pwm_initialized_;
    bool steer_triggered_left_;
    bool steer_triggered_right_;
    int pwm_chip_;
    int pwm_channel_;
    double servo_min_deg_;
    double servo_max_deg_;
    double servo_center_;
    int pulse_min_us_;
    int pulse_max_us_;
    int period_ns_;
    double steer_threshold_;
    int axis_steer_;
    int button_center_;
    double current_angle_ = 90.0;
    std::string pwm_path_;

    // Proportional steering
    int axis_proportional_;
    double proportional_deadzone_;
    double angle_deadband_;
    double slew_rate_deg_;
    int update_interval_ms_;
    std::chrono::steady_clock::time_point last_proportional_update_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SteeringServoNode>());
    rclcpp::shutdown();
    return 0;
}
