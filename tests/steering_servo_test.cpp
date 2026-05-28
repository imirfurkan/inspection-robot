#!/bin/bash
# DS3235-180 servo test via Pi 5 hardware PWM (sysfs)
# GPIO 12 = pwmchip0/pwm0
#
# Requires: dtoverlay=pwm,pin=12,func=4 in /boot/firmware/config.txt
# Run with: sudo bash test_servo_hwpwm.sh
#
# DS3235-180 specs:
#   500us  = 0°
#   1500us = 90° (center)
#   2500us = 180°
#   Physical hard-stops: ~10° to ~162°

PWM_CHIP="/sys/class/pwm/pwmchip0"
PWM_CH="$PWM_CHIP/pwm0"
PERIOD_NS=20000000  # 50 Hz = 20ms

# Convert degrees to duty_cycle in nanoseconds
deg_to_ns() {
    local deg=$1
    # pulse_us = 500 + (deg / 180) * 2000
    # pulse_ns = pulse_us * 1000
    echo $(( (500 + deg * 2000 / 180) * 1000 ))
}

# Export PWM channel if not already exported
if [ ! -d "$PWM_CH" ]; then
    echo 0 > "$PWM_CHIP/export"
    sleep 0.1
fi

# Set period (50 Hz)
echo $PERIOD_NS > "$PWM_CH/period"

# Enable
echo 1 > "$PWM_CH/enable"

echo "=== DS3235 Servo Test (Hardware PWM) ==="
echo "GPIO 12, pwmchip0/pwm0"
echo ""

# Center
echo "Moving to 90° (center)..."
echo $(deg_to_ns 90) > "$PWM_CH/duty_cycle"
sleep 2

// # Left hard-stop
// echo "Moving to 10° (left limit)..."
// echo $(deg_to_ns 10) > "$PWM_CH/duty_cycle"
// sleep 2

// # Right hard-stop
// echo "Moving to 162° (right limit)..."
// echo $(deg_to_ns 162) > "$PWM_CH/duty_cycle"
// sleep 2

// # Back to center
// echo "Moving to 90° (center)..."
// echo $(deg_to_ns 90) > "$PWM_CH/duty_cycle"
// sleep 2

// # Disable PWM (stops servo buzz)
// echo 0 > "$PWM_CH/enable"

// echo "Done. PWM disabled."