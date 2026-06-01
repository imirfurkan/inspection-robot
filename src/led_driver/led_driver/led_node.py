#!/usr/bin/env python3
"""
LED Driver Node
================
Controls MOSFET-driven LED strip via PWM on GPIO13.

Subscribes:
  /led_brightness  (std_msgs/Float32)  0.0–1.0
  /led_mode        (std_msgs/String)   "on"|"off"|"breath"|"strobe"|"set"
Params: gpio_pin, pwm_frequency, default_brightness
"""

import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

try:
    from gpiozero import PWMLED
except ImportError:
    PWMLED = None


class LedDriverNode(Node):
    def __init__(self):
        super().__init__('led_driver_node')

        self.declare_parameter('gpio_pin', 13)
        self.declare_parameter('default_brightness', 0.0)

        pin = self.get_parameter('gpio_pin').value
        self._brightness = float(self.get_parameter('default_brightness').value)
        self._mode = 'off'
        self._lock = threading.Lock()
        self._effect_thread = None
        self._stop_effect = threading.Event()

        if PWMLED is None:
            self.get_logger().error("gpiozero not installed! pip install gpiozero lgpio --break-system-packages")
            self._led = None
        else:
            try:
                self._led = PWMLED(pin)
                self._led.value = self._brightness
                self.get_logger().info(f"LED on GPIO{pin}, brightness={self._brightness}")
            except Exception as e:
                self.get_logger().error(f"LED init failed: {e}")
                self._led = None

        self.create_subscription(Float32, '/led_brightness', self._cb_brightness, 10)
        self.create_subscription(String,  '/led_mode',       self._cb_mode,       10)
        self.create_subscription(String, '/led/cmd', self._on_cmd, 10)


    # ── ROS callbacks ──────────────────────────────────────────────

    def _cb_brightness(self, msg):
        val = max(0.0, min(1.0, msg.data))
        with self._lock:
            self._brightness = val
            if self._mode == 'set':
                self._set_raw(val)

    def _cb_mode(self, msg):
        self.set_mode(msg.data)

    # ── Public API (also called by dashboard thread) ───────────────

    def set_mode(self, mode: str, brightness: float = None):
        """Thread-safe mode switch. Called by ROS sub or Flask."""
        with self._lock:
            if brightness is not None:
                self._brightness = max(0.0, min(1.0, brightness))
            self._mode = mode
            self._stop_effect.set()  # stop any running effect

        # Start effect thread for animated modes
        if mode in ('breath', 'strobe'):
            self._stop_effect.clear()
            self._effect_thread = threading.Thread(
                target=self._run_effect, args=(mode,), daemon=True)
            self._effect_thread.start()
        elif mode == 'on':
            self._set_raw(1.0)
        elif mode == 'off':
            self._set_raw(0.0)
        elif mode == 'set':
            self._set_raw(self._brightness)

    def get_state(self):
        with self._lock:
            return {'mode': self._mode, 'brightness': self._brightness}
        
    def _on_cmd(self, msg):
        # Parse "set:0.75" or plain mode names
        if msg.data.startswith('set:'):
            self.set_mode('set', float(msg.data.split(':')[1]))
        else:
            self.set_mode(msg.data)

    # ── Internal ───────────────────────────────────────────────────

    def _set_raw(self, val):
        if self._led:
            self._led.value = float(val)

    def _run_effect(self, effect):
        if effect == 'breath':
            while not self._stop_effect.is_set():
                for i in range(0, 101, 2):
                    if self._stop_effect.is_set(): return
                    self._set_raw(i / 100.0)
                    time.sleep(0.02)
                for i in range(100, -1, -2):
                    if self._stop_effect.is_set(): return
                    self._set_raw(i / 100.0)
                    time.sleep(0.02)
        elif effect == 'strobe':
            while not self._stop_effect.is_set():
                self._set_raw(1.0); time.sleep(0.05)
                self._set_raw(0.0); time.sleep(0.05)
        self._set_raw(0.0)

    def destroy_node(self):
        self._stop_effect.set()
        if self._led:
            self._led.value = 0.0
            self._led.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LedDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()