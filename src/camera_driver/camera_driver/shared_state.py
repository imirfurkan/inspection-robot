"""
Shared state between the OAK-D pipeline and the Flask dashboard.

Both oakd_node.py and dashboard/server.py import from here.
This avoids circular imports and keeps state in one place.
"""

import threading


# ═══════════════════════════════════════════════════════
# Frame buffers (written by pipeline, read by dashboard)
# ═══════════════════════════════════════════════════════

frame_lock = threading.Lock()
current_frames = {
    "rgb": None,
    "depth": None,
}


# ═══════════════════════════════════════════════════════
# Device references (set by pipeline worker)
# ═══════════════════════════════════════════════════════

device_ref = None
ctrl_queue = None


# ═══════════════════════════════════════════════════════
# Recording state
# ═══════════════════════════════════════════════════════

recording_lock = threading.Lock()
recording_active = False
recording_file = None


# ═══════════════════════════════════════════════════════
# IMU data (written by pipeline, read by dashboard)
# ═══════════════════════════════════════════════════════

imu_lock = threading.Lock()
imu_data = {
    "accel": {"x": 0.0, "y": 0.0, "z": 0.0},
    "gyro": {"x": 0.0, "y": 0.0, "z": 0.0},
    "rotation": {"i": 0.0, "j": 0.0, "k": 0.0, "real": 1.0},
    "orientation": {"pitch": 0.0, "roll": 0.0, "yaw": 0.0},
    "timestamp": 0,
}
# Reference quaternion for "zero" orientation (set by Zero IMU button)
imu_ref_quat = {"i": 0.0, "j": 0.0, "k": 0.0, "real": 1.0}
imu_ref_set = False

# Per-axis Euler offsets (set by Zero IMU)
imu_euler_offsets = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}

# Auto-zero on first valid IMU packet
imu_auto_zero_pending = True

# ═══════════════════════════════════════════════════════
# Motor status (written by oakd_node /motor_status sub, read by dashboard)
# ═══════════════════════════════════════════════════════
# Layout: list of {label, rpm, temp, voltage} for each active motor.
# Order matches dynamixel_driver active_ids_ (ping order): RL=1, FR=6, RR=8, FL=10

motor_status_lock = threading.Lock()
motor_status = []


# ═══════════════════════════════════════════════════════
# Robot position state (pitch-based classification)
# ═══════════════════════════════════════════════════════
#
# The position classifier maps absolute pitch ranges to named states.
# Each entry is (min_deg, max_deg, state_name).
# Ranges are evaluated in order; first match wins.
# Configure via ROS2 parameters: position_thresholds and position_labels.
#
# Default thresholds (overridable from YAML):
#   0–5°   → "horizontal"
#   5–15°  → "buckling"
#   15–35° → "transitional"
#   35–80° → "inclined"
#   80–90° → "vertical"

position_lock = threading.Lock()
position_state = {
    "label": "unknown",       # current classified state name
    "pitch": 0.0,             # pitch value used for classification (absolute)
    "timestamp": 0,           # time of last update
}

# The classifier table: list of (min_deg, max_deg, label)
# Written once at startup by oakd_node, read by the classifier.
position_ranges = []


# ═══════════════════════════════════════════════════════
# Pipeline control events
# ═══════════════════════════════════════════════════════

pipeline_stop = threading.Event()
pipeline_restart = threading.Event()
controls_dirty = threading.Event()

####

# Publisher set by oakd_node; used by server.py to send LED commands
led_cmd_pub = None   # rclpy publisher for /led/cmd (std_msgs/String)

####

current_drive_mode = "unknown"

# ═══════════════════════════════════════════════════════
# Camera config (modified by dashboard, read by pipeline)
# ═══════════════════════════════════════════════════════

config_lock = threading.Lock()
current_config = {
    # Stream
    "resolution": "1080p",
    "fps": 30,
    "preview_width": 640,
    "preview_height": 360,
    # Dashboard
    "dashboard_port": 8080,
    "jpeg_quality": 80,
    # H.265 recording
    "enable_h265_recording": False,
    "h265_bitrate_kbps": 3000,
    "h265_keyframe_interval": 30,
    "recording_dir": "/home/admin/recordings",
    # Exposure
    "auto_exposure": True,
    "exposure_us": 8333,
    "iso": 400,
    # Focus (FF model = fixed, kept for compatibility)
    "auto_focus": True,
    "autofocus_mode": "continuous",
    "manual_focus": 127,
    # White balance
    "auto_white_balance": True,
    "white_balance_k": 5500,
    # Image
    "brightness": 0,
    "contrast": 0,
    "saturation": 0,
    "sharpness": 1,
    "luma_denoise": 1,
    "chroma_denoise": 1,
    # IR
    "ir_dot_brightness": 0.0,
    "ir_flood_brightness": 0.0,
    # Depth
    "enable_depth": True,
    "depth_preset": "HIGH_DENSITY",
    "lr_check": True,
    "extended_disparity": False,
    "subpixel": False,
    "confidence_threshold": 200,
    "median_filter": "KERNEL_7x7",
    # Overlay
    "show_fps": True,
    "show_timestamp": False,
}