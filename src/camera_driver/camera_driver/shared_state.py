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
# Pipeline control events
# ═══════════════════════════════════════════════════════

pipeline_stop = threading.Event()
pipeline_restart = threading.Event()
controls_dirty = threading.Event()


# ═══════════════════════════════════════════════════════
# Camera config (modified by dashboard, read by pipeline)
# ═══════════════════════════════════════════════════════

config_lock = threading.Lock()
current_config = {
    # Stream
    "resolution": "1080p",
    "fps": 30,
    "preview_width": 1280,
    "preview_height": 720,
    # Dashboard
    "dashboard_port": 8080,
    "jpeg_quality": 80,
    # H.265 recording
    "enable_h265_recording": False,
    "h265_bitrate_kbps": 3000,
    "h265_keyframe_interval": 30,
    "recording_dir": "/root/ros2_ws/recordings",
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