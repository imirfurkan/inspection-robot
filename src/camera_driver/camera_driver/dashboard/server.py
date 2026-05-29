"""
Flask web server for the operator dashboard.

Serves camera streams (MJPEG), snapshots, live controls, and recording toggle.
All state is imported from shared_state — no circular dependencies.
"""

import os
import time
from datetime import datetime

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request, render_template_string

from camera_driver.shared_state import (
    frame_lock, current_frames,
    config_lock, current_config,
    imu_lock, imu_data,
    recording_lock,
    pipeline_restart, controls_dirty,
    device_ref,
    position_lock, position_state, position_ranges,
)
import camera_driver.shared_state as state
from camera_driver.dashboard.template import DASHBOARD_HTML
import logging


# ═══════════════════════════════════════════════════════
# MJPEG generators
# ═══════════════════════════════════════════════════════

def mjpeg_gen(stream_key):
    """Yields MJPEG frames for a given stream."""
    while True:
        with frame_lock:
            frame = current_frames.get(stream_key)
        if frame is not None:
            with config_lock:
                quality = current_config["jpeg_quality"]
            ok, buf = cv2.imencode(".jpg", frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, quality])
            if ok:
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n"
                       + buf.tobytes() + b"\r\n")
        time.sleep(0.03)


def mjpeg_both():
    """Yields side-by-side RGB + depth as MJPEG."""
    while True:
        with frame_lock:
            rgb = current_frames.get("rgb")
            depth = current_frames.get("depth")

        if rgb is not None:
            if depth is not None:
                h, w = rgb.shape[:2]
                depth_resized = cv2.resize(depth, (w, h))
                combined = np.hstack([rgb, depth_resized])
            else:
                combined = rgb

            with config_lock:
                quality = current_config["jpeg_quality"]
            ok, buf = cv2.imencode(".jpg", combined,
                                   [cv2.IMWRITE_JPEG_QUALITY, quality])
            if ok:
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n"
                       + buf.tobytes() + b"\r\n")
        time.sleep(0.03)


# ═══════════════════════════════════════════════════════
# Flask app
# ═══════════════════════════════════════════════════════

flask_app = Flask(__name__)
logging.getLogger('werkzeug').setLevel(logging.WARNING)


@flask_app.route("/")
def dashboard():
    return render_template_string(DASHBOARD_HTML)


@flask_app.route("/video/rgb")
def video_rgb():
    return Response(mjpeg_gen("rgb"),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@flask_app.route("/video/depth")
def video_depth():
    return Response(mjpeg_gen("depth"),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@flask_app.route("/video/both")
def video_both_route():
    return Response(mjpeg_both(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@flask_app.route("/snapshot")
def snapshot():
    with frame_lock:
        frame = current_frames.get("rgb")
    if frame is None:
        return "No frame available", 503
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        return "Encode failed", 500
    return Response(buf.tobytes(), mimetype="image/jpeg",
                    headers={"Content-Disposition": "inline; filename=snapshot.jpg"})


@flask_app.route("/status")
def status():
    with config_lock:
        cfg = dict(current_config)
    with recording_lock:
        cfg["recording"] = state.recording_active
    cfg["device_connected"] = state.device_ref is not None
    return jsonify(cfg)


@flask_app.route("/api/controls", methods=["POST"])
def api_controls():
    """Update live controls (no pipeline restart needed)."""
    data = request.get_json(force=True)
    live_keys = {
        "auto_exposure", "exposure_us", "iso",
        "auto_white_balance", "white_balance_k",
        "brightness", "contrast", "saturation", "sharpness",
        "luma_denoise", "chroma_denoise",
        "ir_dot_brightness", "ir_flood_brightness",
        "show_fps", "show_timestamp", "jpeg_quality",
    }
    changed = False
    with config_lock:
        for key, value in data.items():
            if key in live_keys:
                current_config[key] = value
                changed = True
    if changed:
        controls_dirty.set()
    return jsonify({"ok": True})


@flask_app.route("/api/restart", methods=["POST"])
def api_restart():
    """Update config and restart pipeline."""
    data = request.get_json(force=True)
    restart_keys = {
        "resolution", "fps",
        "enable_depth", "depth_preset", "lr_check",
        "extended_disparity", "subpixel",
        "confidence_threshold", "median_filter",
    }
    with config_lock:
        for key, value in data.items():
            if key in restart_keys:
                current_config[key] = value
    pipeline_restart.set()
    return jsonify({"ok": True, "msg": "Pipeline restarting..."})


@flask_app.route("/api/reset", methods=["POST"])
def api_reset():
    """Reset all config to defaults and restart pipeline."""
    defaults = {
        "resolution": "1080p", "fps": 30,
        "auto_exposure": True, "exposure_us": 8333, "iso": 400,
        "auto_white_balance": True, "white_balance_k": 5500,
        "brightness": 0, "contrast": 0, "saturation": 0,
        "sharpness": 1, "luma_denoise": 1, "chroma_denoise": 1,
        "ir_dot_brightness": 0.0, "ir_flood_brightness": 0.0,
        "enable_depth": True, "depth_preset": "HIGH_DENSITY",
        "lr_check": True, "extended_disparity": False, "subpixel": False,
        "confidence_threshold": 200, "median_filter": "KERNEL_7x7",
        "show_fps": True, "show_timestamp": False, "jpeg_quality": 80,
    }
    with config_lock:
        current_config.update(defaults)
    pipeline_restart.set()
    return jsonify({"ok": True})


@flask_app.route("/api/recording/toggle", methods=["POST"])
def api_recording_toggle():
    """Toggle H.265 recording on/off."""
    with recording_lock:
        if state.recording_active:
            if state.recording_file:
                state.recording_file.close()
                state.recording_file = None
            state.recording_active = False
            return jsonify({"recording": False, "msg": "Recording stopped."})
        else:
            with config_lock:
                rec_dir = current_config["recording_dir"]
            os.makedirs(rec_dir, exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filepath = os.path.join(rec_dir, f"oakd_front_{ts}.h265")
            state.recording_file = open(filepath, "wb")
            state.recording_active = True
            return jsonify({"recording": True, "file": filepath,
                            "msg": f"Recording to {filepath}"})


@flask_app.route("/api/imu")
def api_imu():
    """Return latest IMU data as JSON."""
    with imu_lock:
        data = dict(imu_data)
    return jsonify(data)


@flask_app.route("/api/imu/zero", methods=["POST"])
def api_imu_zero():
    """Set current orientation as the zero reference (per-axis Euler offsets)."""
    with imu_lock:
        state.imu_euler_offsets["pitch"] = imu_data["orientation"]["pitch"]
        state.imu_euler_offsets["roll"]  = imu_data["orientation"]["roll"]
        state.imu_euler_offsets["yaw"]   = imu_data["orientation"]["yaw"]
    return jsonify({"ok": True, "msg": "IMU zeroed."})

@flask_app.route("/api/imu/reset", methods=["POST"])
def api_imu_reset():
    """Clear the zero reference."""
    with imu_lock:
        # Correct — mutates in place
        state.imu_euler_offsets["pitch"] = 0.0
        state.imu_euler_offsets["roll"]  = 0.0
        state.imu_euler_offsets["yaw"]   = 0.0
    return jsonify({"ok": True, "msg": "IMU reference cleared."})


@flask_app.route("/api/position")
def api_position():
    """Return current robot position state (pitch-based classification)."""
    with position_lock:
        data = dict(position_state)
    # Include the configured ranges for the dashboard to display
    data["ranges"] = [
        {"min": lo, "max": hi, "label": lbl}
        for (lo, hi, lbl) in position_ranges
    ]
    return jsonify(data)


# ═══════════════════════════════════════════════════════
# Server entry point
# ═══════════════════════════════════════════════════════

def run_server(port=8080):
    """Run the Flask server (call from a daemon thread)."""
    flask_app.run(host="0.0.0.0", port=port,
                  threaded=True, use_reloader=False)
