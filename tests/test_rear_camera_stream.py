#!/usr/bin/env python3
"""
Rear Camera LAN Streaming Server — RPi Camera V2 NoIR (IMX219)
================================================================
Streams the rear camera feed over the local network via HTTP MJPEG.
Camera settings are controllable live via a web dashboard.

Designed to match the OAK-D Pro streaming server architecture so both
can later be merged into a single unified dashboard.

Usage:
  python3 test_rear_camera_stream.py                    # Default: 720p, 30fps, port 8081
  python3 test_rear_camera_stream.py --port 9090        # Custom port
  python3 test_rear_camera_stream.py --resolution 1080p # Start at 1080p
  python3 test_rear_camera_stream.py --fps 15           # Lower FPS (saves CPU)

Access from operator computer:
  Dashboard:      http://<rpi-ip>:8081/
  Video stream:   http://<rpi-ip>:8081/video/rear
  Snapshot:       http://<rpi-ip>:8081/snapshot

Note: Uses port 8081 by default so it can run alongside OAK-D Pro (8080).

Prerequisites:
  sudo apt install -y python3-picamera2 python3-opencv
  pip install flask --break-system-packages

  # picamera2 comes pre-installed on Raspberry Pi OS (Bookworm)
"""

import argparse
import threading
import subprocess
import time
import json
import sys
import os

try:
    from picamera2 import Picamera2
    import cv2
    import numpy as np
    from flask import Flask, Response, jsonify, request, render_template_string
except ImportError as e:
    print(f"  Missing dependency: {e}")
    print("  Install with:")
    print("    sudo apt install -y python3-picamera2 python3-opencv")
    print("    pip install flask --break-system-packages")
    sys.exit(1)


# ─────────────────────────────────────────────
# Resolution presets (IMX219 native modes)
# ─────────────────────────────────────────────

RESOLUTION_MAP = {
    "480p":  (640, 480),
    "720p":  (1280, 720),
    "1080p": (1920, 1080),
}

# ─────────────────────────────────────────────
# Shared state
# ─────────────────────────────────────────────

frame_lock = threading.Lock()
current_frame = None  # BGR numpy array

camera_lock = threading.Lock()
camera_instance = None  # Picamera2

# Current config
current_config = {
    "resolution": "720p",
    "fps": 30,
    # Image controls
    "brightness": 0.0,       # -1.0 to 1.0
    "contrast": 1.0,         # 0.0 to 2.0
    "saturation": 1.0,       # 0.0 to 2.0
    "sharpness": 1.0,        # 0.0 to 2.0
    # Exposure
    "auto_exposure": True,
    "exposure_time": 33000,  # microseconds (manual mode)
    "analogue_gain": 1.0,    # ISO-like (manual mode)
    # White balance
    "auto_wb": True,
    "wb_red_gain": 1.5,      # manual red gain
    "wb_blue_gain": 1.5,     # manual blue gain
    # Flip / rotate
    "hflip": False,
    "vflip": False,
    # Overlay
    "show_fps": True,
    "show_timestamp": False,
    # JPEG quality for streaming
    "jpeg_quality": 80,
}

config_lock = threading.Lock()

# Pipeline restart
restart_requested = threading.Event()
stop_event = threading.Event()


# ─────────────────────────────────────────────
# Camera pipeline
# ─────────────────────────────────────────────

def apply_controls(cam, cfg):
    """Apply live controls to the running camera."""
    controls = {}

    if cfg["auto_exposure"]:
        controls["AeEnable"] = True
    else:
        controls["AeEnable"] = False
        controls["ExposureTime"] = int(cfg["exposure_time"])
        controls["AnalogueGain"] = float(cfg["analogue_gain"])

    if cfg["auto_wb"]:
        controls["AwbEnable"] = True
    else:
        controls["AwbEnable"] = False
        controls["ColourGains"] = (float(cfg["wb_red_gain"]),
                                   float(cfg["wb_blue_gain"]))

    controls["Brightness"] = float(cfg["brightness"])
    controls["Contrast"] = float(cfg["contrast"])
    controls["Saturation"] = float(cfg["saturation"])
    controls["Sharpness"] = float(cfg["sharpness"])

    try:
        cam.set_controls(controls)
    except Exception as e:
        print(f"  [warn] Failed to set controls: {e}")


def camera_worker():
    """Runs the camera and updates shared frame buffer."""
    global current_frame, camera_instance

    while not stop_event.is_set():
        restart_requested.clear()

        with config_lock:
            cfg = dict(current_config)

        res = RESOLUTION_MAP.get(cfg["resolution"], (1280, 720))
        target_fps = cfg["fps"]

        try:
            cam = Picamera2()

            # Configure
            preview_config = cam.create_preview_configuration(
                main={"size": res, "format": "RGB888"},
                controls={"FrameDurationLimits": (
                    int(1_000_000 / target_fps),
                    int(1_000_000 / target_fps)
                )},
                transform=cam.sensor_resolution  # placeholder, overridden below
            )
            # Handle flip via transform
            from libcamera import Transform
            preview_config["transform"] = Transform(
                hflip=cfg["hflip"],
                vflip=cfg["vflip"]
            )

            cam.configure(preview_config)
            cam.start()

            with camera_lock:
                camera_instance = cam

            # Let auto-exposure settle
            time.sleep(1.5)
            apply_controls(cam, cfg)

            print(f"  Camera started: {cfg['resolution']} "
                  f"({res[0]}x{res[1]}) @ {target_fps}fps")

            fps_counter = 0
            fps_time = time.time()
            fps_display = 0.0

            while not stop_event.is_set() and not restart_requested.is_set():
                frame_rgb = cam.capture_array()
                frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

                # FPS calculation
                fps_counter += 1
                elapsed = time.time() - fps_time
                if elapsed >= 1.0:
                    fps_display = fps_counter / elapsed
                    fps_counter = 0
                    fps_time = time.time()

                # Overlays
                with config_lock:
                    overlay_cfg = dict(current_config)

                if overlay_cfg["show_fps"]:
                    cv2.putText(frame_bgr, f"FPS: {fps_display:.1f}",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                0.8, (0, 255, 0), 2)

                if overlay_cfg["show_timestamp"]:
                    ts = time.strftime("%Y-%m-%d %H:%M:%S")
                    cv2.putText(frame_bgr, ts,
                                (10, res[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (200, 200, 200), 1)

                with frame_lock:
                    current_frame = frame_bgr

                # Check for live control updates (no restart needed)
                with config_lock:
                    live_cfg = dict(current_config)
                apply_controls(cam, live_cfg)

                # Pace the loop
                time.sleep(max(0.001, (1.0 / target_fps) - 0.005))

        except Exception as e:
            print(f"  [error] Camera worker: {e}")
            time.sleep(2)

        finally:
            try:
                with camera_lock:
                    camera_instance = None
                cam.stop()
                cam.close()
                print("  Camera stopped.")
            except Exception:
                pass

    print("  Camera worker exited.")


# ─────────────────────────────────────────────
# MJPEG generator
# ─────────────────────────────────────────────

def mjpeg_stream():
    """Yields MJPEG frames for the /video/rear endpoint."""
    while True:
        with frame_lock:
            frame = current_frame

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


# ─────────────────────────────────────────────
# Flask app
# ─────────────────────────────────────────────

app = Flask(__name__)


@app.after_request
def add_cors_headers(response):
    """Allow the front camera dashboard (port 8080) to access rear camera streams."""
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
    return response


DASHBOARD_HTML = """
<!DOCTYPE html>
<html>
<head>
  <title>Rear Camera — RPi V2 NoIR</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }

    body {
      font-family: 'Courier New', monospace;
      background: #0d1117;
      color: #c9d1d9;
      min-height: 100vh;
    }

    .header {
      background: #161b22;
      border-bottom: 1px solid #30363d;
      padding: 14px 24px;
      display: flex;
      align-items: center;
      justify-content: space-between;
    }
    .header h1 {
      font-size: 16px;
      color: #58a6ff;
      font-weight: normal;
      letter-spacing: 1px;
    }
    .header .badge {
      background: #1f6feb;
      color: #fff;
      padding: 3px 10px;
      border-radius: 12px;
      font-size: 11px;
    }
    #conn-status {
      font-size: 11px;
      color: #3fb950;
    }

    .layout {
      display: flex;
      gap: 0;
      height: calc(100vh - 52px);
    }

    .stream-panel {
      flex: 1;
      display: flex;
      align-items: center;
      justify-content: center;
      background: #010409;
      position: relative;
      overflow: hidden;
    }
    .stream-panel img {
      max-width: 100%;
      max-height: 100%;
      object-fit: contain;
    }
    .stream-label {
      position: absolute;
      top: 10px;
      left: 12px;
      background: rgba(0,0,0,0.7);
      color: #f0883e;
      padding: 4px 10px;
      border-radius: 4px;
      font-size: 12px;
      letter-spacing: 1px;
    }

    .controls-panel {
      width: 340px;
      min-width: 340px;
      background: #161b22;
      border-left: 1px solid #30363d;
      overflow-y: auto;
      padding: 16px;
    }

    .section {
      margin-bottom: 18px;
    }
    .section-title {
      color: #f0883e;
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 2px;
      margin-bottom: 10px;
      padding-bottom: 4px;
      border-bottom: 1px solid #21262d;
    }

    .control-row {
      display: flex;
      align-items: center;
      margin-bottom: 8px;
    }
    .control-row label {
      width: 110px;
      font-size: 12px;
      color: #8b949e;
      flex-shrink: 0;
    }
    .control-row input[type=range] {
      flex: 1;
      accent-color: #58a6ff;
      height: 4px;
    }
    .control-row .val {
      width: 55px;
      text-align: right;
      font-size: 12px;
      color: #58a6ff;
      flex-shrink: 0;
      margin-left: 6px;
    }

    select, button {
      background: #21262d;
      color: #c9d1d9;
      border: 1px solid #30363d;
      padding: 5px 10px;
      border-radius: 6px;
      font-family: inherit;
      font-size: 12px;
      cursor: pointer;
    }
    button:hover { background: #30363d; }
    button.active {
      background: #1f6feb;
      border-color: #1f6feb;
      color: #fff;
    }

    .toggle-row {
      display: flex;
      align-items: center;
      margin-bottom: 8px;
    }
    .toggle-row label {
      width: 110px;
      font-size: 12px;
      color: #8b949e;
    }
    .toggle {
      position: relative;
      width: 40px;
      height: 20px;
    }
    .toggle input {
      opacity: 0; width: 0; height: 0;
    }
    .toggle .slider {
      position: absolute;
      cursor: pointer;
      top: 0; left: 0; right: 0; bottom: 0;
      background: #21262d;
      border-radius: 10px;
      transition: 0.2s;
    }
    .toggle .slider:before {
      content: "";
      position: absolute;
      height: 14px; width: 14px;
      left: 3px; bottom: 3px;
      background: #8b949e;
      border-radius: 50%;
      transition: 0.2s;
    }
    .toggle input:checked + .slider {
      background: #1f6feb;
    }
    .toggle input:checked + .slider:before {
      transform: translateX(20px);
      background: #fff;
    }

    .btn-row {
      display: flex;
      gap: 6px;
      margin-top: 6px;
    }
    .btn-row button {
      flex: 1;
      padding: 8px;
    }

    @media (max-width: 900px) {
      .layout { flex-direction: column; height: auto; }
      .stream-panel { min-height: 300px; }
      .controls-panel { width: 100%; min-width: 0; border-left: none; border-top: 1px solid #30363d; }
    }
  </style>
</head>
<body>
  <div class="header">
    <h1>REAR CAM — RPi V2 NoIR</h1>
    <span id="conn-status">● LIVE</span>
    <span class="badge">PORT {{ port }}</span>
  </div>

  <div class="layout">
    <div class="stream-panel">
      <span class="stream-label">REAR</span>
      <img id="stream" src="/video/rear" alt="Rear Camera Stream" />
    </div>

    <div class="controls-panel">

      <!-- STREAM SETTINGS -->
      <div class="section">
        <div class="section-title">Stream</div>
        <div class="control-row">
          <label>Resolution</label>
          <select id="resolution" onchange="restartPipeline('resolution', this.value)">
            <option value="480p">480p</option>
            <option value="720p" selected>720p</option>
            <option value="1080p">1080p</option>
          </select>
        </div>
        <div class="control-row">
          <label>Target FPS</label>
          <select id="fps" onchange="restartPipeline('fps', parseInt(this.value))">
            <option value="10">10</option>
            <option value="15">15</option>
            <option value="24">24</option>
            <option value="30" selected>30</option>
          </select>
        </div>
        <div class="control-row">
          <label>JPEG Quality</label>
          <input type="range" min="30" max="100" value="80"
                 oninput="setControl('jpeg_quality', parseInt(this.value), this)">
          <span class="val" id="val_jpeg_quality">80</span>
        </div>
      </div>

      <!-- EXPOSURE -->
      <div class="section">
        <div class="section-title">Exposure</div>
        <div class="toggle-row">
          <label>Auto Exposure</label>
          <div class="toggle">
            <input type="checkbox" id="auto_exposure" checked
                   onchange="setControl('auto_exposure', this.checked)">
            <span class="slider"></span>
          </div>
        </div>
        <div class="control-row">
          <label>Exposure (µs)</label>
          <input type="range" min="100" max="120000" value="33000" step="100"
                 oninput="setControl('exposure_time', parseInt(this.value), this)">
          <span class="val" id="val_exposure_time">33000</span>
        </div>
        <div class="control-row">
          <label>Gain (ISO)</label>
          <input type="range" min="1.0" max="16.0" value="1.0" step="0.5"
                 oninput="setControl('analogue_gain', parseFloat(this.value), this)">
          <span class="val" id="val_analogue_gain">1.0</span>
        </div>
      </div>

      <!-- WHITE BALANCE -->
      <div class="section">
        <div class="section-title">White Balance</div>
        <div class="toggle-row">
          <label>Auto WB</label>
          <div class="toggle">
            <input type="checkbox" id="auto_wb" checked
                   onchange="setControl('auto_wb', this.checked)">
            <span class="slider"></span>
          </div>
        </div>
        <div class="control-row">
          <label>Red Gain</label>
          <input type="range" min="0.5" max="4.0" value="1.5" step="0.1"
                 oninput="setControl('wb_red_gain', parseFloat(this.value), this)">
          <span class="val" id="val_wb_red_gain">1.5</span>
        </div>
        <div class="control-row">
          <label>Blue Gain</label>
          <input type="range" min="0.5" max="4.0" value="1.5" step="0.1"
                 oninput="setControl('wb_blue_gain', parseFloat(this.value), this)">
          <span class="val" id="val_wb_blue_gain">1.5</span>
        </div>
      </div>

      <!-- IMAGE -->
      <div class="section">
        <div class="section-title">Image</div>
        <div class="control-row">
          <label>Brightness</label>
          <input type="range" min="-1.0" max="1.0" value="0.0" step="0.05"
                 oninput="setControl('brightness', parseFloat(this.value), this)">
          <span class="val" id="val_brightness">0.0</span>
        </div>
        <div class="control-row">
          <label>Contrast</label>
          <input type="range" min="0.0" max="2.0" value="1.0" step="0.05"
                 oninput="setControl('contrast', parseFloat(this.value), this)">
          <span class="val" id="val_contrast">1.0</span>
        </div>
        <div class="control-row">
          <label>Saturation</label>
          <input type="range" min="0.0" max="2.0" value="1.0" step="0.05"
                 oninput="setControl('saturation', parseFloat(this.value), this)">
          <span class="val" id="val_saturation">1.0</span>
        </div>
        <div class="control-row">
          <label>Sharpness</label>
          <input type="range" min="0.0" max="4.0" value="1.0" step="0.1"
                 oninput="setControl('sharpness', parseFloat(this.value), this)">
          <span class="val" id="val_sharpness">1.0</span>
        </div>
      </div>

      <!-- ORIENTATION -->
      <div class="section">
        <div class="section-title">Orientation</div>
        <div class="toggle-row">
          <label>H-Flip</label>
          <div class="toggle">
            <input type="checkbox" id="hflip"
                   onchange="restartPipeline('hflip', this.checked)">
            <span class="slider"></span>
          </div>
        </div>
        <div class="toggle-row">
          <label>V-Flip</label>
          <div class="toggle">
            <input type="checkbox" id="vflip"
                   onchange="restartPipeline('vflip', this.checked)">
            <span class="slider"></span>
          </div>
        </div>
      </div>

      <!-- OVERLAY -->
      <div class="section">
        <div class="section-title">Overlay</div>
        <div class="toggle-row">
          <label>Show FPS</label>
          <div class="toggle">
            <input type="checkbox" id="show_fps" checked
                   onchange="setControl('show_fps', this.checked)">
            <span class="slider"></span>
          </div>
        </div>
        <div class="toggle-row">
          <label>Timestamp</label>
          <div class="toggle">
            <input type="checkbox" id="show_timestamp"
                   onchange="setControl('show_timestamp', this.checked)">
            <span class="slider"></span>
          </div>
        </div>
      </div>

      <!-- ACTIONS -->
      <div class="section">
        <div class="section-title">Actions</div>
        <div class="btn-row">
          <button onclick="snapshot()">📷 Snapshot</button>
          <button onclick="resetDefaults()">↺ Reset</button>
        </div>
      </div>

    </div>
  </div>

  <script>
    // ── Live controls (no pipeline restart) ──
    function setControl(key, value, el) {
      // Update displayed value
      const valSpan = document.getElementById('val_' + key);
      if (valSpan) valSpan.textContent = typeof value === 'number' ?
        (Number.isInteger(value) ? value : value.toFixed(1)) : value;

      fetch('/api/controls', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({[key]: value})
      });
    }

    // ── Controls that require pipeline restart ──
    function restartPipeline(key, value) {
      fetch('/api/restart', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({[key]: value})
      }).then(() => {
        // Reload stream after short delay for pipeline restart
        setTimeout(() => {
          const img = document.getElementById('stream');
          img.src = '/video/rear?' + Date.now();
        }, 2500);
      });
    }

    // ── Snapshot ──
    function snapshot() {
      window.open('/snapshot', '_blank');
    }

    // ── Reset to defaults ──
    function resetDefaults() {
      fetch('/api/reset', {method: 'POST'}).then(() => {
        location.reload();
      });
    }

    // ── Status polling ──
    function pollStatus() {
      fetch('/status')
        .then(r => r.json())
        .then(data => {
          document.getElementById('conn-status').textContent = '● LIVE';
          document.getElementById('conn-status').style.color = '#3fb950';
        })
        .catch(() => {
          document.getElementById('conn-status').textContent = '● OFFLINE';
          document.getElementById('conn-status').style.color = '#f85149';
        });
    }
    setInterval(pollStatus, 3000);

    // ── Sync UI on load ──
    fetch('/status').then(r => r.json()).then(cfg => {
      const res = document.getElementById('resolution');
      if (res) res.value = cfg.resolution || '720p';
      const fps = document.getElementById('fps');
      if (fps) fps.value = cfg.fps || 30;
    });
  </script>
</body>
</html>
"""


# ─────────────────────────────────────────────
# Routes
# ─────────────────────────────────────────────

@app.route("/")
def dashboard():
    return render_template_string(DASHBOARD_HTML,
                                  port=app.config.get("PORT", 8081))


@app.route("/video/rear")
def video_rear():
    return Response(mjpeg_stream(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/snapshot")
def snapshot():
    """Return a single JPEG frame."""
    with frame_lock:
        frame = current_frame

    if frame is None:
        return "No frame available", 503

    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        return "Encode failed", 500

    return Response(buf.tobytes(), mimetype="image/jpeg",
                    headers={"Content-Disposition":
                             f"inline; filename=rear_snapshot.jpg"})


@app.route("/status")
def status():
    with config_lock:
        cfg = dict(current_config)
    cfg["camera_connected"] = camera_instance is not None
    return jsonify(cfg)


@app.route("/api/controls", methods=["POST"])
def api_controls():
    """Update live controls (no pipeline restart)."""
    data = request.get_json(force=True)

    live_keys = {"brightness", "contrast", "saturation", "sharpness",
                 "auto_exposure", "exposure_time", "analogue_gain",
                 "auto_wb", "wb_red_gain", "wb_blue_gain",
                 "show_fps", "show_timestamp", "jpeg_quality"}

    with config_lock:
        for key, value in data.items():
            if key in live_keys:
                current_config[key] = value

    return jsonify({"ok": True})


@app.route("/api/restart", methods=["POST"])
def api_restart():
    """Update config and restart the camera pipeline."""
    data = request.get_json(force=True)

    restart_keys = {"resolution", "fps", "hflip", "vflip"}

    with config_lock:
        for key, value in data.items():
            if key in restart_keys:
                current_config[key] = value

    restart_requested.set()
    return jsonify({"ok": True, "msg": "Pipeline restarting..."})


@app.route("/api/reset", methods=["POST"])
def api_reset():
    """Reset all config to defaults and restart pipeline."""
    defaults = {
        "resolution": "720p", "fps": 30,
        "brightness": 0.0, "contrast": 1.0,
        "saturation": 1.0, "sharpness": 1.0,
        "auto_exposure": True, "exposure_time": 33000,
        "analogue_gain": 1.0, "auto_wb": True,
        "wb_red_gain": 1.5, "wb_blue_gain": 1.5,
        "hflip": False, "vflip": False,
        "show_fps": True, "show_timestamp": False,
        "jpeg_quality": 80,
    }

    with config_lock:
        current_config.update(defaults)

    restart_requested.set()
    return jsonify({"ok": True})


# ─────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Rear Camera LAN Streaming Server (RPi V2 NoIR)")
    p.add_argument("--fps", type=int, default=30,
                   help="Target FPS (default 30)")
    p.add_argument("--resolution", default="720p",
                   choices=list(RESOLUTION_MAP.keys()),
                   help="Starting resolution (default 720p)")
    p.add_argument("--port", type=int, default=8081,
                   help="HTTP port (default 8081, avoids conflict with OAK-D on 8080)")
    p.add_argument("--host", default="0.0.0.0",
                   help="Bind address (default 0.0.0.0)")
    return p.parse_args()


def main():
    args = parse_args()

    with config_lock:
        current_config["fps"] = args.fps
        current_config["resolution"] = args.resolution

    app.config["PORT"] = args.port

    # Check for CSI camera
    try:
        cam_test = Picamera2()
        cam_info = cam_test.camera_properties
        cam_test.close()
        sensor = cam_info.get("Model", "unknown")
        print(f"  Detected sensor: {sensor}")
    except Exception as e:
        print(f"  No CSI camera detected: {e}")
        print("  Check:")
        print("    1. Ribbon cable seated properly (blue side toward USB ports)")
        print("    2. Camera enabled: sudo raspi-config → Interface → Camera")
        print("    3. Try: libcamera-hello --list-cameras")
        sys.exit(1)

    # Start camera thread
    cam_thread = threading.Thread(target=camera_worker, daemon=True)
    cam_thread.start()
    time.sleep(2)

    # Discover network addresses
    ip_hints = []
    try:
        result = subprocess.run(
            ["ip", "-4", "-o", "addr", "show"],
            capture_output=True, text=True, timeout=5)
        for line in result.stdout.strip().split("\n"):
            parts = line.split()
            if len(parts) >= 4:
                iface = parts[1]
                ip_addr = parts[3].split("/")[0]
                if ip_addr != "127.0.0.1" and not iface.startswith("docker"):
                    ip_hints.append((iface, ip_addr))
    except Exception:
        pass

    if not ip_hints:
        ip_hints = [("unknown", "<rpi-ip>")]

    print(f"\n{'=' * 50}")
    print(f"  Rear Camera Stream Server (RPi V2 NoIR)")
    print(f"{'=' * 50}")
    for iface, ip in ip_hints:
        print(f"  [{iface}] http://{ip}:{args.port}/")
    print(f"{'=' * 50}")
    print(f"  Endpoints:")
    print(f"    /             Dashboard")
    print(f"    /video/rear   MJPEG stream")
    print(f"    /snapshot     Single JPEG frame")
    print(f"    /status       JSON config")
    print(f"    /api/controls POST live controls")
    print(f"    /api/restart  POST pipeline restart")
    print(f"    /api/reset    POST reset to defaults")
    print(f"{'=' * 50}")
    print(f"  {args.resolution} @ {args.fps}fps")
    print(f"  Press Ctrl+C to stop\n")

    try:
        app.run(host=args.host, port=args.port,
                threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        print("\n  Shutting down...")
    finally:
        stop_event.set()
        restart_requested.set()


if __name__ == "__main__":
    main()