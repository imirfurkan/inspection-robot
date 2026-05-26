#!/usr/bin/env python3
"""
OAK-D Pro LAN Streaming Server
================================
Streams OAK-D Pro camera feeds over the local network via HTTP MJPEG.
All camera settings are controllable live via a web dashboard.

Adapted from: https://github.com/nihatguness/camera_config
Original authors: nihatguness and contributors
Modified for RPi5 (removed Jetson-specific setup)

Usage:
  python3 test_oakd_pro.py                  # Default: 30fps, 1080p, port 8080
  python3 test_oakd_pro.py --fps 60         # 60 FPS
  python3 test_oakd_pro.py --port 9090      # Custom port
  python3 test_oakd_pro.py --resolution 720p  # Start at 720p

Access from operator computer:
  Dashboard:    http://<rpi-ip>:8080/
  RGB stream:   http://<rpi-ip>:8080/video/rgb
  Depth stream: http://<rpi-ip>:8080/video/depth
  Both:         http://<rpi-ip>:8080/video/both

Prerequisites:
  pip install depthai opencv-python flask --break-system-packages
"""

import argparse
import threading
import time
import sys
from pathlib import Path

import cv2
import numpy as np
import depthai as dai
from flask import Flask, Response, jsonify, request, render_template_string

# ─────────────────────────────────────────────
# Global shared state (thread-safe with locks)
# ─────────────────────────────────────────────

frame_lock = threading.Lock()
current_frames = {
    "rgb": None,
    "depth": None,
}

pipeline_lock = threading.Lock()
pipeline_instance = None
pipeline_thread = None

# Sensor capabilities (populated at first pipeline start)
sensor_caps = []
sensor_caps_lock = threading.Lock()

pipeline_stop_event = threading.Event()

# Camera control queue (set after pipeline starts)
ctrl_queue = None
device_ref = None

# Current config state (reflected in dashboard)
current_config = {
    "resolution": "1080p",
    "fps": 30,
    "show_rgb": True,
    "show_depth": True,
    "stream_mode": "both",
    # Camera controls
    "auto_exposure": True,
    "exposure": 8333,
    "iso": 400,
    "auto_focus": True,
    "focus": 127,
    "autofocus_mode": "continuous",
    "white_balance_auto": True,
    "white_balance": 5500,
    "brightness": 0,
    "contrast": 0,
    "saturation": 0,
    "sharpness": 1,
    "luma_denoise": 1,
    "chroma_denoise": 1,
    "ir_brightness": 0.0,
    "flood_brightness": 0.0,
    # Depth settings
    "extended_disparity": False,
    "subpixel": False,
    "lr_check": True,
    "confidence_threshold": 200,
    "median_filter": "7x7",
    "min_depth": 500,
    "max_depth": 15000,
    "depth_preset": "accuracy",
}

RESOLUTION_MAP = {
    "4k":    (3840, 2160),
    "2k":    (2024, 1520),
    "1080p": (1920, 1080),
    "720p":  (1280, 720),
    "480p":  (640, 480),
}

FPS_ABS_MAX = 60
FPS_ABS_MIN = 1

MONO_RESOLUTION_MAP = {
    "4k":    dai.MonoCameraProperties.SensorResolution.THE_800_P,
    "2k":    dai.MonoCameraProperties.SensorResolution.THE_800_P,
    "1080p": dai.MonoCameraProperties.SensorResolution.THE_800_P,
    "720p":  dai.MonoCameraProperties.SensorResolution.THE_720_P,
    "480p":  dai.MonoCameraProperties.SensorResolution.THE_480_P,
}

MEDIAN_MAP = {
    "off": dai.MedianFilter.MEDIAN_OFF,
    "3x3": dai.MedianFilter.KERNEL_3x3,
    "5x5": dai.MedianFilter.KERNEL_5x5,
    "7x7": dai.MedianFilter.KERNEL_7x7,
}

config_lock = threading.Lock()
pipeline_restart_requested = threading.Event()


# ─────────────────────────────────────────────
# DepthAI Pipeline
# ─────────────────────────────────────────────

def colorize_depth(depth_data, min_d, max_d):
    """Convert raw depth to colorized BGR image."""
    clipped = np.clip(depth_data, min_d, max_d)
    normalized = ((clipped - min_d) / (max_d - min_d) * 255).astype(np.uint8)
    return cv2.applyColorMap(normalized, cv2.COLORMAP_JET)


def apply_camera_controls(ctrl_q, cfg):
    """Send a CameraControl message with current config values."""
    if ctrl_q is None:
        return

    ctrl = dai.CameraControl()

    if cfg["auto_exposure"]:
        ctrl.setAutoExposureEnable()
    else:
        ctrl.setManualExposure(cfg["exposure"], cfg["iso"])

    if cfg["auto_focus"] and cfg["autofocus_mode"] != "off":
        af_map = {
            "auto":       dai.CameraControl.AutoFocusMode.AUTO,
            "macro":      dai.CameraControl.AutoFocusMode.MACRO,
            "continuous": dai.CameraControl.AutoFocusMode.CONTINUOUS_VIDEO,
            "edof":       dai.CameraControl.AutoFocusMode.EDOF,
        }
        mode = af_map.get(cfg["autofocus_mode"],
                          dai.CameraControl.AutoFocusMode.CONTINUOUS_VIDEO)
        ctrl.setAutoFocusMode(mode)
    else:
        ctrl.setManualFocus(cfg["focus"])

    if cfg["white_balance_auto"]:
        ctrl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.AUTO)
    else:
        ctrl.setManualWhiteBalance(cfg["white_balance"])

    ctrl.setBrightness(cfg["brightness"])
    ctrl.setContrast(cfg["contrast"])
    ctrl.setSaturation(cfg["saturation"])
    ctrl.setSharpness(cfg["sharpness"])
    ctrl.setLumaDenoise(cfg["luma_denoise"])
    ctrl.setChromaDenoise(cfg["chroma_denoise"])

    try:
        ctrl_q.send(ctrl)
    except Exception as e:
        print(f"[ctrl] Failed to send control: {e}")


def apply_ir_controls(dev, cfg):
    """Apply IR projector and flood light settings."""
    if dev is None:
        return
    try:
        dev.setIrLaserDotProjectorIntensity(cfg["ir_brightness"])
        dev.setIrFloodLightIntensity(cfg["flood_brightness"])
    except Exception:
        pass


def _log_sensor_caps(caps):
    print("[pipeline] Supported sensor configurations:")
    for c in caps:
        print(f"[pipeline]   {c['width']}x{c['height']} "
              f"@ {c['minFps']:.1f} - {c['maxFps']:.1f} fps")


def _find_max_fps_for_resolution(caps, width, height):
    for c in caps:
        if c["width"] >= width and c["height"] >= height:
            return c["minFps"], c["maxFps"]
    return None


def run_pipeline(cfg_snapshot):
    """Run the DepthAI pipeline with the given config. Blocks until stop requested."""
    global ctrl_queue, device_ref, sensor_caps

    resolution = cfg_snapshot["resolution"]
    fps = cfg_snapshot["fps"]
    show_rgb = cfg_snapshot["show_rgb"]
    show_depth = cfg_snapshot["show_depth"]
    rgb_size = RESOLUTION_MAP.get(resolution, (1920, 1080))

    fps = max(FPS_ABS_MIN, min(FPS_ABS_MAX, fps))

    try:
        with dai.Pipeline() as pipeline:
            queues = {}

            # Query sensor capabilities
            try:
                dev = pipeline.getDefaultDevice()
                if dev is not None:
                    features = dev.getConnectedCameraFeatures()
                    caps = []
                    for feat in features:
                        for conf in getattr(feat, 'configs', []):
                            caps.append({
                                "width": conf.width,
                                "height": conf.height,
                                "minFps": conf.minFps,
                                "maxFps": conf.maxFps,
                            })
                    if caps:
                        with sensor_caps_lock:
                            sensor_caps = caps
            except Exception as e:
                print(f"[pipeline] Could not query sensor capabilities: {e}")

            # ── RGB Camera ──
            if show_rgb:
                cam = pipeline.create(dai.node.Camera)
                try:
                    cam = cam.build(sensorResolution=rgb_size,
                                    sensorFps=float(fps))
                except RuntimeError as e:
                    print(f"[pipeline] {resolution} ({rgb_size[0]}x{rgb_size[1]}) "
                          f"@ {fps}fps not supported: {e}")
                    with sensor_caps_lock:
                        local_caps = list(sensor_caps)
                    if local_caps:
                        _log_sensor_caps(local_caps)
                        fps_range = _find_max_fps_for_resolution(
                            local_caps, rgb_size[0], rgb_size[1])
                        if fps_range:
                            clamped_fps = max(fps_range[0],
                                              min(fps_range[1], fps))
                            print(f"[pipeline] Retrying {resolution} "
                                  f"@ {clamped_fps:.0f}fps")
                            cam = pipeline.create(dai.node.Camera)
                            try:
                                cam = cam.build(
                                    sensorResolution=rgb_size,
                                    sensorFps=float(clamped_fps))
                                fps = int(clamped_fps)
                            except RuntimeError:
                                print("[pipeline] Falling back to 1080p @ 30fps")
                                cam = pipeline.create(dai.node.Camera)
                                cam = cam.build(sensorResolution=(1920, 1080),
                                                sensorFps=30.0)
                                rgb_size = (1920, 1080)
                                fps = 30
                        else:
                            print("[pipeline] Falling back to 1080p @ 30fps")
                            cam = pipeline.create(dai.node.Camera)
                            cam = cam.build(sensorResolution=(1920, 1080),
                                            sensorFps=30.0)
                            rgb_size = (1920, 1080)
                            fps = 30
                    else:
                        print("[pipeline] Falling back to 1080p @ 30fps")
                        cam = pipeline.create(dai.node.Camera)
                        cam = cam.build(sensorResolution=(1920, 1080),
                                        sensorFps=30.0)
                        rgb_size = (1920, 1080)
                        fps = 30

                preview_size = (min(rgb_size[0], 1920), min(rgb_size[1], 1080))
                rgb_out = cam.requestOutput(
                    preview_size,
                    dai.ImgFrame.Type.BGR888p,
                    fps=fps
                )
                queues["rgb"] = rgb_out.createOutputQueue()
                ctrl_in = cam.inputControl.createInputQueue()
            else:
                ctrl_in = None

            # ── Depth ──
            if show_depth:
                depth_fps = max(FPS_ABS_MIN, min(42, fps))
                stereo = pipeline.create(dai.node.StereoDepth)
                stereo = stereo.build(autoCreateCameras=True,
                                       fps=float(depth_fps))
                stereo.setExtendedDisparity(cfg_snapshot["extended_disparity"])
                stereo.setSubpixel(cfg_snapshot["subpixel"])
                stereo.setLeftRightCheck(cfg_snapshot["lr_check"])
                stereo.initialConfig.setConfidenceThreshold(
                    cfg_snapshot["confidence_threshold"])
                stereo.initialConfig.setMedianFilter(
                    MEDIAN_MAP.get(cfg_snapshot["median_filter"],
                                   dai.MedianFilter.KERNEL_7x7))

                if cfg_snapshot["depth_preset"] == "accuracy":
                    stereo.setDefaultProfilePreset(
                        dai.node.StereoDepth.PresetMode.ACCURACY)
                else:
                    stereo.setDefaultProfilePreset(
                        dai.node.StereoDepth.PresetMode.DENSITY)

                queues["depth"] = stereo.depth.createOutputQueue()

            pipeline.start()

            ctrl_queue = ctrl_in

            try:
                device_ref = pipeline.getDefaultDevice()
            except Exception:
                device_ref = None

            if ctrl_in:
                time.sleep(0.5)
                apply_camera_controls(ctrl_in, cfg_snapshot)
                apply_ir_controls(device_ref, cfg_snapshot)

            print(f"[pipeline] Started — {resolution} @ {fps}fps | "
                  f"RGB:{show_rgb} Depth:{show_depth}")

            while pipeline.isRunning() and not pipeline_stop_event.is_set():
                if "rgb" in queues:
                    f = queues["rgb"].tryGet()
                    if f is not None:
                        with frame_lock:
                            current_frames["rgb"] = f.getCvFrame()

                if "depth" in queues:
                    f = queues["depth"].tryGet()
                    if f is not None:
                        depth_data = f.getFrame()
                        colored = colorize_depth(
                            depth_data,
                            cfg_snapshot["min_depth"],
                            cfg_snapshot["max_depth"])
                        with frame_lock:
                            current_frames["depth"] = colored

                if pipeline_restart_requested.is_set():
                    print("[pipeline] Restart requested...")
                    break

                time.sleep(0.001)

    except Exception as e:
        print(f"[pipeline] Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        ctrl_queue = None
        device_ref = None
        print("[pipeline] Stopped.")


def pipeline_manager():
    """Manages the pipeline lifecycle — restarts when config changes require it."""
    while not pipeline_stop_event.is_set():
        pipeline_restart_requested.clear()

        with config_lock:
            cfg_snapshot = dict(current_config)

        run_pipeline(cfg_snapshot)

        if pipeline_stop_event.is_set():
            break

        if not pipeline_restart_requested.is_set():
            time.sleep(2)  # pipeline crashed, wait before retry
        else:
            time.sleep(0.3)  # brief pause before restart


# ─────────────────────────────────────────────
# MJPEG Stream Generators
# ─────────────────────────────────────────────

def generate_rgb():
    while True:
        with frame_lock:
            frame = current_frames.get("rgb")
        if frame is not None:
            ok, buf = cv2.imencode(".jpg", frame,
                                    [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                       + buf.tobytes() + b"\r\n")
        time.sleep(0.03)


def generate_depth():
    while True:
        with frame_lock:
            frame = current_frames.get("depth")
        if frame is not None:
            ok, buf = cv2.imencode(".jpg", frame,
                                    [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                       + buf.tobytes() + b"\r\n")
        time.sleep(0.03)


def generate_both():
    while True:
        with frame_lock:
            rgb = current_frames.get("rgb")
            depth = current_frames.get("depth")
        if rgb is not None and depth is not None:
            h = min(rgb.shape[0], depth.shape[0])
            rgb_r = cv2.resize(rgb, (int(rgb.shape[1] * h / rgb.shape[0]), h))
            depth_r = cv2.resize(depth,
                                  (int(depth.shape[1] * h / depth.shape[0]), h))
            combined = np.hstack([rgb_r, depth_r])
            ok, buf = cv2.imencode(".jpg", combined,
                                    [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                       + buf.tobytes() + b"\r\n")
        elif rgb is not None:
            ok, buf = cv2.imencode(".jpg", rgb,
                                    [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                       + buf.tobytes() + b"\r\n")
        time.sleep(0.03)


# ─────────────────────────────────────────────
# Flask App
# ─────────────────────────────────────────────

app = Flask(__name__)

DASHBOARD_HTML = """
<!DOCTYPE html>
<html>
<head>
  <title>OAK-D Pro Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: monospace; background: #1a1a2e; color: #eee; margin: 20px; }
    h1 { color: #0f0; }
    .stream { margin: 10px 0; }
    .stream img { max-width: 100%; border: 1px solid #333; }
    .controls { background: #16213e; padding: 15px; border-radius: 8px;
                margin: 10px 0; }
    .control-group { margin: 8px 0; }
    label { display: inline-block; width: 180px; }
    input[type=range] { width: 200px; vertical-align: middle; }
    select, button { background: #0a1128; color: #eee; border: 1px solid #555;
                     padding: 5px 10px; border-radius: 4px; cursor: pointer; }
    button { background: #0f3460; }
    button:hover { background: #16a085; }
    .value { display: inline-block; width: 60px; text-align: right; }
    .section-title { color: #e94560; margin-top: 15px; font-weight: bold; }
    #status { color: #0f0; }
  </style>
</head>
<body>
  <h1>OAK-D Pro Dashboard</h1>
  <p id="status">Connecting...</p>

  <div class="controls">
    <div class="section-title">Stream</div>
    <div class="control-group">
      <label>Mode:</label>
      <select id="stream_mode" onchange="setStreamMode(this.value)">
        <option value="both">RGB + Depth</option>
        <option value="rgb">RGB Only</option>
        <option value="depth">Depth Only</option>
      </select>
      <label>Resolution:</label>
      <select id="resolution" onchange="updateConfig('resolution', this.value)">
        <option value="480p">480p</option>
        <option value="720p">720p</option>
        <option value="1080p" selected>1080p</option>
        <option value="2k">2K</option>
        <option value="4k">4K</option>
      </select>
      <label>FPS:</label>
      <input type="number" id="fps" value="30" min="1" max="60" style="width:50px"
             onchange="updateConfig('fps', parseInt(this.value))">
    </div>

    <div class="section-title">IR (OAK-D Pro only)</div>
    <div class="control-group">
      <label>Dot Projector:</label>
      <input type="range" id="ir_brightness" min="0" max="1" step="0.05" value="0"
             oninput="updateConfig('ir_brightness', parseFloat(this.value))">
      <span class="value" id="ir_brightness_val">0</span>
    </div>
    <div class="control-group">
      <label>Flood Light:</label>
      <input type="range" id="flood_brightness" min="0" max="1" step="0.05" value="0"
             oninput="updateConfig('flood_brightness', parseFloat(this.value))">
      <span class="value" id="flood_brightness_val">0</span>
    </div>

    <div class="section-title">Camera</div>
    <div class="control-group">
      <label>Auto Exposure:</label>
      <input type="checkbox" id="auto_exposure" checked
             onchange="updateConfig('auto_exposure', this.checked)">
    </div>
    <div class="control-group">
      <label>Auto Focus:</label>
      <input type="checkbox" id="auto_focus" checked
             onchange="updateConfig('auto_focus', this.checked)">
    </div>
    <div class="control-group">
      <label>Brightness:</label>
      <input type="range" id="brightness" min="-10" max="10" value="0"
             oninput="updateConfig('brightness', parseInt(this.value))">
      <span class="value" id="brightness_val">0</span>
    </div>
    <div class="control-group">
      <label>Contrast:</label>
      <input type="range" id="contrast" min="-10" max="10" value="0"
             oninput="updateConfig('contrast', parseInt(this.value))">
      <span class="value" id="contrast_val">0</span>
    </div>

    <div class="section-title">Depth</div>
    <div class="control-group">
      <label>Confidence:</label>
      <input type="range" id="confidence_threshold" min="0" max="255" value="200"
             oninput="updateConfig('confidence_threshold', parseInt(this.value))">
      <span class="value" id="confidence_threshold_val">200</span>
    </div>
    <div class="control-group">
      <label>Median Filter:</label>
      <select id="median_filter"
              onchange="updateConfig('median_filter', this.value)">
        <option value="off">Off</option>
        <option value="3x3">3x3</option>
        <option value="5x5">5x5</option>
        <option value="7x7" selected>7x7</option>
      </select>
    </div>

    <div style="margin-top:15px">
      <button onclick="resetControls()">Reset All</button>
      <button onclick="takeSnapshot()">Snapshot</button>
    </div>
  </div>

  <div class="stream">
    <img id="stream_img" src="/video/both">
  </div>

  <script>
    function setStreamMode(mode) {
      document.getElementById('stream_img').src = '/video/' + mode;
      updateConfig('stream_mode', mode);
    }

    function updateConfig(key, value) {
      // Update displayed value
      var valEl = document.getElementById(key + '_val');
      if (valEl) valEl.textContent = value;

      var data = {};
      data[key] = value;
      fetch('/api/controls', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(data)
      }).then(r => r.json()).then(d => {
        if (d.needs_restart) {
          document.getElementById('status').textContent = 'Restarting pipeline...';
          setTimeout(() => {
            document.getElementById('status').textContent = 'Connected';
            // Refresh stream
            var img = document.getElementById('stream_img');
            var src = img.src;
            img.src = '';
            setTimeout(() => img.src = src, 1000);
          }, 3000);
        }
      });
    }

    function resetControls() {
      fetch('/api/controls/reset', {method: 'POST'})
        .then(r => r.json())
        .then(d => location.reload());
    }

    function takeSnapshot() {
      window.open('/snapshot', '_blank');
    }

    // Poll status
    setInterval(() => {
      fetch('/status').then(r => r.json()).then(d => {
        var s = d.connected ? 'Connected' : 'Disconnected';
        if (d.rgb_active) s += ' | RGB';
        if (d.depth_active) s += ' | Depth';
        document.getElementById('status').textContent = s;
      }).catch(() => {
        document.getElementById('status').textContent = 'Connection lost';
      });
    }, 3000);
  </script>
</body>
</html>
"""


@app.route("/video/rgb")
def stream_rgb():
    return Response(generate_rgb(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/video/depth")
def stream_depth():
    return Response(generate_depth(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/video/both")
def stream_both():
    return Response(generate_both(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/snapshot")
def snapshot():
    with frame_lock:
        rgb = current_frames.get("rgb")
        depth = current_frames.get("depth")
    frame = rgb if rgb is not None else depth
    if frame is None:
        return "No frame available", 503
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        return "Encode failed", 500
    return Response(buf.tobytes(), mimetype="image/jpeg",
                    headers={"Content-Disposition":
                             "attachment; filename=snapshot.jpg"})

@app.route("/status")
def status():
    with config_lock:
        cfg = dict(current_config)
    with frame_lock:
        has_rgb = current_frames["rgb"] is not None
        has_depth = current_frames["depth"] is not None
    return jsonify({
        "connected": has_rgb or has_depth,
        "rgb_active": has_rgb,
        "depth_active": has_depth,
        "config": cfg,
    })

@app.route("/api/controls", methods=["GET"])
def get_controls():
    with config_lock:
        return jsonify(dict(current_config))


_PIPELINE_RESTART_KEYS = {
    "resolution", "fps", "show_rgb", "show_depth",
    "extended_disparity", "subpixel", "lr_check",
    "confidence_threshold", "median_filter", "depth_preset",
}

_INT_KEYS = {
    "fps", "exposure", "iso", "focus", "white_balance",
    "brightness", "contrast", "saturation", "sharpness",
    "luma_denoise", "chroma_denoise", "min_depth", "max_depth",
    "confidence_threshold",
}

_FLOAT_KEYS = {"ir_brightness", "flood_brightness"}

_BOOL_KEYS = {
    "show_rgb", "show_depth", "auto_exposure", "auto_focus",
    "white_balance_auto", "extended_disparity", "subpixel", "lr_check",
}


@app.route("/api/controls", methods=["POST"])
def set_controls():
    global current_config
    data = request.get_json(force=True)
    if not data:
        return jsonify({"error": "No JSON body"}), 400

    needs_restart = False

    with config_lock:
        for key, val in data.items():
            if key not in current_config:
                continue

            if key in _INT_KEYS:
                val = int(val)
            elif key in _FLOAT_KEYS:
                val = float(val)
            elif key in _BOOL_KEYS:
                if isinstance(val, str):
                    val = val.lower() in ("true", "1", "yes")
                else:
                    val = bool(val)

            if key == "resolution" and val not in RESOLUTION_MAP:
                continue

            if key == "fps":
                val = max(FPS_ABS_MIN, min(FPS_ABS_MAX, val))

            current_config[key] = val

            if key in _PIPELINE_RESTART_KEYS:
                needs_restart = True

        cfg_snapshot = dict(current_config)

    if needs_restart:
        pipeline_restart_requested.set()
    else:
        apply_camera_controls(ctrl_queue, cfg_snapshot)
        apply_ir_controls(device_ref, cfg_snapshot)

    return jsonify({"ok": True, "needs_restart": needs_restart,
                    "config": cfg_snapshot})


@app.route("/api/controls/reset", methods=["POST"])
def reset_controls():
    with config_lock:
        current_config["auto_exposure"] = True
        current_config["auto_focus"] = True
        current_config["autofocus_mode"] = "continuous"
        current_config["white_balance_auto"] = True
        current_config["brightness"] = 0
        current_config["contrast"] = 0
        current_config["saturation"] = 0
        current_config["sharpness"] = 1
        current_config["luma_denoise"] = 1
        current_config["chroma_denoise"] = 1
        current_config["ir_brightness"] = 0.0
        current_config["flood_brightness"] = 0.0
        cfg_snapshot = dict(current_config)

    apply_camera_controls(ctrl_queue, cfg_snapshot)
    apply_ir_controls(device_ref, cfg_snapshot)
    return jsonify({"ok": True, "config": cfg_snapshot})


@app.route("/")
def dashboard():
    return render_template_string(DASHBOARD_HTML)


# ─────────────────────────────────────────────
# Entry Point
# ─────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="OAK-D Pro LAN Streaming Server")
    p.add_argument("--fps", type=int, default=30,
                   help="Starting FPS (default 30)")
    p.add_argument("--resolution", default="1080p",
                   choices=list(RESOLUTION_MAP.keys()),
                   help="Starting resolution")
    p.add_argument("--port", type=int, default=8080,
                   help="HTTP port (default 8080)")
    p.add_argument("--host", default="0.0.0.0",
                   help="Host to bind (default 0.0.0.0)")
    p.add_argument("--mode", default="both",
                   choices=["rgb", "depth", "both"],
                   help="Starting stream mode")
    return p.parse_args()


def main():
    args = parse_args()

    current_config["fps"] = args.fps
    current_config["resolution"] = args.resolution
    current_config["show_rgb"] = args.mode in ("rgb", "both")
    current_config["show_depth"] = args.mode in ("depth", "both")
    current_config["stream_mode"] = args.mode

    # Check OAK device
    devices = dai.Device.getAllAvailableDevices()
    if not devices:
        print("  No OAK-D devices found! Check USB connection.")
        print("  Run test_oakd_connection.py for diagnostics.")
        sys.exit(1)

    print(f"  Found {len(devices)} OAK device(s)")

    # Start pipeline manager
    t = threading.Thread(target=pipeline_manager, daemon=True)
    t.start()
    time.sleep(2)

    # Discover network addresses
    ip_hints = []
    try:
        import subprocess
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
    print(f"  OAK-D Pro Stream Server")
    print(f"{'=' * 50}")
    for iface, ip in ip_hints:
        print(f"  [{iface}] http://{ip}:{args.port}/")
    print(f"{'=' * 50}")
    print(f"  Endpoints: /video/rgb  /video/depth  /video/both")
    print(f"             /snapshot   /status  /api/controls")
    print(f"{'=' * 50}")
    print(f"  Mode: {args.mode} | {args.resolution} @ {args.fps}fps")
    print(f"  Press Ctrl+C to stop\n")

    try:
        app.run(host=args.host, port=args.port,
                threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        print("\n  Shutting down...")
    finally:
        pipeline_stop_event.set()
        pipeline_restart_requested.set()


if __name__ == "__main__":
    main()
