#!/usr/bin/env python3
"""
OAK-D Pro Camera Node (ROS2 + Flask Dashboard + H.265 HW Encoding)
=====================================================================
Owns the DepthAI pipeline directly for full parameter control.

Three simultaneous outputs from the same pipeline:
  1. Preview (low-res)  → Flask MJPEG dashboard for operator
  2. Video (high-res)   → Hardware H.265 encoder on Myriad X → disk recording
  3. Stereo depth       → Colorized depth map → dashboard

Dashboard: http://<rpi-ip>:8080/

ROS2 topics (commented out, uncomment for SLAM/autonomy):
  /camera/front/rgb              sensor_msgs/Image
  /camera/front/depth            sensor_msgs/Image
  /camera/front/rgb/compressed   sensor_msgs/CompressedImage

Prerequisites (inside Docker):
  pip install depthai opencv-python flask --break-system-packages

Usage:
  ros2 launch camera_driver oakd.launch.py
  ros2 run camera_driver oakd_node
"""

import threading
import time
import os
import sys
from datetime import datetime

import cv2
import numpy as np

try:
    import depthai as dai
except ImportError:
    print("  [ERROR] depthai not installed.")
    print("  pip install depthai --break-system-packages")
    sys.exit(1)

try:
    from flask import Flask, Response, jsonify, request, render_template_string
except ImportError:
    print("  [ERROR] flask not installed.")
    print("  pip install flask --break-system-packages")
    sys.exit(1)

import rclpy
from rclpy.node import Node as RosNode

# ─────────────────────────────────────────────────────────────────
# [FUTURE] Uncomment when SLAM / autonomy nodes need camera data
# ─────────────────────────────────────────────────────────────────
# from sensor_msgs.msg import Image, CompressedImage
# from cv_bridge import CvBridge


# ═════════════════════════════════════════════════════════════════
# Resolution presets
# ═════════════════════════════════════════════════════════════════

RESOLUTION_MAP = {
    "480p":  (dai.ColorCameraProperties.SensorResolution.THE_800_P,  640,  480),
    "720p":  (dai.ColorCameraProperties.SensorResolution.THE_800_P,  1280, 720),
    "1080p": (dai.ColorCameraProperties.SensorResolution.THE_1080_P, 1920, 1080),
    "4k":    (dai.ColorCameraProperties.SensorResolution.THE_4_K,    3840, 2160),
}

MEDIAN_MAP = {
    "OFF":        dai.MedianFilter.MEDIAN_OFF,
    "KERNEL_3x3": dai.MedianFilter.KERNEL_3x3,
    "KERNEL_5x5": dai.MedianFilter.KERNEL_5x5,
    "KERNEL_7x7": dai.MedianFilter.KERNEL_7x7,
}

DEPTH_PRESET_MAP = {
    "HIGH_DENSITY":  dai.node.StereoDepth.PresetMode.HIGH_DENSITY,
    "HIGH_ACCURACY": dai.node.StereoDepth.PresetMode.HIGH_ACCURACY,
}


# ═════════════════════════════════════════════════════════════════
# Shared state
# ═════════════════════════════════════════════════════════════════

frame_lock = threading.Lock()
current_frames = {
    "rgb": None,
    "depth": None,
}

device_ref = None
ctrl_queue = None

# Recording state
recording_lock = threading.Lock()
recording_active = False
recording_file = None

# Pipeline control
pipeline_stop = threading.Event()
pipeline_restart = threading.Event()


# ═════════════════════════════════════════════════════════════════
# Current config (modified by dashboard, read by pipeline)
# ═════════════════════════════════════════════════════════════════

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
    # Focus
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


# ═════════════════════════════════════════════════════════════════
# DepthAI pipeline builder
# ═════════════════════════════════════════════════════════════════

def build_pipeline(cfg):
    """Build a DepthAI pipeline with RGB + depth + H.265 encoder."""
    pipeline = dai.Pipeline()

    res_key = cfg["resolution"]
    sensor_res, vid_w, vid_h = RESOLUTION_MAP.get(res_key, RESOLUTION_MAP["1080p"])
    fps = cfg["fps"]
    preview_w = cfg["preview_width"]
    preview_h = cfg["preview_height"]

    # ── Color camera ──
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(sensor_res)
    cam_rgb.setVideoSize(vid_w, vid_h)
    cam_rgb.setPreviewSize(preview_w, preview_h)
    cam_rgb.setFps(fps)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    # Preview output → dashboard MJPEG + ROS2 image topic
    xout_preview = pipeline.create(dai.node.XLinkOut)
    xout_preview.setStreamName("preview")
    cam_rgb.preview.link(xout_preview.input)

    # Camera control input
    ctrl_in = pipeline.create(dai.node.XLinkIn)
    ctrl_in.setStreamName("cam_ctrl")
    ctrl_in.out.link(cam_rgb.inputControl)

    # ── Hardware H.265 encoder (on Myriad X, zero CPU cost) ──
    h265_enc = pipeline.create(dai.node.VideoEncoder)
    h265_enc.setDefaultProfilePreset(fps, dai.VideoEncoderProperties.Profile.H265_MAIN)
    if cfg["h265_bitrate_kbps"] > 0:
        h265_enc.setBitrateKbps(cfg["h265_bitrate_kbps"])
    h265_enc.setKeyframeFrequency(cfg["h265_keyframe_interval"])

    cam_rgb.video.link(h265_enc.input)

    xout_h265 = pipeline.create(dai.node.XLinkOut)
    xout_h265.setStreamName("h265")
    h265_enc.bitstream.link(xout_h265.input)

    # ── Stereo depth ──
    if cfg["enable_depth"]:
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_left.setCamera("left")
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setFps(fps)

        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_right.setCamera("right")
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setFps(fps)

        stereo = pipeline.create(dai.node.StereoDepth)
        preset = DEPTH_PRESET_MAP.get(cfg["depth_preset"],
                                      dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDefaultProfilePreset(preset)
        stereo.setLeftRightCheck(cfg["lr_check"])
        stereo.setExtendedDisparity(cfg["extended_disparity"])
        stereo.setSubpixel(cfg["subpixel"])
        stereo.initialConfig.setConfidenceThreshold(cfg["confidence_threshold"])
        median = MEDIAN_MAP.get(cfg["median_filter"], dai.MedianFilter.KERNEL_7x7)
        stereo.initialConfig.setMedianFilter(median)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.disparity.link(xout_depth.input)

    return pipeline


# ═════════════════════════════════════════════════════════════════
# Camera controls (applied live without pipeline restart)
# ═════════════════════════════════════════════════════════════════

def apply_camera_controls(queue, cfg):
    """Send camera controls to the running pipeline."""
    if queue is None:
        return

    ctrl = dai.CameraControl()

    # Exposure
    if cfg["auto_exposure"]:
        ctrl.setAutoExposureEnable()
    else:
        ctrl.setManualExposure(int(cfg["exposure_us"]), int(cfg["iso"]))

    # Focus
    if cfg["auto_focus"]:
        mode_map = {
            "continuous": dai.CameraControl.AutoFocusMode.CONTINUOUS_VIDEO,
            "auto":       dai.CameraControl.AutoFocusMode.AUTO,
            "macro":      dai.CameraControl.AutoFocusMode.MACRO,
        }
        af_mode = mode_map.get(cfg["autofocus_mode"],
                               dai.CameraControl.AutoFocusMode.CONTINUOUS_VIDEO)
        ctrl.setAutoFocusMode(af_mode)
    else:
        ctrl.setManualFocus(int(cfg["manual_focus"]))

    # White balance
    if cfg["auto_white_balance"]:
        ctrl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.AUTO)
    else:
        ctrl.setManualWhiteBalance(int(cfg["white_balance_k"]))

    # Image adjustments
    ctrl.setBrightness(int(cfg["brightness"]))
    ctrl.setContrast(int(cfg["contrast"]))
    ctrl.setSaturation(int(cfg["saturation"]))
    ctrl.setSharpness(int(cfg["sharpness"]))
    ctrl.setLumaDenoise(int(cfg["luma_denoise"]))
    ctrl.setChromaDenoise(int(cfg["chroma_denoise"]))

    try:
        queue.send(ctrl)
    except Exception as e:
        print(f"  [warn] Failed to send camera control: {e}")


def apply_ir_controls(device, cfg):
    """Set IR dot projector and flood light brightness."""
    if device is None:
        return
    try:
        device.setIrLaserDotProjectorBrightness(float(cfg["ir_dot_brightness"]))
        device.setIrFloodLightBrightness(float(cfg["ir_flood_brightness"]))
    except Exception as e:
        print(f"  [warn] IR control failed: {e}")


# ═════════════════════════════════════════════════════════════════
# Pipeline worker thread
# ═════════════════════════════════════════════════════════════════

def pipeline_worker(ros_node):
    """Runs the DepthAI pipeline and updates shared frame buffers."""
    global device_ref, ctrl_queue, recording_active, recording_file

    # ─────────────────────────────────────────────
    # [FUTURE] Uncomment for ROS2 image publishing
    # ─────────────────────────────────────────────
    # bridge = CvBridge()
    # rgb_pub = ros_node.create_publisher(Image, '/camera/front/rgb', 10)
    # depth_pub = ros_node.create_publisher(Image, '/camera/front/depth', 10)
    # compressed_pub = ros_node.create_publisher(
    #     CompressedImage, '/camera/front/rgb/compressed', 10)

    while not pipeline_stop.is_set():
        pipeline_restart.clear()

        with config_lock:
            cfg = dict(current_config)

        ros_node.get_logger().info(
            f"Starting pipeline: {cfg['resolution']} @ {cfg['fps']}fps")

        try:
            pipeline = build_pipeline(cfg)

            with dai.Device(pipeline) as device:
                device_ref = device

                # Camera control queue
                ctrl_queue = device.getInputQueue("cam_ctrl")

                # Output queues
                q_preview = device.getOutputQueue("preview", maxSize=4, blocking=False)
                q_h265 = device.getOutputQueue("h265", maxSize=30, blocking=False)
                q_depth = None
                if cfg["enable_depth"]:
                    q_depth = device.getOutputQueue("depth", maxSize=4, blocking=False)

                # Apply initial controls
                time.sleep(0.5)
                apply_camera_controls(ctrl_queue, cfg)
                apply_ir_controls(device, cfg)

                # FPS tracking
                fps_counter = 0
                fps_time = time.time()
                fps_display = 0.0

                # Depth colormap scaling
                max_disparity = 95  # default for 400p mono
                if cfg["extended_disparity"]:
                    max_disparity *= 2
                if cfg["subpixel"]:
                    max_disparity *= 32  # subpixel is 5 bits

                ros_node.get_logger().info("Pipeline running. Dashboard ready.")

                while (not pipeline_stop.is_set()
                       and not pipeline_restart.is_set()):

                    # ── Preview frame (dashboard + ROS2) ──
                    preview_pkt = q_preview.tryGet()
                    if preview_pkt is not None:
                        frame_bgr = preview_pkt.getCvFrame()

                        # FPS
                        fps_counter += 1
                        elapsed = time.time() - fps_time
                        if elapsed >= 1.0:
                            fps_display = fps_counter / elapsed
                            fps_counter = 0
                            fps_time = time.time()

                        # Overlays
                        with config_lock:
                            ov = dict(current_config)

                        if ov["show_fps"]:
                            cv2.putText(frame_bgr, f"FPS: {fps_display:.1f}",
                                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.7, (0, 255, 0), 2)

                        if ov["show_timestamp"]:
                            ts = time.strftime("%Y-%m-%d %H:%M:%S")
                            h = frame_bgr.shape[0]
                            cv2.putText(frame_bgr, ts,
                                        (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.5, (200, 200, 200), 1)

                        with frame_lock:
                            current_frames["rgb"] = frame_bgr

                        # ─────────────────────────────────────
                        # [FUTURE] Publish to ROS2 image topics
                        # ─────────────────────────────────────
                        # if rgb_pub.get_subscription_count() > 0:
                        #     msg = bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
                        #     msg.header.stamp = ros_node.get_clock().now().to_msg()
                        #     msg.header.frame_id = "oakd_rgb_optical_frame"
                        #     rgb_pub.publish(msg)
                        #
                        # if compressed_pub.get_subscription_count() > 0:
                        #     comp_msg = CompressedImage()
                        #     comp_msg.header.stamp = ros_node.get_clock().now().to_msg()
                        #     comp_msg.header.frame_id = "oakd_rgb_optical_frame"
                        #     comp_msg.format = "jpeg"
                        #     _, buf = cv2.imencode('.jpg', frame_bgr,
                        #                          [cv2.IMWRITE_JPEG_QUALITY, 80])
                        #     comp_msg.data = buf.tobytes()
                        #     compressed_pub.publish(comp_msg)

                    # ── Depth frame ──
                    if q_depth is not None:
                        depth_pkt = q_depth.tryGet()
                        if depth_pkt is not None:
                            disp = depth_pkt.getFrame()
                            disp_norm = (disp * 255.0 / max_disparity).astype(np.uint8)
                            disp_color = cv2.applyColorMap(disp_norm,
                                                           cv2.COLORMAP_JET)
                            with frame_lock:
                                current_frames["depth"] = disp_color

                            # ─────────────────────────────────
                            # [FUTURE] Publish depth to ROS2
                            # ─────────────────────────────────
                            # if depth_pub.get_subscription_count() > 0:
                            #     depth_msg = bridge.cv2_to_imgmsg(
                            #         disp, encoding="mono16")
                            #     depth_msg.header.stamp = (
                            #         ros_node.get_clock().now().to_msg())
                            #     depth_msg.header.frame_id = (
                            #         "oakd_stereo_optical_frame")
                            #     depth_pub.publish(depth_msg)

                    # ── H.265 encoded stream ──
                    h265_pkt = q_h265.tryGet()
                    if h265_pkt is not None:
                        with recording_lock:
                            if recording_active and recording_file:
                                recording_file.write(h265_pkt.getData())

                    # Live control updates (no restart needed)
                    with config_lock:
                        live_cfg = dict(current_config)
                    apply_camera_controls(ctrl_queue, live_cfg)
                    apply_ir_controls(device, live_cfg)

                    time.sleep(0.001)

        except Exception as e:
            ros_node.get_logger().error(f"Pipeline error: {e}")
            time.sleep(2)

        finally:
            device_ref = None
            ctrl_queue = None
            with recording_lock:
                if recording_file:
                    recording_file.close()
                    recording_file = None
                    recording_active = False
            ros_node.get_logger().info("Pipeline stopped.")

    ros_node.get_logger().info("Pipeline worker exited.")


# ═════════════════════════════════════════════════════════════════
# MJPEG generators
# ═════════════════════════════════════════════════════════════════

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
                # Resize depth to match RGB height
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


# ═════════════════════════════════════════════════════════════════
# Dashboard HTML
# ═════════════════════════════════════════════════════════════════

DASHBOARD_HTML = """
<!DOCTYPE html>
<html>
<head>
  <title>OAK-D Pro — Front Camera</title>
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
    .header h1 { font-size: 16px; color: #58a6ff; font-weight: normal; letter-spacing: 1px; }
    .header .badge { background: #1f6feb; color: #fff; padding: 3px 10px; border-radius: 12px; font-size: 11px; }
    #conn-status { font-size: 11px; color: #3fb950; }

    .layout { display: flex; gap: 0; height: calc(100vh - 52px); }

    .stream-panel {
      flex: 1; display: flex; align-items: center; justify-content: center;
      background: #010409; position: relative; overflow: hidden;
    }
    .stream-panel img { max-width: 100%; max-height: 100%; object-fit: contain; }
    .stream-label {
      position: absolute; top: 10px; left: 12px;
      background: rgba(0,0,0,0.7); color: #f0883e;
      padding: 4px 10px; border-radius: 4px; font-size: 12px; letter-spacing: 1px;
    }

    .controls-panel {
      width: 360px; min-width: 360px; background: #161b22;
      border-left: 1px solid #30363d; overflow-y: auto; padding: 16px;
    }

    .section { margin-bottom: 18px; }
    .section-title {
      color: #f0883e; font-size: 12px; text-transform: uppercase;
      letter-spacing: 2px; margin-bottom: 10px; padding-bottom: 4px;
      border-bottom: 1px solid #21262d;
    }

    .control-row { display: flex; align-items: center; margin-bottom: 8px; }
    .control-row label { width: 130px; font-size: 12px; color: #8b949e; flex-shrink: 0; }
    .control-row input[type=range] { flex: 1; accent-color: #58a6ff; height: 4px; }
    .control-row .val { width: 55px; text-align: right; font-size: 12px; color: #58a6ff; flex-shrink: 0; margin-left: 6px; }

    select, button {
      background: #21262d; color: #c9d1d9; border: 1px solid #30363d;
      padding: 5px 10px; border-radius: 6px; font-family: inherit; font-size: 12px; cursor: pointer;
    }
    button:hover { background: #30363d; }
    button.active { background: #1f6feb; border-color: #1f6feb; color: #fff; }
    button.rec { background: #da3633; border-color: #da3633; color: #fff; }
    button.rec:hover { background: #f85149; }

    .toggle-row { display: flex; align-items: center; margin-bottom: 8px; }
    .toggle-row label { width: 130px; font-size: 12px; color: #8b949e; }
    .toggle { position: relative; width: 40px; height: 20px; }
    .toggle input { opacity: 0; width: 0; height: 0; }
    .toggle .slider {
      position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0;
      background: #21262d; border-radius: 10px; transition: 0.2s;
    }
    .toggle .slider:before {
      content: ""; position: absolute; height: 14px; width: 14px;
      left: 3px; bottom: 3px; background: #8b949e; border-radius: 50%; transition: 0.2s;
    }
    .toggle input:checked + .slider { background: #1f6feb; }
    .toggle input:checked + .slider:before { transform: translateX(20px); background: #fff; }

    .btn-row { display: flex; gap: 6px; margin-top: 6px; }
    .btn-row button { flex: 1; padding: 8px; }

    #rec-indicator { display: none; color: #da3633; font-size: 11px; }
    #rec-indicator.active { display: inline; }

    @media (max-width: 900px) {
      .layout { flex-direction: column; height: auto; }
      .stream-panel { min-height: 300px; }
      .controls-panel { width: 100%; min-width: 0; border-left: none; border-top: 1px solid #30363d; }
    }
  </style>
</head>
<body>
  <div class="header">
    <h1>OAK-D PRO — FRONT CAM</h1>
    <span id="conn-status">● LIVE</span>
    <span id="rec-indicator">⏺ REC</span>
    <span class="badge">PORT {{ port }}</span>
  </div>

  <div class="layout">
    <div class="stream-panel">
      <span class="stream-label">FRONT — RGB + DEPTH</span>
      <img id="stream" src="/video/both" alt="OAK-D Stream" />
    </div>

    <div class="controls-panel">

      <!-- STREAM -->
      <div class="section">
        <div class="section-title">Stream</div>
        <div class="control-row">
          <label>View</label>
          <select id="stream_mode" onchange="switchStream(this.value)">
            <option value="both" selected>RGB + Depth</option>
            <option value="rgb">RGB Only</option>
            <option value="depth">Depth Only</option>
          </select>
        </div>
        <div class="control-row">
          <label>Resolution</label>
          <select id="resolution" onchange="restartPipeline('resolution', this.value)">
            <option value="480p">480p</option>
            <option value="720p">720p</option>
            <option value="1080p" selected>1080p</option>
            <option value="4k">4K</option>
          </select>
        </div>
        <div class="control-row">
          <label>FPS</label>
          <select id="fps" onchange="restartPipeline('fps', parseInt(this.value))">
            <option value="10">10</option>
            <option value="15">15</option>
            <option value="24">24</option>
            <option value="30" selected>30</option>
            <option value="60">60</option>
          </select>
        </div>
        <div class="control-row">
          <label>JPEG Quality</label>
          <input type="range" min="30" max="100" value="80"
                 oninput="setControl('jpeg_quality', parseInt(this.value), this)">
          <span class="val" id="val_jpeg_quality">80</span>
        </div>
      </div>

      <!-- RECORDING -->
      <div class="section">
        <div class="section-title">H.265 Recording</div>
        <div class="btn-row">
          <button id="rec-btn" onclick="toggleRecording()">⏺ Start Recording</button>
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
          <input type="range" min="1" max="33000" value="8333" step="100"
                 oninput="setControl('exposure_us', parseInt(this.value), this)">
          <span class="val" id="val_exposure_us">8333</span>
        </div>
        <div class="control-row">
          <label>ISO</label>
          <input type="range" min="100" max="1600" value="400" step="50"
                 oninput="setControl('iso', parseInt(this.value), this)">
          <span class="val" id="val_iso">400</span>
        </div>
      </div>

      <!-- FOCUS -->
      <div class="section">
        <div class="section-title">Focus</div>
        <div class="toggle-row">
          <label>Auto Focus</label>
          <div class="toggle">
            <input type="checkbox" id="auto_focus" checked
                   onchange="setControl('auto_focus', this.checked)">
            <span class="slider"></span>
          </div>
        </div>
        <div class="control-row">
          <label>AF Mode</label>
          <select onchange="setControl('autofocus_mode', this.value)">
            <option value="continuous" selected>Continuous</option>
            <option value="auto">Auto (trigger)</option>
            <option value="macro">Macro</option>
          </select>
        </div>
        <div class="control-row">
          <label>Manual Focus</label>
          <input type="range" min="0" max="255" value="127"
                 oninput="setControl('manual_focus', parseInt(this.value), this)">
          <span class="val" id="val_manual_focus">127</span>
        </div>
      </div>

      <!-- WHITE BALANCE -->
      <div class="section">
        <div class="section-title">White Balance</div>
        <div class="toggle-row">
          <label>Auto WB</label>
          <div class="toggle">
            <input type="checkbox" id="auto_white_balance" checked
                   onchange="setControl('auto_white_balance', this.checked)">
            <span class="slider"></span>
          </div>
        </div>
        <div class="control-row">
          <label>Color Temp (K)</label>
          <input type="range" min="1000" max="12000" value="5500" step="100"
                 oninput="setControl('white_balance_k', parseInt(this.value), this)">
          <span class="val" id="val_white_balance_k">5500</span>
        </div>
      </div>

      <!-- IMAGE -->
      <div class="section">
        <div class="section-title">Image</div>
        <div class="control-row">
          <label>Brightness</label>
          <input type="range" min="-10" max="10" value="0"
                 oninput="setControl('brightness', parseInt(this.value), this)">
          <span class="val" id="val_brightness">0</span>
        </div>
        <div class="control-row">
          <label>Contrast</label>
          <input type="range" min="-10" max="10" value="0"
                 oninput="setControl('contrast', parseInt(this.value), this)">
          <span class="val" id="val_contrast">0</span>
        </div>
        <div class="control-row">
          <label>Saturation</label>
          <input type="range" min="-10" max="10" value="0"
                 oninput="setControl('saturation', parseInt(this.value), this)">
          <span class="val" id="val_saturation">0</span>
        </div>
        <div class="control-row">
          <label>Sharpness</label>
          <input type="range" min="0" max="4" value="1"
                 oninput="setControl('sharpness', parseInt(this.value), this)">
          <span class="val" id="val_sharpness">1</span>
        </div>
        <div class="control-row">
          <label>Luma Denoise</label>
          <input type="range" min="0" max="4" value="1"
                 oninput="setControl('luma_denoise', parseInt(this.value), this)">
          <span class="val" id="val_luma_denoise">1</span>
        </div>
        <div class="control-row">
          <label>Chroma Denoise</label>
          <input type="range" min="0" max="4" value="1"
                 oninput="setControl('chroma_denoise', parseInt(this.value), this)">
          <span class="val" id="val_chroma_denoise">1</span>
        </div>
      </div>

      <!-- IR -->
      <div class="section">
        <div class="section-title">IR Illumination</div>
        <div class="control-row">
          <label>Dot Projector</label>
          <input type="range" min="0" max="1" value="0" step="0.01"
                 oninput="setControl('ir_dot_brightness', parseFloat(this.value), this)">
          <span class="val" id="val_ir_dot_brightness">0.0</span>
        </div>
        <div class="control-row">
          <label>Flood Light</label>
          <input type="range" min="0" max="1" value="0" step="0.01"
                 oninput="setControl('ir_flood_brightness', parseFloat(this.value), this)">
          <span class="val" id="val_ir_flood_brightness">0.0</span>
        </div>
      </div>

      <!-- DEPTH -->
      <div class="section">
        <div class="section-title">Stereo Depth</div>
        <div class="control-row">
          <label>Confidence</label>
          <input type="range" min="0" max="255" value="200"
                 oninput="restartPipeline('confidence_threshold', parseInt(this.value)); this.nextElementSibling.textContent=this.value">
          <span class="val">200</span>
        </div>
        <div class="control-row">
          <label>Median Filter</label>
          <select onchange="restartPipeline('median_filter', this.value)">
            <option value="OFF">Off</option>
            <option value="KERNEL_3x3">3×3</option>
            <option value="KERNEL_5x5">5×5</option>
            <option value="KERNEL_7x7" selected>7×7</option>
          </select>
        </div>
        <div class="toggle-row">
          <label>LR Check</label>
          <div class="toggle">
            <input type="checkbox" checked
                   onchange="restartPipeline('lr_check', this.checked)">
            <span class="slider"></span>
          </div>
        </div>
        <div class="toggle-row">
          <label>Ext. Disparity</label>
          <div class="toggle">
            <input type="checkbox"
                   onchange="restartPipeline('extended_disparity', this.checked)">
            <span class="slider"></span>
          </div>
        </div>
        <div class="toggle-row">
          <label>Subpixel</label>
          <div class="toggle">
            <input type="checkbox"
                   onchange="restartPipeline('subpixel', this.checked)">
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
    function setControl(key, value, el) {
      const valSpan = document.getElementById('val_' + key);
      if (valSpan) valSpan.textContent = typeof value === 'number' ?
        (Number.isInteger(value) ? value : value.toFixed(2)) : value;

      fetch('/api/controls', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({[key]: value})
      });
    }

    function restartPipeline(key, value) {
      fetch('/api/restart', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({[key]: value})
      }).then(() => {
        setTimeout(() => {
          const img = document.getElementById('stream');
          const mode = document.getElementById('stream_mode').value;
          img.src = '/video/' + mode + '?' + Date.now();
        }, 3000);
      });
    }

    function switchStream(mode) {
      const img = document.getElementById('stream');
      img.src = '/video/' + mode + '?' + Date.now();
    }

    function snapshot() { window.open('/snapshot', '_blank'); }

    function resetDefaults() {
      fetch('/api/reset', {method: 'POST'}).then(() => location.reload());
    }

    function toggleRecording() {
      fetch('/api/recording/toggle', {method: 'POST'})
        .then(r => r.json())
        .then(data => {
          const btn = document.getElementById('rec-btn');
          const ind = document.getElementById('rec-indicator');
          if (data.recording) {
            btn.textContent = '⏹ Stop Recording';
            btn.classList.add('rec');
            ind.classList.add('active');
          } else {
            btn.textContent = '⏺ Start Recording';
            btn.classList.remove('rec');
            ind.classList.remove('active');
          }
        });
    }

    function pollStatus() {
      fetch('/status').then(r => r.json()).then(data => {
        document.getElementById('conn-status').textContent = '● LIVE';
        document.getElementById('conn-status').style.color = '#3fb950';
      }).catch(() => {
        document.getElementById('conn-status').textContent = '● OFFLINE';
        document.getElementById('conn-status').style.color = '#f85149';
      });
    }
    setInterval(pollStatus, 3000);

    fetch('/status').then(r => r.json()).then(cfg => {
      const res = document.getElementById('resolution');
      if (res) res.value = cfg.resolution || '1080p';
      const fps = document.getElementById('fps');
      if (fps) fps.value = cfg.fps || 30;
    });
  </script>
</body>
</html>
"""


# ═════════════════════════════════════════════════════════════════
# Flask app
# ═════════════════════════════════════════════════════════════════

flask_app = Flask(__name__)


@flask_app.route("/")
def dashboard():
    return render_template_string(DASHBOARD_HTML,
                                  port=flask_app.config.get("PORT", 8080))


@flask_app.route("/video/rgb")
def video_rgb():
    return Response(mjpeg_gen("rgb"),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@flask_app.route("/video/depth")
def video_depth():
    return Response(mjpeg_gen("depth"),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@flask_app.route("/video/both")
def video_both():
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
        cfg["recording"] = recording_active
    cfg["device_connected"] = device_ref is not None
    return jsonify(cfg)


@flask_app.route("/api/controls", methods=["POST"])
def api_controls():
    """Update live controls (no pipeline restart needed)."""
    data = request.get_json(force=True)
    live_keys = {
        "auto_exposure", "exposure_us", "iso",
        "auto_focus", "autofocus_mode", "manual_focus",
        "auto_white_balance", "white_balance_k",
        "brightness", "contrast", "saturation", "sharpness",
        "luma_denoise", "chroma_denoise",
        "ir_dot_brightness", "ir_flood_brightness",
        "show_fps", "show_timestamp", "jpeg_quality",
    }
    with config_lock:
        for key, value in data.items():
            if key in live_keys:
                current_config[key] = value
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
        "auto_focus": True, "autofocus_mode": "continuous", "manual_focus": 127,
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
    global recording_active, recording_file

    with recording_lock:
        if recording_active:
            # Stop recording
            if recording_file:
                recording_file.close()
                recording_file = None
            recording_active = False
            return jsonify({"recording": False, "msg": "Recording stopped."})
        else:
            # Start recording
            with config_lock:
                rec_dir = current_config["recording_dir"]
            os.makedirs(rec_dir, exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filepath = os.path.join(rec_dir, f"oakd_front_{ts}.h265")
            recording_file = open(filepath, "wb")
            recording_active = True
            return jsonify({"recording": True, "file": filepath,
                            "msg": f"Recording to {filepath}"})


# ═════════════════════════════════════════════════════════════════
# ROS2 Node
# ═════════════════════════════════════════════════════════════════

class OakDNode(RosNode):
    def __init__(self):
        super().__init__('oakd_node')

        # Load parameters from YAML → current_config
        self._declare_and_load_params()

        self.get_logger().info("OAK-D Pro node initializing...")

        # Check for device
        devices = dai.Device.getAllAvailableDevices()
        if not devices:
            self.get_logger().error(
                "No OAK-D devices found! Check USB. "
                "Run test_oakd_connection.py for diagnostics.")
            return

        self.get_logger().info(f"Found {len(devices)} OAK device(s)")

        # Start pipeline thread
        self._pipeline_thread = threading.Thread(
            target=pipeline_worker, args=(self,), daemon=True)
        self._pipeline_thread.start()

        # Start Flask dashboard thread
        with config_lock:
            port = current_config["dashboard_port"]
        flask_app.config["PORT"] = port

        self._flask_thread = threading.Thread(
            target=self._run_flask, args=(port,), daemon=True)
        self._flask_thread.start()

        # Print access info
        self._print_endpoints(port)

    def _declare_and_load_params(self):
        """Declare ROS2 parameters and sync to current_config."""
        param_defaults = {
            "resolution": "1080p",
            "fps": 30,
            "preview_width": 640,
            "preview_height": 360,
            "dashboard_port": 8080,
            "jpeg_quality": 80,
            "enable_h265_recording": False,
            "h265_bitrate_kbps": 3000,
            "h265_keyframe_interval": 30,
            "recording_dir": "/home/admin/recordings",
            "auto_exposure": True,
            "exposure_us": 8333,
            "iso": 400,
            "auto_focus": True,
            "autofocus_mode": "continuous",
            "manual_focus": 127,
            "auto_white_balance": True,
            "white_balance_k": 5500,
            "brightness": 0,
            "contrast": 0,
            "saturation": 0,
            "sharpness": 1,
            "luma_denoise": 1,
            "chroma_denoise": 1,
            "ir_dot_brightness": 0.0,
            "ir_flood_brightness": 0.0,
            "enable_depth": True,
            "depth_preset": "HIGH_DENSITY",
            "lr_check": True,
            "extended_disparity": False,
            "subpixel": False,
            "confidence_threshold": 200,
            "median_filter": "KERNEL_7x7",
            "show_fps": True,
            "show_timestamp": False,
        }

        with config_lock:
            for key, default in param_defaults.items():
                self.declare_parameter(key, default)
                value = self.get_parameter(key).value
                current_config[key] = value

    def _run_flask(self, port):
        """Run Flask in a background thread."""
        flask_app.run(host="0.0.0.0", port=port,
                      threaded=True, use_reloader=False)

    def _print_endpoints(self, port):
        """Print network addresses and endpoints."""
        import subprocess
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
                    if (ip_addr != "127.0.0.1"
                            and not iface.startswith("docker")):
                        ip_hints.append((iface, ip_addr))
        except Exception:
            pass

        if not ip_hints:
            ip_hints = [("unknown", "<rpi-ip>")]

        self.get_logger().info("=" * 50)
        self.get_logger().info("  OAK-D Pro Camera Node")
        self.get_logger().info("=" * 50)
        for iface, ip in ip_hints:
            self.get_logger().info(f"  [{iface}] http://{ip}:{port}/")
        self.get_logger().info("=" * 50)
        self.get_logger().info("  /video/rgb   /video/depth   /video/both")
        self.get_logger().info("  /snapshot    /status        /api/controls")
        self.get_logger().info("  /api/recording/toggle")
        self.get_logger().info("=" * 50)


# ═════════════════════════════════════════════════════════════════
# Entry point
# ═════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = OakDNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        pipeline_stop.set()
        pipeline_restart.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()