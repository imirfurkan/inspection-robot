#!/usr/bin/env python3
"""
OAK-D Pro Camera Node (ROS2 + DepthAI Pipeline + H.265 HW Encoding)
=====================================================================
Owns the DepthAI pipeline directly for full parameter control.
The Flask dashboard runs in dashboard/server.py.

Three simultaneous outputs from the same pipeline:
  1. Preview (low-res)  → shared frame buffer → dashboard MJPEG
  2. Video (high-res)   → hardware H.265 encoder → disk recording
  3. Stereo depth       → colorized depth map → dashboard

Position classifier:
  Uses IMU pitch to publish the robot's orientation state on /robot_position.
  Thresholds and labels are configurable via ROS2 parameters.

Usage:
  ros2 launch camera_driver oakd.launch.py
  ros2 run camera_driver oakd_node
"""

import threading
import time
import subprocess
import sys


import cv2
import numpy as np

try:
    import depthai as dai
except ImportError:
    print("  [ERROR] depthai not installed.")
    print("  pip install depthai --break-system-packages")
    sys.exit(1)

import rclpy
from rclpy.node import Node as RosNode
from std_msgs.msg import String, Empty, Float32MultiArray

from camera_driver.shared_state import (
    frame_lock, current_frames,
    config_lock, current_config,
    imu_lock, imu_data, imu_euler_offsets,
    recording_lock,
    pipeline_stop, pipeline_restart, controls_dirty,
    position_lock, position_state, position_ranges,
    motor_status_lock,
)
import camera_driver.shared_state as state

from camera_driver.dashboard.server import run_server

# ─────────────────────────────────────────────────────────────────
# [FUTURE] Uncomment when SLAM / autonomy nodes need camera data
# ─────────────────────────────────────────────────────────────────
# from sensor_msgs.msg import Image, CompressedImage
# from cv_bridge import CvBridge


# ═══════════════════════════════════════════════════════
# Resolution presets
# ═══════════════════════════════════════════════════════

# IMX378 sensor supports: 1080_P, 4_K, 12_MP natively.
# For lower resolutions, we set 1080p sensor and scale via setVideoSize/setPreviewSize.
RESOLUTION_MAP = {
    "480p":  (dai.ColorCameraProperties.SensorResolution.THE_1080_P, 854,  480),
    "720p":  (dai.ColorCameraProperties.SensorResolution.THE_1080_P, 1280, 720),
    "1080p": (dai.ColorCameraProperties.SensorResolution.THE_1080_P, 1920, 1080),
}

MEDIAN_MAP = {
    "OFF":        dai.MedianFilter.MEDIAN_OFF,
    "KERNEL_3x3": dai.MedianFilter.KERNEL_3x3,
    "KERNEL_5x5": dai.MedianFilter.KERNEL_5x5,
    "KERNEL_7x7": dai.MedianFilter.KERNEL_7x7,
}

# depthai v2 uses HIGH_DENSITY/HIGH_ACCURACY, v3 uses FAST_DENSITY/FAST_ACCURACY
_pm = dai.node.StereoDepth.PresetMode
if hasattr(_pm, "HIGH_DENSITY"):
    DEPTH_PRESET_MAP = {
        "HIGH_DENSITY":  _pm.HIGH_DENSITY,
        "HIGH_ACCURACY": _pm.HIGH_ACCURACY,
    }
    _DEFAULT_DEPTH_PRESET = _pm.HIGH_DENSITY
elif hasattr(_pm, "FAST_DENSITY"):
    DEPTH_PRESET_MAP = {
        "HIGH_DENSITY":  _pm.FAST_DENSITY,
        "HIGH_ACCURACY": _pm.FAST_ACCURACY,
    }
    _DEFAULT_DEPTH_PRESET = _pm.FAST_DENSITY
else:
    DEPTH_PRESET_MAP = {}
    _DEFAULT_DEPTH_PRESET = list(_pm.__members__.values())[0]


# ═══════════════════════════════════════════════════════
# DepthAI pipeline builder
# ═══════════════════════════════════════════════════════

def build_pipeline(cfg):
    """Build a DepthAI pipeline with RGB + depth + H.265 encoder."""
    pipeline = dai.Pipeline()
    queues = {}

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

    queues["preview"] = cam_rgb.preview.createOutputQueue()
    queues["cam_ctrl"] = cam_rgb.inputControl.createInputQueue()

    # ── Hardware H.265 encoder ──
    h265_enc = pipeline.create(dai.node.VideoEncoder)
    h265_enc.setDefaultProfilePreset(fps, dai.VideoEncoderProperties.Profile.H265_MAIN)
    if cfg["h265_bitrate_kbps"] > 0:
        h265_enc.setBitrateKbps(cfg["h265_bitrate_kbps"])
    h265_enc.setKeyframeFrequency(cfg["h265_keyframe_interval"])

    cam_rgb.video.link(h265_enc.input)
    queues["h265"] = h265_enc.bitstream.createOutputQueue()

    # ── Stereo depth ──
    if cfg["enable_depth"]:
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setFps(fps)

        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setFps(fps)

        stereo = pipeline.create(dai.node.StereoDepth)
        preset = DEPTH_PRESET_MAP.get(cfg["depth_preset"], _DEFAULT_DEPTH_PRESET)
        stereo.setDefaultProfilePreset(preset)
        stereo.setLeftRightCheck(cfg["lr_check"])
        stereo.setExtendedDisparity(cfg["extended_disparity"])
        stereo.setSubpixel(cfg["subpixel"])
        stereo.initialConfig.setConfidenceThreshold(cfg["confidence_threshold"])
        median = MEDIAN_MAP.get(cfg["median_filter"], dai.MedianFilter.KERNEL_7x7)
        stereo.initialConfig.setMedianFilter(median)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        queues["depth"] = stereo.disparity.createOutputQueue()

    # ── IMU (BNO085: 9-axis with on-chip sensor fusion) ──
    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW,
                         dai.IMUSensor.GYROSCOPE_RAW,
                         dai.IMUSensor.ROTATION_VECTOR], 100) # was: ROTATION_VECTOR
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)
    queues["imu"] = imu.out.createOutputQueue()

    return pipeline, queues


# ═══════════════════════════════════════════════════════
# Camera controls
# ═══════════════════════════════════════════════════════

def apply_camera_controls(queue, cfg):
    """Send camera controls to the running pipeline."""
    if queue is None:
        return

    try:
        # Exposure
        ctrl = dai.CameraControl()
        if cfg["auto_exposure"]:
            ctrl.setAutoExposureEnable()
        else:
            ctrl.setManualExposure(int(cfg["exposure_us"]), int(cfg["iso"]))
        queue.send(ctrl)

        # White balance
        ctrl = dai.CameraControl()
        if cfg["auto_white_balance"]:
            ctrl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.AUTO)
        else:
            ctrl.setManualWhiteBalance(int(cfg["white_balance_k"]))
        queue.send(ctrl)

        # Image adjustments
        ctrl = dai.CameraControl()
        ctrl.setBrightness(int(cfg["brightness"]))
        ctrl.setContrast(int(cfg["contrast"]))
        ctrl.setSaturation(int(cfg["saturation"]))
        ctrl.setSharpness(int(cfg["sharpness"]))
        ctrl.setLumaDenoise(int(cfg["luma_denoise"]))
        ctrl.setChromaDenoise(int(cfg["chroma_denoise"]))
        queue.send(ctrl)

    except Exception as e:
        print(f"  [warn] Failed to send camera control: {e}")


_ir_warned = False

def apply_ir_controls(device, cfg):
    """Set IR dot projector and flood light brightness."""
    global _ir_warned
    if device is None:
        return

    dot = float(cfg["ir_dot_brightness"])
    flood = float(cfg["ir_flood_brightness"])

    if dot == 0.0 and flood == 0.0:
        return

    try:
        dot_ma = int(dot * 765)
        flood_ma = int(flood * 1500)

        if hasattr(device, 'setIrLaserDotProjectorBrightness'):
            device.setIrLaserDotProjectorBrightness(dot_ma)
            device.setIrFloodLightBrightness(flood_ma)
        elif hasattr(device, 'setIrLaserDotProjectorIntensity'):
            device.setIrLaserDotProjectorIntensity(dot)
            device.setIrFloodLightIntensity(flood)
        else:
            if not _ir_warned:
                print("  [warn] IR control methods not found. IR controls disabled.")
                _ir_warned = True
    except Exception as e:
        if not _ir_warned:
            print(f"  [warn] IR control failed: {e}")
            _ir_warned = True


# ═══════════════════════════════════════════════════════
# Position classifier
# ═══════════════════════════════════════════════════════

def classify_position(pitch_deg):
    """Classify absolute pitch into a named position state.

    Uses the position_ranges table from shared_state.
    Returns the label string, or 'unknown' if no range matches.
    """
    abs_pitch = abs(pitch_deg)
    for (min_deg, max_deg, label) in position_ranges:
        if min_deg <= abs_pitch < max_deg:
            return label
    return "unknown"


# ═══════════════════════════════════════════════════════
# Pipeline worker thread
# ═══════════════════════════════════════════════════════

def pipeline_worker(ros_node):
    """Runs the DepthAI pipeline and updates shared frame buffers."""

    # Position publisher (std_msgs/String on /robot_position)
    position_pub = ros_node.create_publisher(String, '/robot_position', 10)
    _last_position_label = ""

    while not pipeline_stop.is_set():
        pipeline_restart.clear()

        with config_lock:
            cfg = dict(current_config)

        ros_node.get_logger().info(
            f"Starting pipeline: {cfg['resolution']} @ {cfg['fps']}fps")

        pipeline = None
        try:
            pipeline, queues = build_pipeline(cfg)
            pipeline.start()

            # Get device reference for IR controls
            if hasattr(pipeline, 'getDefaultDevice'):
                state.device_ref = pipeline.getDefaultDevice()
            elif hasattr(pipeline, 'getDevices'):
                devs = pipeline.getDevices()
                state.device_ref = devs[0] if devs else None
            else:
                state.device_ref = None
                ros_node.get_logger().warn(
                    "Could not get device ref. IR controls may not work.")

            q_preview = queues["preview"]
            q_h265 = queues["h265"]
            state.ctrl_queue = queues["cam_ctrl"]
            q_depth = queues.get("depth")
            q_imu = queues.get("imu")

            # Apply initial controls
            time.sleep(0.5)
            apply_camera_controls(state.ctrl_queue, cfg)
            apply_ir_controls(state.device_ref, cfg)

            # FPS tracking
            fps_counter = 0
            fps_time = time.time()
            fps_display = 0.0

            # Depth colormap scaling
            max_disparity = 95
            if cfg["extended_disparity"]:
                max_disparity *= 2
            if cfg["subpixel"]:
                max_disparity *= 32

            ros_node.get_logger().info("Pipeline running. Dashboard ready.")

            while (not pipeline_stop.is_set()
                   and not pipeline_restart.is_set()
                   and pipeline.isRunning()):

                # ── Preview frame ──
                preview_pkt = q_preview.tryGet()
                if preview_pkt is not None:
                    frame_bgr = preview_pkt.getCvFrame()

                    fps_counter += 1
                    elapsed = time.time() - fps_time
                    if elapsed >= 1.0:
                        fps_display = fps_counter / elapsed
                        fps_counter = 0
                        fps_time = time.time()

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

                # ── Depth frame ──
                if q_depth is not None:
                    depth_pkt = q_depth.tryGet()
                    if depth_pkt is not None:
                        disp = depth_pkt.getFrame()
                        disp_norm = (disp * 255.0 / max_disparity).astype(np.uint8)
                        disp_color = cv2.applyColorMap(disp_norm, cv2.COLORMAP_JET)
                        with frame_lock:
                            current_frames["depth"] = disp_color

                # ── H.265 encoded stream ──
                h265_pkt = q_h265.tryGet()
                if h265_pkt is not None:
                    with recording_lock:
                        if state.recording_active and state.recording_file:
                            state.recording_file.write(h265_pkt.getData())

                # ── IMU data ──
                if q_imu is not None:
                    imu_pkt = q_imu.tryGet()
                    if imu_pkt is not None:
                        imu_packets = imu_pkt.packets
                        if imu_packets:
                            p = imu_packets[-1]
                            # print([a for a in dir(p) if 'ot' in a.lower() or 'vec' in a.lower()])

                            a = p.acceleroMeter
                            g = p.gyroscope
                            rv = p.rotationVector

                            with imu_lock:
                                imu_data["accel"]["x"] = round(a.x, 3)
                                imu_data["accel"]["y"] = round(a.y, 3)
                                imu_data["accel"]["z"] = round(a.z, 3)
                                imu_data["gyro"]["x"] = round(g.x, 3)
                                imu_data["gyro"]["y"] = round(g.y, 3)
                                imu_data["gyro"]["z"] = round(g.z, 3)
                                imu_data["rotation"]["i"] = round(rv.i, 5)
                                imu_data["rotation"]["j"] = round(rv.j, 5)
                                imu_data["rotation"]["k"] = round(rv.k, 5)
                                imu_data["rotation"]["real"] = round(rv.real, 5)

                                # Compute orientation relative to reference
                                qi, qj, qk, qr = rv.i, rv.j, rv.k, rv.real

                                # if state.imu_ref_set:
                                #     ri = -state.imu_ref_quat["i"]
                                #     rj = -state.imu_ref_quat["j"]
                                #     rk = -state.imu_ref_quat["k"]
                                #     rr = state.imu_ref_quat["real"]
                                #     qi2 = rr*qi + ri*qr + rj*qk - rk*qj
                                #     qj2 = rr*qj - ri*qk + rj*qr + rk*qi
                                #     qk2 = rr*qk + ri*qj - rj*qi + rk*qr
                                #     qr2 = rr*qr - ri*qi - rj*qj - rk*qk
                                #     qi, qj, qk, qr = qi2, qj2, qk2, qr2

                                # Euler from quaternion (ZYX)
                                import math
                                sinr = 2 * (qr * qi + qj * qk)
                                cosr = 1 - 2 * (qi * qi + qj * qj)
                                # sensor_roll = math.atan2(sinr, cosr) * 57.2958
                                _sr = math.atan2(sinr, cosr) * -57.2958
                                # Fold atan2 [-180,180] → [-90,90] (robot pitch can't exceed ±90°)
                                sensor_roll = 180.0 - _sr if _sr > 90.0 else (-180.0 - _sr if _sr < -90.0 else _sr)


                                sinp = 2 * (qr * qj - qk * qi)
                                sensor_pitch = (math.copysign(90, sinp) if abs(sinp) >= 1
                                                else math.asin(sinp) * 57.2958)

                                siny = 2 * (qr * qk + qi * qj)
                                cosy = 1 - 2 * (qj * qj + qk * qk)
                                sensor_yaw = math.atan2(siny, cosy) * 57.2958

                                off = state.imu_euler_offsets

                                # ── Auto-zero on first valid IMU packet ──
                                if state.imu_auto_zero_pending:
                                    off["pitch"] = sensor_roll
                                    off["roll"]  = sensor_pitch
                                    off["yaw"]   = sensor_yaw
                                    state.imu_auto_zero_pending = False
                                    ros_node.get_logger().info("IMU auto-zeroed at startup.")

                                # Swap pitch/roll (sensor axes don't match robot axes)
                                imu_data["orientation"]["pitch"] = round(sensor_roll  - off["pitch"], 1)
                                imu_data["orientation"]["roll"]  = round(sensor_pitch - off["roll"],  1)
                                imu_data["orientation"]["yaw"]   = round(sensor_yaw   - off["yaw"],   1)

                                imu_data["timestamp"] = time.time()

                            # ── Position classification ──
                            current_pitch = imu_data["orientation"]["pitch"]
                            label = classify_position(current_pitch)
                            now = time.time()

                            with position_lock:
                                position_state["label"] = label
                                position_state["pitch"] = current_pitch
                                position_state["timestamp"] = now

                            # Publish on /robot_position (only on state change)
                            if label != _last_position_label:
                                pos_msg = String()
                                pos_msg.data = label
                                position_pub.publish(pos_msg)
                                ros_node.get_logger().info(
                                    f"Position: {label} (pitch={current_pitch:.1f}°)")
                                _last_position_label = label

                # ── Live controls (only when changed) ──
                if controls_dirty.is_set():
                    controls_dirty.clear()
                    with config_lock:
                        live_cfg = dict(current_config)
                    apply_camera_controls(state.ctrl_queue, live_cfg)
                    apply_ir_controls(state.device_ref, live_cfg)

                time.sleep(0.001)

        except Exception as e:
            ros_node.get_logger().error(f"Pipeline error: {e}")
            import traceback
            traceback.print_exc()
            time.sleep(2)

        finally:
            state.device_ref = None
            state.ctrl_queue = None
            if pipeline is not None:
                try:
                    pipeline.stop()
                except Exception:
                    pass
            with recording_lock:
                if state.recording_file:
                    state.recording_file.close()
                    state.recording_file = None
                    state.recording_active = False
            ros_node.get_logger().info("Pipeline stopped.")

    ros_node.get_logger().info("Pipeline worker exited.")


# ═══════════════════════════════════════════════════════
# ROS2 Node
# ═══════════════════════════════════════════════════════

class OakDNode(RosNode):
    def __init__(self):
        super().__init__('oakd_node')
        self._declare_and_load_params()

        self.get_logger().info("OAK-D Pro node initializing...")

        devices = dai.Device.getAllAvailableDevices()
        if not devices:
            self.get_logger().error(
                "No OAK-D devices found! Check USB.")
            return
        
        # Publisher for LED commands → led_node subscribes to /led/cmd
        state.led_cmd_pub = self.create_publisher(String, '/led/cmd', 10)

        self.get_logger().info(f"Found {len(devices)} OAK device(s)")

        # Start pipeline thread
        self._pipeline_thread = threading.Thread(
            target=pipeline_worker, args=(self,), daemon=True)
        self._pipeline_thread.start()

        # Start Flask dashboard thread
        with config_lock:
            port = current_config["dashboard_port"]

        self._flask_thread = threading.Thread(
            target=run_server, args=(port,), daemon=True)
        self._flask_thread.start()

        # IMU button subscriptions
        self.create_subscription(Empty, '/imu/zero',  lambda _: self._imu_zero(),  10)
        self.create_subscription(Empty, '/imu/reset', lambda _: self._imu_reset(), 10)

        # Motor status subscription — bridges /motor_status topic into shared state for the dashboard
        # Data layout per motor: [rpm, temp, voltage]; order matches active_ids_: RL=1, FR=6, RR=8, FL=10
        self._motor_labels = ['RL', 'FR', 'RR', 'FL']
        self.create_subscription(
            Float32MultiArray, '/motor_status', self._motor_status_cb, 10)

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

        # ── Position classifier parameters ──
        # Two parallel arrays define the pitch ranges and their labels.
        # position_thresholds: [0, 5, 15, 35, 80, 90]
        #   → ranges are [0,5), [5,15), [15,35), [35,80), [80,90)
        # position_labels: ["horizontal", "buckling", "transitional", "inclined", "vertical"]
        #   → len(labels) must equal len(thresholds) - 1
        self.declare_parameter(
            "position_thresholds",
            [0.0, 2.0, 10.0, 45.0, 55.0, 79.0, 87.0, 90.0])
        self.declare_parameter(
            "position_labels",
            ["horizontal", "horizontal_buckling", "transitional-1", "transitional-2", "transitional-3", "vertical_buckling", "vertical"])

        thresholds = self.get_parameter("position_thresholds").value
        labels = self.get_parameter("position_labels").value

        if len(labels) != len(thresholds) - 1:
            self.get_logger().error(
                f"position_labels ({len(labels)}) must be exactly "
                f"position_thresholds ({len(thresholds)}) - 1!")
            self.get_logger().warn("Using default position ranges.")
            thresholds = [0.0, 2.0, 10.0, 45.0, 55.0, 79.0, 87.0, 90.0]
            labels = ["horizontal", "horizontal_buckling", "transitional-1", "transitional-2", "transitional-3", "vertical_buckling", "vertical"]

        # Build the classifier table
        ranges = []
        for i in range(len(labels)):
            ranges.append((thresholds[i], thresholds[i + 1], labels[i]))

        # Write to shared state (read by classify_position)
        position_ranges.clear()
        position_ranges.extend(ranges)

        self.get_logger().info("Position classifier ranges:")
        for (lo, hi, lbl) in ranges:
            self.get_logger().info(f"  {lo:5.1f}° – {hi:5.1f}° → {lbl}")

    def _print_endpoints(self, port):
        ip_hints = []
        try:
            import socket
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip_hints.append(("eth0", s.getsockname()[0]))
            s.close()
        except Exception:
            ip_hints = [("unknown", "<rpi-ip>")]

            if not ip_hints:
                ip_hints = [("unknown", "<rpi-ip>")]

        self.get_logger().info("=" * 50)
        self.get_logger().info("  OAK-D Pro Camera Node")
        self.get_logger().info("=" * 50)
        for iface, ip in ip_hints:
            self.get_logger().info(f"  [{iface}] http://{ip}:{port}/")
        self.get_logger().info(f"  Publishing /robot_position (String)")
        self.get_logger().info("=" * 50)

    def _imu_zero(self):
        with state.imu_lock:
            state.imu_euler_offsets["pitch"] += state.imu_data["orientation"]["pitch"]
            state.imu_euler_offsets["roll"]  += state.imu_data["orientation"]["roll"]
            state.imu_euler_offsets["yaw"]   += state.imu_data["orientation"]["yaw"]
        self.get_logger().info("IMU zeroed via joystick button.")

    def _imu_reset(self):
        with state.imu_lock:
            state.imu_euler_offsets["pitch"] = 0.0
            state.imu_euler_offsets["roll"]  = 0.0
            state.imu_euler_offsets["yaw"]   = 0.0
        self.get_logger().info("IMU reference cleared via joystick button.")

    def _motor_status_cb(self, msg):
        """Bridge /motor_status topic into shared_state for the Flask dashboard.
        Data layout per motor: [rpm, temp, voltage]
        Order matches dynamixel active_ids_ (ping order): RL=1, FR=6, RR=8, FL=10
        """
        data = msg.data
        entries = []
        for i in range(len(data) // 3):
            label = self._motor_labels[i] if i < len(self._motor_labels) else f'M{i}'
            entries.append({
                'label':   label,
                'rpm':     round(float(data[i*3]),   1),
                'temp':    round(float(data[i*3+1]), 1),
                'voltage': round(float(data[i*3+2]), 2),
            })
        with motor_status_lock:
            state.motor_status = entries


# ═══════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════

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