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

from camera_driver.shared_state import (
    frame_lock, current_frames,
    config_lock, current_config,
    imu_lock, imu_data,
    recording_lock,
    pipeline_stop, pipeline_restart, controls_dirty,
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
                         dai.IMUSensor.ROTATION_VECTOR], 100)
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
# Pipeline worker thread
# ═══════════════════════════════════════════════════════

def pipeline_worker(ros_node):
    """Runs the DepthAI pipeline and updates shared frame buffers."""

    # [FUTURE] Uncomment for ROS2 image publishing
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

                    # [FUTURE] Publish to ROS2
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
                        disp_color = cv2.applyColorMap(disp_norm, cv2.COLORMAP_JET)
                        with frame_lock:
                            current_frames["depth"] = disp_color

                        # [FUTURE] Publish depth to ROS2
                        # if depth_pub.get_subscription_count() > 0:
                        #     depth_msg = bridge.cv2_to_imgmsg(disp, encoding="mono16")
                        #     depth_msg.header.stamp = ros_node.get_clock().now().to_msg()
                        #     depth_msg.header.frame_id = "oakd_stereo_optical_frame"
                        #     depth_pub.publish(depth_msg)

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

                                if state.imu_ref_set:
                                    # q_rel = q_ref_inv * q_current
                                    # Inverse of unit quaternion: negate i,j,k
                                    ri = -state.imu_ref_quat["i"]
                                    rj = -state.imu_ref_quat["j"]
                                    rk = -state.imu_ref_quat["k"]
                                    rr = state.imu_ref_quat["real"]
                                    # Quaternion multiplication: ref_inv * current
                                    qi2 = rr*qi + ri*qr + rj*qk - rk*qj
                                    qj2 = rr*qj - ri*qk + rj*qr + rk*qi
                                    qk2 = rr*qk + ri*qj - rj*qi + rk*qr
                                    qr2 = rr*qr - ri*qi - rj*qj - rk*qk
                                    qi, qj, qk, qr = qi2, qj2, qk2, qr2

                                # Euler from quaternion (ZYX)
                                import math
                                sinr = 2 * (qr * qi + qj * qk)
                                cosr = 1 - 2 * (qi * qi + qj * qj)
                                sensor_roll = math.atan2(sinr, cosr) * 57.2958

                                sinp = 2 * (qr * qj - qk * qi)
                                sensor_pitch = (math.copysign(90, sinp) if abs(sinp) >= 1
                                                else math.asin(sinp) * 57.2958)

                                siny = 2 * (qr * qk + qi * qj)
                                cosy = 1 - 2 * (qj * qj + qk * qk)
                                sensor_yaw = math.atan2(siny, cosy) * 57.2958

                                # Swap pitch/roll (sensor axes don't match robot axes)
                                imu_data["orientation"]["pitch"] = round(sensor_roll, 1)
                                imu_data["orientation"]["roll"] = round(sensor_yaw, 1)
                                imu_data["orientation"]["yaw"] = round(sensor_pitch, 1)

                                imu_data["timestamp"] = time.time()

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

    def _print_endpoints(self, port):
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