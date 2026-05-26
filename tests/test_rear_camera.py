"""
Rear Camera Test — RPi Camera V2 NoIR (IMX219PQ)
==================================================
Displays live video feed from the CSI-connected camera.

Wiring:
  Camera ribbon cable → RPi CSI connector
  (make sure the blue side of the ribbon faces the USB ports on RPi5)

Prerequisites:
  sudo apt install -y python3-picamera2 python3-opencv
  # picamera2 comes pre-installed on Raspberry Pi OS (Bookworm)

Usage:
  python3 test_rear_camera.py

  If running over SSH, you need X11 forwarding:
    ssh -X pi@<ip>
  Or use the --headless flag to save frames to disk instead:
    python3 test_rear_camera.py --headless

Controls (in GUI mode):
  q     - quit
  s     - save current frame
  f     - toggle FPS overlay
  1/2/3 - switch resolution (480p / 720p / 1080p)
"""

import sys
import time
import argparse
import os

try:
    from picamera2 import Picamera2
    import cv2
except ImportError as e:
    print(f"Missing dependency: {e}")
    print("Install with:")
    print("  sudo apt install -y python3-picamera2 python3-opencv")
    sys.exit(1)


RESOLUTIONS = {
    "480p":  (640, 480),
    "720p":  (1280, 720),
    "1080p": (1920, 1080),
}

SAVE_DIR = "/home/pi/rear_camera_captures"


def run_gui(camera: Picamera2):
    """Live preview with OpenCV window."""
    show_fps = True
    frame_count = 0
    fps_time = time.time()
    fps_display = 0.0

    print("  Live preview started. Press 'q' to quit.")
    print("  Keys: s=save, f=toggle FPS, 1/2/3=resolution")

    while True:
        frame = camera.capture_array()

        # Convert RGB to BGR for OpenCV
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # FPS calculation
        frame_count += 1
        elapsed = time.time() - fps_time
        if elapsed >= 1.0:
            fps_display = frame_count / elapsed
            frame_count = 0
            fps_time = time.time()

        if show_fps:
            cv2.putText(frame_bgr, f"FPS: {fps_display:.1f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (0, 255, 0), 2)

        cv2.imshow("Rear Camera", frame_bgr)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('s'):
            os.makedirs(SAVE_DIR, exist_ok=True)
            fname = os.path.join(SAVE_DIR, f"rear_{int(time.time())}.jpg")
            cv2.imwrite(fname, frame_bgr)
            print(f"  Saved: {fname}")
        elif key == ord('f'):
            show_fps = not show_fps
        elif key == ord('1'):
            camera.stop()
            camera.configure(camera.create_preview_configuration(
                main={"size": RESOLUTIONS["480p"], "format": "RGB888"}))
            camera.start()
            print("  Switched to 480p")
        elif key == ord('2'):
            camera.stop()
            camera.configure(camera.create_preview_configuration(
                main={"size": RESOLUTIONS["720p"], "format": "RGB888"}))
            camera.start()
            print("  Switched to 720p")
        elif key == ord('3'):
            camera.stop()
            camera.configure(camera.create_preview_configuration(
                main={"size": RESOLUTIONS["1080p"], "format": "RGB888"}))
            camera.start()
            print("  Switched to 1080p")

    cv2.destroyAllWindows()


def run_headless(camera: Picamera2, num_frames: int = 5):
    """Capture a few frames and save to disk (for SSH without X11)."""
    os.makedirs(SAVE_DIR, exist_ok=True)

    print(f"  Headless mode: capturing {num_frames} frames...")

    for i in range(num_frames):
        frame = camera.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        fname = os.path.join(SAVE_DIR, f"rear_headless_{i}.jpg")
        cv2.imwrite(fname, frame_bgr)
        print(f"  Saved: {fname}")
        time.sleep(1)

    print(f"  Done. Frames saved in {SAVE_DIR}")


def main():
    parser = argparse.ArgumentParser(description="Rear camera test")
    parser.add_argument("--headless", action="store_true",
                        help="Save frames to disk instead of showing GUI")
    parser.add_argument("--resolution", choices=["480p", "720p", "1080p"],
                        default="720p", help="Starting resolution")
    args = parser.parse_args()

    print("=" * 50)
    print("  Rear Camera Test (RPi Camera V2 NoIR / IMX219)")
    print("=" * 50)

    camera = Picamera2()

    res = RESOLUTIONS[args.resolution]
    config = camera.create_preview_configuration(
        main={"size": res, "format": "RGB888"}
    )
    camera.configure(config)
    camera.start()

    # Let auto-exposure settle
    print(f"  Camera started at {args.resolution} ({res[0]}x{res[1]})")
    print("  Waiting for auto-exposure to settle...")
    time.sleep(2)

    try:
        if args.headless:
            run_headless(camera)
        else:
            run_gui(camera)
    except KeyboardInterrupt:
        print("\n  Stopped.")
    finally:
        camera.stop()
        camera.close()
        print("  Camera closed.")


if __name__ == "__main__":
    main()
