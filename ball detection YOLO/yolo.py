"""
Soccer Robot Vision System - Final Version
==========================================
Real-time soccer ball tracking and coordinate transmission system
designed to work with the Arduino blind-charge strategy.
"""

import cv2
import numpy as np
import time
import json
import os
import serial
from ultralytics import YOLO

# ══════════════════════════════════════════════════════════════════════
# CONFIGURATION
# ══════════════════════════════════════════════════════════════════════

# Camera configuration
CAMERA_INDEX = 1
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Calibration and model files
CALIBRATION_FILE = "calibration.json"
MODEL_NAME = "yolov8s.onnx"

# YOLO detection settings
CONFIDENCE_THRESHOLD = 0.30  # Minimum confidence required for valid detection
TARGET_CLASS_ID = 32  # COCO class ID for sports ball

# Image enhancement settings
USE_CLAHE = True
CLAHE_CLIP_LIMIT = 2.0
CLAHE_TILE_SIZE = (16, 16)

# Geometric validation settings
USE_GEOMETRIC_FILTER = True
ASPECT_RATIO_TOLERANCE = 0.45  # Accept slightly distorted circular shapes

# Coordinate smoothing settings
EMA_ALPHA = 0.25  # Exponential moving average factor
DEADZONE_CM = 1.0  # Ignore very small coordinate changes

# Physical robot calibration
CAMERA_TO_TOE_OFFSET_Y_CM = 0.0  # Forward distance from camera to robot toe
CAMERA_OFFSET_X_CM = 0.0  # Horizontal camera offset from robot center

# Serial communication settings
SEND_TO_ESP32 = True
ESP32_COM_PORT = "COM8"  # Change to the correct serial port
ESP32_BAUD_RATE = 115200
SEND_EVERY_N_FRAMES = 1

# Display settings
SHOW_HUD = True  # Toggle tracking overlay
SHOW_DEBUG_INFO = True

# Ball tracking fail-safe
BALL_LOST_FRAMES_THRESHOLD = 8  # Frames before sending stop/reset signal

# ══════════════════════════════════════════════════════════════════════
# HELPER FUNCTIONS
# ══════════════════════════════════════════════════════════════════════


def load_calibration(filename):
    """
    Load the homography matrix used to convert image coordinates
    into real-world coordinates.
    """
    if not os.path.exists(filename):
        print(f"ERROR: {filename} not found!")
        print("Please run the calibration script first.")
        return None

    with open(filename, "r") as f:
        data = json.load(f)

    H = np.array(data["homography_matrix"], dtype=np.float32)
    print(f"Loaded calibration from {filename}")
    return H, data


def pixel_to_world(px, py, H):
    """
    Convert pixel coordinates from the camera image into
    real-world coordinates in centimeters.
    """
    pixel_pt = np.array([[[px, py]]], dtype=np.float32)
    world_pt = cv2.perspectiveTransform(pixel_pt, H)
    return float(world_pt[0][0][0]), float(world_pt[0][0][1])


def apply_clahe(frame):
    """
    Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
    to improve visibility and detection reliability under uneven lighting.
    """
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)

    clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP_LIMIT, tileGridSize=CLAHE_TILE_SIZE)

    l = clahe.apply(l)
    enhanced = cv2.merge([l, a, b])

    return cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)


def is_ball_shaped(x1, y1, x2, y2):
    """
    Validate whether the detected bounding box has approximately
    circular proportions consistent with a soccer ball.
    """
    width = x2 - x1
    height = y2 - y1

    if width <= 0 or height <= 0:
        return False

    aspect_ratio = width / height

    return abs(aspect_ratio - 1.0) < ASPECT_RATIO_TOLERANCE


def connect_to_esp32():
    """
    Establish a serial connection to the ESP32 controller.
    """
    if not SEND_TO_ESP32:
        return None

    try:
        esp = serial.Serial(ESP32_COM_PORT, ESP32_BAUD_RATE, timeout=0.01)

        # Allow time for the ESP32 to reset after connection
        time.sleep(2.5)

        esp.flushInput()

        print(f"Connected to ESP32 on {ESP32_COM_PORT}")
        return esp

    except Exception as e:
        print(f"Could not connect to ESP32: {e}")
        print(f"Check that {ESP32_COM_PORT} is correct and the board is connected")
        return None


def draw_hud(frame, ball_detected, smoothed_x, smoothed_y, fps, best_conf=0):
    """
    Draw the on-screen tracking interface and robot status information.
    """

    # Create a semi-transparent background panel for readability
    overlay = frame.copy()

    cv2.rectangle(overlay, (0, 0), (640, 150), (0, 0, 0), -1)

    cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

    cv2.rectangle(frame, (0, 0), (640, 150), (50, 50, 50), 2)

    # Detection status section
    if ball_detected:
        status_text = "BALL LOCKED"
        status_color = (0, 255, 0)

        cv2.putText(
            frame, status_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2
        )

        cv2.putText(
            frame,
            f"Confidence: {best_conf:.0%}",
            (20, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )

    else:
        status_text = "SEARCHING..."
        status_color = (0, 0, 255)

        cv2.putText(
            frame, status_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2
        )

    # Position and alignment information
    if ball_detected:

        cv2.putText(
            frame,
            f"Position: ({smoothed_x:.1f}, {smoothed_y:.1f}) cm",
            (20, 95),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )

        # Determine alignment relative to robot center
        x_aligned = abs(smoothed_x) <= 10.0
        y_close = smoothed_y <= 4.5

        x_status = (
            "ALIGNED"
            if x_aligned
            else f"{'LEFT' if smoothed_x < 0 else 'RIGHT'} OFFSET {abs(smoothed_x):.1f}cm"
        )

        y_status = "BALL IN BLIND SPOT" if y_close else f"{smoothed_y:.1f}cm away"

        cv2.putText(
            frame,
            f"X: {x_status}",
            (20, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 0) if x_aligned else (0, 165, 255),
            1,
        )

        cv2.putText(
            frame,
            f"Y: {y_status}",
            (300, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 0, 0) if y_close else (255, 255, 255),
            1,
        )

    # Display FPS counter
    cv2.putText(
        frame,
        f"FPS: {fps:.0f}",
        (550, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )


# ══════════════════════════════════════════════════════════════════════
# MAIN APPLICATION
# ══════════════════════════════════════════════════════════════════════


def main():

    print("\n" + "=" * 60)
    print("SOCCER ROBOT VISION SYSTEM")
    print("=" * 60 + "\n")

    # Load camera calibration data
    result = load_calibration(CALIBRATION_FILE)

    if result is None:
        return

    H, calib_data = result

    # Load YOLO detection model
    try:
        print(f"Loading YOLO model: {MODEL_NAME}")

        model = YOLO(MODEL_NAME)

        print("Model loaded successfully")

    except Exception as e:
        print(f"ERROR loading model: {e}")
        return

    # Open video capture device
    print(f"Opening camera {CAMERA_INDEX}...")

    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

    # Configure requested camera resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    # Reduce camera buffer size to minimize latency
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # Read back actual camera resolution
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"Requested: {FRAME_WIDTH}x{FRAME_HEIGHT}")
    print(f"Actual: {actual_width}x{actual_height}")

    if actual_width != FRAME_WIDTH or actual_height != FRAME_HEIGHT:
        print("WARNING: Camera resolution mismatch detected")

    # Attempt to reduce motion blur by disabling auto exposure
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)

    if not cap.isOpened():
        print(f"Could not open camera {CAMERA_INDEX}")
        return

    print("Camera opened successfully")

    # Initialize serial communication with ESP32
    esp32 = connect_to_esp32()

    # Runtime tracking variables
    frame_count = 0
    fps_start = time.time()
    fps = 0

    smoothed_x = 0.0
    smoothed_y = 0.0

    previous_sent_x = 0.0
    previous_sent_y = 0.0

    frames_without_ball = 0
    ball_lost_signal_sent = False

    print("\n" + "=" * 60)
    print("Tracking started - Press 'Q' to quit")
    print("=" * 60 + "\n")

    # ══════════════════════════════════════════════════════════════════
    # MAIN PROCESSING LOOP
    # ══════════════════════════════════════════════════════════════════

    while True:

        # Capture current frame
        ret, frame = cap.read()

        if not ret:
            print("Failed to capture frame")
            break

        frame_count += 1

        # Apply optional image enhancement
        processed_frame = apply_clahe(frame) if USE_CLAHE else frame

        # Run YOLO object tracking
        results = model.track(
            processed_frame,
            persist=True,
            verbose=False,
            conf=CONFIDENCE_THRESHOLD,
            tracker="bytetrack.yaml",
        )

        # Variables for selecting the best ball detection
        ball_detected = False
        best_conf = 0
        best_box = None

        # Search through all detections
        for result in results:

            if result.boxes is None:
                continue

            for box in result.boxes:

                # Ignore non-ball detections
                if int(box.cls[0]) != TARGET_CLASS_ID:
                    continue

                # Extract bounding box coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                # Validate geometry if enabled
                if USE_GEOMETRIC_FILTER and not is_ball_shaped(x1, y1, x2, y2):
                    continue

                # Keep the highest confidence detection
                conf = float(box.conf[0])

                if conf > best_conf:
                    best_conf = conf
                    best_box = box

        # Process valid ball detection
        if best_box is not None:

            ball_detected = True
            frames_without_ball = 0
            ball_lost_signal_sent = False

            # Convert bounding box coordinates to integers
            x1, y1, x2, y2 = map(int, best_box.xyxy[0].cpu().numpy())

            # Calculate center point of detected ball
            ball_px = (x1 + x2) // 2
            ball_py = (y1 + y2) // 2

            # Convert image coordinates into world coordinates
            raw_wx, raw_wy = pixel_to_world(ball_px, ball_py, H)

            # Apply robot-specific coordinate offsets
            foot_wx = raw_wx - CAMERA_OFFSET_X_CM
            foot_wy = raw_wy - CAMERA_TO_TOE_OFFSET_Y_CM

            # Smooth coordinates using exponential moving average
            if smoothed_x == 0.0 and smoothed_y == 0.0:

                smoothed_x = foot_wx
                smoothed_y = foot_wy

            else:

                smoothed_x = (EMA_ALPHA * foot_wx) + ((1 - EMA_ALPHA) * smoothed_x)

                smoothed_y = (EMA_ALPHA * foot_wy) + ((1 - EMA_ALPHA) * smoothed_y)

            # Draw detection bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

            # Draw center point
            cv2.circle(frame, (ball_px, ball_py), 8, (0, 0, 255), -1)

            # Draw targeting crosshair
            cv2.line(
                frame, (ball_px - 15, ball_py), (ball_px + 15, ball_py), (0, 255, 0), 2
            )

            cv2.line(
                frame, (ball_px, ball_py - 15), (ball_px, ball_py + 15), (0, 255, 0), 2
            )

        # Handle loss of ball tracking
        if not ball_detected:

            frames_without_ball += 1

            if (
                frames_without_ball >= BALL_LOST_FRAMES_THRESHOLD
                and not ball_lost_signal_sent
            ):

                # Read diagnostic messages from ESP32
                if esp32 is not None:

                    try:

                        if esp32.in_waiting > 0:

                            response = esp32.readline().decode(errors="ignore").strip()

                            if response:
                                print(f"ESP32: {response}")

                    except Exception:
                        print(
                            "WARNING: Robot disconnected - camera will continue running"
                        )
                        esp32 = None

                # Reset smoothed coordinates
                smoothed_x = 0.0
                smoothed_y = 0.0

        # Send coordinates to ESP32
        if (
            ball_detected
            and esp32 is not None
            and frame_count % SEND_EVERY_N_FRAMES == 0
        ):

            if (
                abs(smoothed_x - previous_sent_x) > DEADZONE_CM
                or abs(smoothed_y - previous_sent_y) > DEADZONE_CM
            ):

                try:

                    message = f"{smoothed_x:.2f},{smoothed_y:.2f}\n"

                    esp32.write(message.encode())

                    previous_sent_x = smoothed_x
                    previous_sent_y = smoothed_y

                except Exception:
                    print("WARNING: Robot disconnected - camera will continue running")
                    esp32 = None

        # Read incoming serial messages from ESP32
        if esp32 is not None and esp32.in_waiting > 0:

            try:

                esp_msg = esp32.readline().decode("utf-8", errors="ignore").strip()

                if esp_msg:
                    print(f"ESP32: {esp_msg}")

            except:
                pass

        # Update FPS counter every 10 frames
        if frame_count % 10 == 0:

            elapsed = time.time() - fps_start

            fps = 10 / elapsed if elapsed > 0 else 0

            fps_start = time.time()

        # Draw user interface overlay
        if SHOW_HUD:
            draw_hud(frame, ball_detected, smoothed_x, smoothed_y, fps, best_conf)

        # Display processed frame
        cv2.imshow("Soccer Robot Vision", frame)

        # Exit application when Q is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("\nShutting down...")
            break

    # ══════════════════════════════════════════════════════════════════
    # CLEANUP
    # ══════════════════════════════════════════════════════════════════

    cap.release()

    if esp32:
        # Send stop command before disconnecting
        esp32.write(b"0,0\n")
        esp32.close()

    cv2.destroyAllWindows()

    print("Shutdown complete")


if __name__ == "__main__":
    main()
