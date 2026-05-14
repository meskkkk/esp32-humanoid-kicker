"""
Camera-to-World Calibration Tool
================================
Interactive calibration utility for converting camera pixel coordinates
into real-world robot coordinates using homography transformation.
"""

import cv2
import numpy as np
import json

# ══════════════════════════════════════════════════════════════════════
# CAMERA CONFIGURATION
# ══════════════════════════════════════════════════════════════════════

CAMERA_INDEX = 1
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Output calibration file
CALIBRATION_FILE = "calibration.json"

# Default reference rectangle dimensions (centimeters)
DEFAULT_WIDTH_CM = 30.0
DEFAULT_HEIGHT_CM = 21.0

# ══════════════════════════════════════════════════════════════════════
# ROBOT COORDINATE SYSTEM OFFSET
# Defines the origin relationship between the robot and calibration area
# ══════════════════════════════════════════════════════════════════════

ROBOT_X_OFFSET = 0.0

ROBOT_Y_OFFSET = 10.0 + (DEFAULT_HEIGHT_CM / 2.0)

# Stores user-selected calibration points
clicked_points = []

# OpenCV window title
window_name = "Camera Calibration — Click 4 corners of reference rectangle"

# ══════════════════════════════════════════════════════════════════════
# MOUSE INPUT HANDLER
# ══════════════════════════════════════════════════════════════════════


def mouse_click(event, x, y, flags, param):
    """
    Record mouse clicks used for selecting calibration corners.
    """
    global clicked_points

    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < 4:

        clicked_points.append((x, y))

        print(
            f"Point {len(clicked_points)} selected " f"at pixel coordinates ({x}, {y})"
        )


# ══════════════════════════════════════════════════════════════════════
# CALIBRATION USER INTERFACE
# ══════════════════════════════════════════════════════════════════════


def draw_calibration_overlay(frame, points, instruction):
    """
    Draw the calibration interface overlay, including instructions,
    selected points, and rectangle visualization.
    """

    overlay = frame.copy()

    cv2.rectangle(overlay, (0, 0), (FRAME_WIDTH, 80), (0, 0, 0), -1)

    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

    # Display current instruction
    cv2.putText(
        frame, instruction, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2
    )

    # Display point selection progress
    cv2.putText(
        frame,
        f"Clicked: {len(points)}/4 corners",
        (15, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1,
    )

    # Labels corresponding to selection order
    corner_labels = ["TOP-LEFT", "TOP-RIGHT", "BOTTOM-RIGHT", "BOTTOM-LEFT"]

    # Draw selected points and labels
    for i, (px, py) in enumerate(points):

        cv2.circle(frame, (px, py), 10, (0, 255, 0), -1)

        cv2.circle(frame, (px, py), 12, (255, 255, 255), 2)

        cv2.putText(
            frame,
            f"{i+1}: {corner_labels[i]}",
            (px + 15, py + 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )

    # Draw connecting rectangle lines
    if len(points) >= 2:

        for i in range(len(points)):

            if i + 1 < len(points):

                cv2.line(frame, points[i], points[i + 1], (0, 255, 0), 2)

        # Close rectangle when all four corners exist
        if len(points) == 4:

            cv2.line(frame, points[3], points[0], (0, 255, 0), 2)

    # Bottom instruction hint
    cv2.putText(
        frame,
        "Press R to reset | Q to quit | ENTER when 4 corners selected",
        (15, FRAME_HEIGHT - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (200, 200, 200),
        1,
    )

    return frame


# ══════════════════════════════════════════════════════════════════════
# HOMOGRAPHY CALCULATION
# ══════════════════════════════════════════════════════════════════════


def compute_homography(image_points, world_corners):
    """
    Compute the homography matrix that maps image pixel coordinates
    into real-world coordinates.
    """

    src = np.array(image_points, dtype=np.float32)
    dst = np.array(world_corners, dtype=np.float32)

    H, _ = cv2.findHomography(src, dst)

    return H


# ══════════════════════════════════════════════════════════════════════
# CALIBRATION VERIFICATION
# ══════════════════════════════════════════════════════════════════════


def test_calibration(H, frame, mouse_pos):
    """
    Display real-world coordinates in real time as the user
    moves the mouse over the camera image.
    """

    if mouse_pos is not None:

        px, py = mouse_pos

        # Convert pixel coordinates into world coordinates
        pixel_pt = np.array([[[px, py]]], dtype=np.float32)

        world_pt = cv2.perspectiveTransform(pixel_pt, H)

        wx, wy = world_pt[0][0]

        # Draw cursor marker
        cv2.circle(frame, (px, py), 8, (255, 0, 255), -1)

        # Display transformed coordinates
        cv2.putText(
            frame,
            f"({wx:.1f}, {wy:.1f}) cm",
            (px + 15, py + 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 0, 255),
            2,
        )


# ══════════════════════════════════════════════════════════════════════
# MAIN APPLICATION
# ══════════════════════════════════════════════════════════════════════


def main():

    global clicked_points

    print("=" * 65)
    print("CAMERA-TO-WORLD CALIBRATION")
    print("Robot Vision Calibration Utility")
    print("=" * 65)

    # ══════════════════════════════════════════════════════════════════
    # STEP 1: REFERENCE OBJECT CONFIGURATION
    # ══════════════════════════════════════════════════════════════════

    print("\nSTEP 1: Reference object setup")
    print("-" * 65)

    print("Place a rectangular object with known dimensions " "in front of the camera.")

    print("(Example: A4 paper in landscape orientation = 30cm x 21cm)")

    print(f"\nDefault dimensions: " f"{DEFAULT_WIDTH_CM} cm × {DEFAULT_HEIGHT_CM} cm")

    try:

        user_input = input("Use default dimensions? (Y/n): ").strip().lower()

        if user_input == "n":

            ref_width = float(input("Reference WIDTH in cm: "))

            ref_height = float(input("Reference HEIGHT in cm: "))

        else:

            ref_width = DEFAULT_WIDTH_CM
            ref_height = DEFAULT_HEIGHT_CM

    except (ValueError, KeyboardInterrupt):

        print("Using default dimensions.")

        ref_width = DEFAULT_WIDTH_CM
        ref_height = DEFAULT_HEIGHT_CM

    print(f"Reference object dimensions: " f"{ref_width} cm × {ref_height} cm")

    # ══════════════════════════════════════════════════════════════════
    # STEP 2: CAMERA INITIALIZATION
    # ══════════════════════════════════════════════════════════════════

    print("\nSTEP 2: Opening camera")

    cap = cv2.VideoCapture(CAMERA_INDEX)

    if not cap.isOpened():

        print("ERROR: Could not open camera")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    print("Camera initialized successfully")

    # ══════════════════════════════════════════════════════════════════
    # STEP 3: USER POINT SELECTION
    # ══════════════════════════════════════════════════════════════════

    print("\nSTEP 3: Select 4 corners of the reference rectangle")
    print("-" * 65)

    print("Corner selection order is important:")

    print("1. TOP-LEFT corner")
    print("2. TOP-RIGHT corner")
    print("3. BOTTOM-RIGHT corner")
    print("4. BOTTOM-LEFT corner")

    print("\nControls:")
    print("R = Reset points")
    print("Q = Quit")
    print("ENTER = Confirm selection\n")

    cv2.namedWindow(window_name)

    cv2.setMouseCallback(window_name, mouse_click)

    instructions = [
        "Click TOP-LEFT corner of reference object",
        "Click TOP-RIGHT corner",
        "Click BOTTOM-RIGHT corner",
        "Click BOTTOM-LEFT corner",
        "All corners selected - Press ENTER to confirm",
    ]

    while True:

        ret, frame = cap.read()

        if not ret:
            break

        # Draw calibration overlay
        instruction = instructions[min(len(clicked_points), 4)]

        frame = draw_calibration_overlay(frame, clicked_points, instruction)

        cv2.imshow(window_name, frame)

        key = cv2.waitKey(1) & 0xFF

        # Quit application
        if key == ord("q"):

            print("Calibration cancelled")

            cap.release()
            cv2.destroyAllWindows()

            return

        # Reset selected points
        elif key == ord("r"):

            clicked_points = []

            print("Point selection reset")

        # Confirm calibration
        elif key == 13 and len(clicked_points) == 4:

            break

    # Ensure exactly four points were selected
    if len(clicked_points) != 4:

        print("ERROR: Exactly 4 points are required")

        cap.release()
        cv2.destroyAllWindows()

        return

    # ══════════════════════════════════════════════════════════════════
    # STEP 4: HOMOGRAPHY COMPUTATION
    # ══════════════════════════════════════════════════════════════════

    print("\nSTEP 4: Computing homography matrix")

    half_w = ref_width / 2
    half_h = ref_height / 2

    # Real-world corner coordinates
    world_corners = [
        # TOP-LEFT
        (ROBOT_X_OFFSET - half_w, ROBOT_Y_OFFSET + half_h),
        # TOP-RIGHT
        (ROBOT_X_OFFSET + half_w, ROBOT_Y_OFFSET + half_h),
        # BOTTOM-RIGHT
        (ROBOT_X_OFFSET + half_w, ROBOT_Y_OFFSET - half_h),
        # BOTTOM-LEFT
        (ROBOT_X_OFFSET - half_w, ROBOT_Y_OFFSET - half_h),
    ]

    # Compute transformation matrix
    H = compute_homography(clicked_points, world_corners)

    print("Homography matrix computed successfully")

    print(f"\nMatrix:\n{H}")

    # ══════════════════════════════════════════════════════════════════
    # STEP 5: LIVE VERIFICATION
    # ══════════════════════════════════════════════════════════════════

    print("\nSTEP 5: Calibration verification")
    print("-" * 65)

    print("Move the mouse over the image to inspect " "real-world coordinates.")

    print("Press S to save or Q to restart calibration.\n")

    mouse_pos = [None]

    def verify_mouse(event, x, y, flags, param):

        if event == cv2.EVENT_MOUSEMOVE:
            mouse_pos[0] = (x, y)

    cv2.setMouseCallback(window_name, verify_mouse)

    while True:

        ret, frame = cap.read()

        if not ret:
            break

        # Draw calibration rectangle
        for i in range(4):

            cv2.line(
                frame, clicked_points[i], clicked_points[(i + 1) % 4], (0, 255, 0), 2
            )

            cv2.circle(frame, clicked_points[i], 6, (0, 255, 0), -1)

        # Draw top information banner
        overlay = frame.copy()

        cv2.rectangle(overlay, (0, 0), (FRAME_WIDTH, 50), (0, 0, 0), -1)

        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

        cv2.putText(
            frame,
            "VERIFICATION MODE - Move mouse to inspect coordinates",
            (15, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )

        # Display live coordinate conversion
        test_calibration(H, frame, mouse_pos[0])

        # Draw bottom instructions
        cv2.putText(
            frame,
            "Press S to SAVE | Q to restart",
            (15, FRAME_HEIGHT - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
        )

        cv2.imshow(window_name, frame)

        key = cv2.waitKey(1) & 0xFF

        # Save calibration
        if key == ord("s"):
            break

        # Restart calibration process
        elif key == ord("q"):

            print("Restarting calibration process")

            cap.release()
            cv2.destroyAllWindows()

            clicked_points = []

            return main()

    # ══════════════════════════════════════════════════════════════════
    # SAVE CALIBRATION DATA
    # ══════════════════════════════════════════════════════════════════

    calibration_data = {
        "frame_width": FRAME_WIDTH,
        "frame_height": FRAME_HEIGHT,
        "reference_dimensions_cm": {"width": ref_width, "height": ref_height},
        "image_corners_pixels": clicked_points,
        "world_corners_cm": world_corners,
        "homography_matrix": H.tolist(),
        "robot_origin_offset_cm": {"x": ROBOT_X_OFFSET, "y": ROBOT_Y_OFFSET},
    }

    # Write calibration data to JSON file
    with open(CALIBRATION_FILE, "w") as f:

        json.dump(calibration_data, f, indent=2)

    print("\n" + "=" * 65)

    print(f"Calibration saved successfully to " f"{CALIBRATION_FILE}")

    print("=" * 65)

    print("You can now run the ball detection system.")

    print("=" * 65 + "\n")

    # Release system resources
    cap.release()

    cv2.destroyAllWindows()


# ══════════════════════════════════════════════════════════════════════
# APPLICATION ENTRY POINT
# ══════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    main()
