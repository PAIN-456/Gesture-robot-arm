"""
main.py
-------
Gesture-Controlled Robot Arm — OpenCV webcam + PyBullet 3D KUKA iiwa.

Controls
────────
  Move fingers  →  robot joints move in real time
  Q             →  quit
"""

import sys
import cv2
import numpy as np
from collections import deque

from hand_tracker import HandTracker
from robot_sim    import RobotSimulation

# ── Config ────────────────────────────────────────────────────────────
CAMERA_INDEX   = 0
SMOOTHING_SIZE = 6
CAM_WINDOW     = "Webcam  |  Press Q to quit"

FINGER_ORDER = ["thumb", "index", "middle", "ring", "pinky"]
JOINT_LABELS = {
    "thumb":  "Upper-arm twist",
    "index":  "Base + Wrist",
    "middle": "Shoulder + Wrist-rot",
    "ring":   "Elbow",
    "pinky":  "Forearm",
}


# ── Smoothing ─────────────────────────────────────────────────────────
class AngleSmoother:
    def __init__(self, size, fingers):
        self.buffers = {f: deque([0.0] * size, maxlen=size) for f in fingers}

    def update(self, angles):
        for f, v in angles.items():
            self.buffers[f].append(v)
        return {f: float(np.mean(b)) for f, b in self.buffers.items()}


# ── Webcam HUD ────────────────────────────────────────────────────────
def draw_hud(frame, angles, hand_detected):
    h, w = frame.shape[:2]

    if hand_detected:
        cv2.rectangle(frame, (0, 0), (w, 45), (20, 70, 20), -1)
        cv2.putText(frame, "● HAND DETECTED", (12, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, (80, 255, 100), 2)
    else:
        cv2.rectangle(frame, (0, 0), (w, 45), (30, 20, 20), -1)
        cv2.putText(frame, "○ Show your hand...", (12, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, (80, 100, 255), 2)

    for i, finger in enumerate(FINGER_ORDER):
        y   = 65 + i * 38
        val = angles.get(finger, 0.0)
        label = f"{finger.capitalize():<7s}  {JOINT_LABELS[finger]}"
        cv2.putText(frame, label, (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)
        cv2.rectangle(frame, (10, y + 4), (210, y + 18), (50, 50, 50), -1)
        fill = int(val * 200)
        r = int(255 * val); g = int(255 * (1 - val))
        cv2.rectangle(frame, (10, y + 4), (10 + fill, y + 18), (0, g, r), -1)
        cv2.putText(frame, f"{int(val*100):3d}%", (216, y + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)

    cv2.rectangle(frame, (0, h - 28), (w, h), (15, 15, 15), -1)
    cv2.putText(frame, "PyBullet KUKA iiwa  |  Press Q to quit",
                (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (130, 130, 130), 1)


# ── Main ─────────────────────────────────────────────────────────────
def main():
    print("\n  Gesture-Controlled KUKA iiwa  (PyBullet)")
    print("=" * 44)
    print("  Finger   ->  Joint")
    print("  Index    ->  Base rotation + Wrist flex")
    print("  Middle   ->  Shoulder + Wrist rotation")
    print("  Thumb    ->  Upper-arm twist")
    print("  Ring     ->  Elbow flex")
    print("  Pinky    ->  Forearm rotation")
    print("=" * 44)
    print("  Two windows will open:")
    print("  1) PyBullet — 3D KUKA arm")
    print("  2) OpenCV   — your webcam")
    print("  Press Q in the webcam window to quit.\n")

    tracker  = HandTracker()
    robot    = RobotSimulation()
    smoother = AngleSmoother(SMOOTHING_SIZE, FINGER_ORDER)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {CAMERA_INDEX}")
        robot.close()
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    cv2.namedWindow(CAM_WINDOW)
    cv2.moveWindow(CAM_WINDOW, 40, 100)

    print("Camera ready — show your hand!\n")

    try:
        while cap.isOpened() and robot.is_running():
            ret, frame = cap.read()
            if not ret:
                continue

            frame = cv2.flip(frame, 1)

            raw_angles, hand_detected, frame = tracker.process(frame)
            smooth_angles = smoother.update(raw_angles)

            robot.set_finger_angles(smooth_angles)
            robot.update_hud(smooth_angles, hand_detected)

            draw_hud(frame, smooth_angles, hand_detected)
            cv2.imshow(CAM_WINDOW, frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("[INFO] Quitting...")
                break

    except KeyboardInterrupt:
        print("[INFO] Interrupted.")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        robot.close()
        print("Goodbye!\n")


if __name__ == "__main__":
    main()