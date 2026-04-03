"""
hand_tracker.py
---------------
Uses MediaPipe to detect hand landmarks and compute
normalized bend angles [0.0 - 1.0] for each finger.

  0.0 = fully straight
  1.0 = fully bent / curled
"""

import cv2
import mediapipe as mp
import numpy as np


class HandTracker:
    # MediaPipe landmark indices for each finger: [MCP, PIP, DIP, TIP]
    FINGER_INDICES = {
        "thumb":  [1,  2,  3,  4],
        "index":  [5,  6,  7,  8],
        "middle": [9,  10, 11, 12],
        "ring":   [13, 14, 15, 16],
        "pinky":  [17, 18, 19, 20],
    }

    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.mp_draw  = mp.solutions.drawing_utils
        self.mp_style = mp.solutions.drawing_styles

        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.75,
            min_tracking_confidence=0.6,
        )

    # ------------------------------------------------------------------
    def process(self, bgr_frame):
        """
        Process a BGR camera frame.

        Returns
        -------
        angles       : dict  {finger_name: float [0-1]}
        hand_detected: bool
        annotated    : frame with landmarks drawn on it
        """
        rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        angles = {name: 0.0 for name in self.FINGER_INDICES}
        hand_detected = False

        if results.multi_hand_landmarks:
            hand_detected = True
            lm = results.multi_hand_landmarks[0]

            # Draw landmarks on frame
            self.mp_draw.draw_landmarks(
                bgr_frame, lm,
                self.mp_hands.HAND_CONNECTIONS,
                self.mp_style.get_default_hand_landmarks_style(),
                self.mp_style.get_default_hand_connections_style(),
            )

            for name, idx in self.FINGER_INDICES.items():
                angles[name] = self._bend_angle(lm.landmark, idx)

        return angles, hand_detected, bgr_frame

    # ------------------------------------------------------------------
    def _bend_angle(self, landmarks, indices):
        """
        Compute the bend angle at the PIP (middle) knuckle.
        Returns a value normalised to [0, 1].
        """
        pts = [
            np.array([landmarks[i].x, landmarks[i].y, landmarks[i].z])
            for i in indices[:3]          # MCP, PIP, DIP
        ]
        v1 = pts[0] - pts[1]             # MCP → PIP
        v2 = pts[2] - pts[1]             # DIP → PIP

        cos_a = np.dot(v1, v2) / (
            np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-8
        )
        angle_deg = np.degrees(np.arccos(np.clip(cos_a, -1.0, 1.0)))

        # Straight finger ≈ 160–180°  →  0.0
        # Fully bent      ≈  60–90°   →  1.0
        normalised = 1.0 - (angle_deg / 180.0)
        return float(np.clip(normalised, 0.0, 1.0))