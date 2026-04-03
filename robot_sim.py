"""
robot_sim.py
------------
PyBullet 3-D KUKA iiwa 7-DOF robot arm simulation.

Finger → Joint mapping
───────────────────────────────────────────
Joint 0  Base rotation          ← Index
Joint 1  Shoulder flex          ← Middle
Joint 2  Upper-arm rotation     ← Thumb
Joint 3  Elbow flex             ← Ring
Joint 4  Forearm rotation       ← Pinky
Joint 5  Wrist flex             ← Index  (mirrors J0)
Joint 6  Wrist rotation         ← Middle (mirrors J1)
"""

import pybullet as p
import pybullet_data
import numpy as np


_GREEN  = [0.2, 1.0, 0.4]
_RED    = [1.0, 0.3, 0.3]
_YELLOW = [1.0, 0.9, 0.2]
_CYAN   = [0.2, 0.9, 1.0]


class RobotSimulation:

    FINGER_TO_JOINT = {
        0: "index",
        1: "middle",
        2: "thumb",
        3: "ring",
        4: "pinky",
        5: "index",
        6: "middle",
    }

    def __init__(self):
        self.client = p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(1)

        # ── Scene ────────────────────────────────────────────────────
        p.loadURDF("plane.urdf")
        self.robot = p.loadURDF(
            "kuka_iiwa/model.urdf",
            basePosition=[0, 0, 0],
            useFixedBase=True,
        )

        # ── Camera view ──────────────────────────────────────────────
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0.4],
        )
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # hide default GUI sliders

        # ── Joint metadata ───────────────────────────────────────────
        self.num_joints    = p.getNumJoints(self.robot)
        self.joint_limits  = self._load_joint_limits()

        # ── Persistent debug text ─────────────────────────────────────
        self._title_id  = p.addUserDebugText(
            "Gesture Robot  |  KUKA iiwa",
            [0, 0, 1.4],
            textColorRGB=_YELLOW,
            textSize=1.6,
        )
        self._status_id = p.addUserDebugText(
            "Waiting for hand...",
            [0, 0, 1.22],
            textColorRGB=_RED,
            textSize=1.1,
        )
        self._bar_ids: list = []

        # ── Draw world axes for reference ─────────────────────────────
        p.addUserDebugLine([0,0,0],[0.3,0,0],[1,0,0],2)
        p.addUserDebugLine([0,0,0],[0,0.3,0],[0,1,0],2)
        p.addUserDebugLine([0,0,0],[0,0,0.3],[0,0,1],2)

    # ── Public API ───────────────────────────────────────────────────

    def set_finger_angles(self, angles: dict):
        """Drive all 7 joints from normalised finger angles [0-1]."""
        for joint_idx, finger in self.FINGER_TO_JOINT.items():
            target = self._denormalise(angles.get(finger, 0.0), joint_idx)
            p.setJointMotorControl2(
                bodyUniqueId=self.robot,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target,
                force=500,
                maxVelocity=1.5,
                positionGain=0.3,
                velocityGain=1.0,
            )

    def update_hud(self, angles: dict, hand_detected: bool):
        """Refresh the PyBullet in-sim overlay."""
        status_text  = "● HAND DETECTED" if hand_detected else "○ No hand..."
        status_color = _GREEN if hand_detected else _RED

        p.removeUserDebugItem(self._status_id)
        self._status_id = p.addUserDebugText(
            status_text, [0, 0, 1.22],
            textColorRGB=status_color, textSize=1.1,
        )

        # Remove old bars
        for _id in self._bar_ids:
            p.removeUserDebugItem(_id)
        self._bar_ids.clear()

        finger_order = ["index", "middle", "thumb", "ring", "pinky"]
        joint_names  = ["Base", "Shoulder", "Upper-arm", "Elbow", "Forearm"]

        for i, (finger, jname) in enumerate(zip(finger_order, joint_names)):
            val   = angles.get(finger, 0.0)
            bar   = "█" * int(val * 12) + "░" * (12 - int(val * 12))
            label = f"{jname:<10s} {bar} {int(val*100):3d}%"
            color = [0.3 + val * 0.7, 0.9 - val * 0.6, 0.4 + val * 0.4]
            _id   = p.addUserDebugText(
                label,
                [-0.7, 0.6 - i * 0.13, 0.05],
                textColorRGB=color,
                textSize=0.95,
            )
            self._bar_ids.append(_id)

        # Show end-effector world position
        ee_state = p.getLinkState(self.robot, self.num_joints - 1)
        ee_pos   = ee_state[0]
        p.removeUserDebugItem(getattr(self, "_ee_id", -1))
        self._ee_id = p.addUserDebugText(
            f"EE  x:{ee_pos[0]:+.2f}  y:{ee_pos[1]:+.2f}  z:{ee_pos[2]:+.2f}",
            [0, 0, 1.06],
            textColorRGB=_CYAN,
            textSize=0.9,
        )

    def is_running(self) -> bool:
        return p.isConnected(self.client)

    def close(self):
        if p.isConnected(self.client):
            p.disconnect(self.client)

    # ── Internal helpers ─────────────────────────────────────────────

    def _load_joint_limits(self):
        limits = []
        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot, i)
            lower, upper = info[8], info[9]
            if lower >= upper:
                lower, upper = -np.pi, np.pi
            limits.append((lower, upper))
        return limits

    def _denormalise(self, normalised: float, joint_idx: int) -> float:
        lower, upper = self.joint_limits[joint_idx]
        return lower + float(normalised) * (upper - lower)