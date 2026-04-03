# 🤖 Gesture-Controlled Robot Arm

Control a **KUKA iiwa 7-DOF robot arm** in a real-time 3D physics simulation using nothing but your hand and a webcam.

Built with OpenCV, MediaPipe and PyBullet.

---

## 📽️ Demo

> Show your hand to the webcam — each finger controls a different robot joint in real time.

---

## 🧠 How It Works

```
Webcam
  ↓
MediaPipe (21 hand landmarks)
  ↓
Finger bend angles [0.0 - 1.0]
  ↓
Averaged over 6 frames (jitter smoothing)
  ↓
Mapped to KUKA iiwa joint ranges
  ↓
PyBullet POSITION_CONTROL (real physics)
```

---

## 🖐️ Finger → Joint Mapping

| Finger | Robot Joint | Motion |
|--------|------------|--------|
| Index  | Joint 0 + 5 | Base rotation + Wrist flex |
| Middle | Joint 1 + 6 | Shoulder + Wrist rotation  |
| Thumb  | Joint 2     | Upper-arm twist            |
| Ring   | Joint 3     | Elbow flex                 |
| Pinky  | Joint 4     | Forearm rotation           |

---

## 📁 Project Structure

```
gesture-robot-arm/
├── main.py          ← Entry point
├── hand_tracker.py  ← MediaPipe hand landmark + angle extraction
├── robot_sim.py     ← PyBullet KUKA iiwa simulation
├── requirements.txt
└── README.md
```

---

## ⚡ Quick Start

```bash
# 1. Clone the repo
git clone https://github.com/PAIN-456/Gesture-robot-arm.git
cd Gesture-robot-arm

# 2. Create virtual environment (Python 3.11 recommended)
python -m venv .venv
.venv\Scripts\activate      # Windows
# source .venv/bin/activate  # Mac/Linux

# 3. Install dependencies
pip install -r requirements.txt

# 4. Run
python main.py
```

> ⚠️ PyBullet requires **Microsoft C++ Build Tools** on Windows.
> Download from: https://visualstudio.microsoft.com/visual-cpp-build-tools/

---

## 🔧 Requirements

- Python 3.11
- Webcam
- Microsoft C++ Build Tools (Windows only, for PyBullet)

---

## 📦 Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| opencv-python | ≥ 4.8 | Camera capture + drawing |
| mediapipe | 0.10.9 | Hand landmark detection |
| pybullet | ≥ 3.2 | 3D physics simulation |
| numpy | ≥ 1.24 | Angle math + smoothing |

---

## 🚀 Ideas to Extend

- Add a gripper that opens/closes with fist gesture
- Inverse kinematics — arm tip follows your fingertip position
- Record joint angles to CSV and replay as a robot program
- Add ROS publisher node to control a real robot
- Swap KUKA for Franka Panda or a custom URDF

---

## 👤 Author

**PAIN-456** — https://github.com/PAIN-456
