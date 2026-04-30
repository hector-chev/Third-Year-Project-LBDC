# LBDC – Low-Budget Dexterous Controller: Vision-Based Robotic Arm Teleoperation System

---

## Introduction

This project implements a real-time teleoperation system for a 6-degree-of-freedom robotic arm, controlled entirely through hand gestures captured by two standard USB cameras and an IMU-equipped glove. The system tracks the 3D position of a human hand in space using computer vision and translates that position — along with wrist orientation and gripper state — into servo commands for the robotic arm, with no specialist motion-capture hardware required.

The software spans three hardware platforms working in concert: a Raspberry Pi running the computer vision pipeline, an Arduino Nano mounted on the glove measuring wrist orientation via an MPU-6050 IMU, and an Arduino Mega acting as the robot controller, performing inverse kinematics and driving six servo motors. The overall goal is a low-cost, markerless teleoperation interface that can replicate the operator's hand position and orientation in the robot's workspace in real time.

---

## Contextual Overview

The system architecture follows a three-stage pipeline:

```
┌──────────────────────────────────────────────────────────────────────┐
│                        SENSING LAYER                                 │
│                                                                      │
│  ┌─────────────┐   ┌─────────────┐   ┌──────────────────────────┐   │
│  │  Side USB   │   │  Top USB    │   │  Arduino Nano (on glove) │   │
│  │  Camera     │   │  Camera     │   │  MPU-6050 IMU            │   │
│  │  (Y, Z pos) │   │  (X, Y pos) │   │  (wrist θ, φ angles)    │   │
│  └──────┬──────┘   └──────┬──────┘   └────────────┬─────────────┘   │
│         └────────────┬────┘                        │ Serial (COM6)   │
└──────────────────────│────────────────────────────│─────────────────┘
                       │                             │
┌──────────────────────▼─────────────────────────────▼─────────────────┐
│                   RASPBERRY PI (Python)                               │
│                                                                       │
│  • Captures & undistorts frames from both cameras                    │
│  • Runs MediaPipe Hands on each frame                                 │
│  • Computes real-world X, Y, Z hand position (cm)                    │
│  • Reads θ (pitch) and φ (roll) from Arduino Nano via serial         │
│  • Computes normalised finger-spread distance D for gripper control  │
│  • Sends CSV string: X, Y, Z, D, θ, φ, timestamp → Arduino Mega     │
│                                                                       │
└──────────────────────────────┬────────────────────────────────────────┘
                               │ Serial (COM3, 115200 baud)
┌──────────────────────────────▼────────────────────────────────────────┐
│                  ARDUINO MEGA (C++)                                    │
│                                                                        │
│  • Parses incoming CSV from Raspberry Pi                               │
│  • Applies EWM smoothing filters to X, Y, Z, θ, φ                    │
│  • Solves geometric inverse kinematics for 3-DOF arm                  │
│  • Maps wrist angles and gripper distance to servo outputs            │
│  • Drives 6 × servo motors (base, shoulder, elbow, pitch, roll, grip)│
│  • Reads potentiometer to set maximum step size (speed limit)         │
│  • Displays live state on SSD1306 OLED display                        │
└────────────────────────────────────────────────────────────────────────┘
```

**Camera placement:** the side camera observes the hand from the side, providing Y (lateral) and Z (height) coordinates; the top camera observes from above, providing X (depth) and Y coordinates. The two Y estimates are cross-checked and the discrepancy is logged.

---

## Installation Instructions

### Required Hardware

- Raspberry Pi (any model with USB ports and Python 3 support; tested on Raspberry Pi 4)
- Arduino Mega 2560
- Arduino Nano
- 2 × USB webcams
- MPU-6050 IMU module (connected to Arduino Nano via I²C)
- 6 × hobby servo motors (PWM, e.g. MG996R or SG90 class)
- Analogue potentiometer (connected to Arduino Mega A8)
- SSD1306 128×64 OLED display (I²C, address 0x3C)
- 3 × LEDs (red, yellow, green) with appropriate resistors (connected to Nano pins 4, 5, 6)
- Pre-calibrated camera calibration matrices (`.npy` files — see below)

### Software Dependencies

#### Raspberry Pi

Ensure Python 3.7+ is installed, then install the required packages:

```bash
pip install opencv-python mediapipe numpy pyserial
```

#### Arduino IDE

Install the following libraries via **Sketch → Include Library → Manage Libraries**:

- `Servo` (built-in)
- `Wire` (built-in)
- `Adafruit GFX Library` by Adafruit
- `Adafruit SSD1306` by Adafruit

The Arduino Nano sketch uses only the built-in `Wire` library — no additional installs required.

### Camera Calibration Files

The Raspberry Pi script requires four NumPy binary files containing the intrinsic camera matrices and distortion coefficients for both cameras. These must be generated in advance using a standard checkerboard calibration procedure (e.g. with OpenCV or MATLAB). Place the following files in the same directory as `Raspberry_Pi_Code.py`:

```
side_camera_matrix.npy
side_distortion.npy
top_camera_matrix.npy
top_distortion.npy
```

Pre-computed distortion matrices for the cameras used during development are provided in `Distortion Matrices.zip`.

### Hardware Wiring Summary

**Arduino Mega servo pin assignments:**

| Servo | Joint | Pin |
|-------|-------|-----|
| `servo_joint0` | Base rotation | 3 |
| `servo_joint1` | Shoulder | 4 |
| `servo_joint2` | Elbow | 5 |
| `servo_pitch` | Wrist pitch | 6 |
| `servo_roll` | Wrist roll | 7 |
| `servo_gripper` | Gripper | 8 |

Analogue feedback potentiometers for joints 1 and 2 connect to A1 and A2 respectively. The speed-control potentiometer connects to A8. The SSD1306 OLED connects via I²C (SDA/SCL).

**Arduino Nano:**

| Component | Pin |
|-----------|-----|
| MPU-6050 | SDA/SCL (I²C) |
| Green LED | 4 |
| Yellow LED | 5 |
| Red LED | 6 |

---

## How to Run the Software

### Step 1 — Flash the Arduino sketches

1. Open `Arduino_Nano_Code.ino` in the Arduino IDE.
2. Select **Board: Arduino Nano** and the correct COM port.
3. Upload the sketch. On power-up, the yellow LED lights during IMU calibration; the green LED indicates successful transmission.
4. Open `Arduino_Mega_Code.ino` in the Arduino IDE.
5. Select **Board: Arduino Mega 2560** and its COM port.
6. Upload the sketch. The OLED will display `System Ready` on startup.

### Step 2 — Connect cameras

Plug both USB cameras into the Raspberry Pi. Identify which device index corresponds to each view by testing with:

```python
import cv2
cap = cv2.VideoCapture(1)  # try 0, 1, 2 ...
ret, frame = cap.read()
cv2.imshow("test", frame)
cv2.waitKey(0)
```

Update `cv2.VideoCapture(2)` (side) and `cv2.VideoCapture(1)` (top) in `Raspberry_Pi_Code.py` to match your system.

### Step 3 — Update serial port assignments

In `Raspberry_Pi_Code.py`, update the COM port strings to match your system:

```python
arduino = serial.Serial('COM3', 115200, timeout=0)      # Arduino Mega
arduino_nano = serial.Serial('COM6', 115200, timeout=0) # Arduino Nano
```

On Linux/macOS these will typically be `/dev/ttyUSB0`, `/dev/ttyACM0`, etc.

### Step 4 — Run the vision pipeline

With both Arduinos connected and powered, run:

```bash
python Raspberry_Pi_Code.py
```

Two OpenCV windows will open — one for the side camera and one for the top camera — showing the live MediaPipe hand skeleton overlay and the estimated X, Y, Z coordinates in centimetres. Press **Q** to quit cleanly.

---

## Technical Details

### 3D Hand Position Estimation

Two cameras are placed orthogonally. Each frame is undistorted using the preloaded calibration matrices via `cv2.undistort`. MediaPipe Hands locates 21 hand landmarks per frame; landmark 0 (wrist base) is used as the hand reference point.

The pixel coordinates `(px, py)` are back-projected to metric coordinates using the pinhole camera model:

```
Y = (px - cx) * Z_depth / fx     (side camera → lateral Y)
Z = -((py - cy) * Z_depth / fy)  (side camera → height Z)

X = (px - cx) * Z_depth / fx     (top camera → depth X)
```

where `(cx, cy)` is the principal point and `(fx, fy)` are the focal lengths, all loaded from the calibration matrices. The depth value fed into the top-camera projection is updated each frame using the Z estimate from the side camera, and vice versa.

Finger-spread distance `D` is estimated as the pixel distance between the index fingertip (landmark 8) and thumb tip (landmark 4), normalised by the focal length magnitude and scaled by the known hand depth:

```
D = pixel_distance * known_depth / sqrt(fx² + fy²)
```

### Complementary Filter (IMU — Arduino Nano)

The Arduino Nano fuses accelerometer and gyroscope data from the MPU-6050 to produce stable pitch (θ) and roll (φ) estimates. A complementary filter is used:

```
θ = K_fusion * θ_gyro_integral + (1 - K_fusion) * θ_accelerometer
```

where `K_fusion = 0.94`. The gyroscope integral is corrected by two calibration offsets, C1 (mean gyro bias) and C2 (integral drift), computed at startup from 300 static samples. Angles are averaged over N=100 samples before transmission to reduce noise. An LED indicates calibration (yellow) or active transmission (green).

### Inverse Kinematics (Arduino Mega)

The 3-DOF arm (base, shoulder, elbow) uses geometric inverse kinematics. Given a target position (X, Y, Z) and link lengths L1 = 31 cm, L2 = 17 cm:

**Base rotation (φ₁):**
```
φ₁ = atan2(X, Y)
```

**Shoulder angle (φ₂):**
```
φ₂ = acos((r² + L1² - L2²) / (2 * L1 * r)) + asin(Z / r)
```
where r = sqrt(X² + Y² + Z²).

**Elbow angle (φ₃):**
```
φ₃ = 180° - acos((L1² + L2² - r²) / (2 * L1 * L2))
```

Each computed angle is mapped to the specific servo's calibrated output range using empirically determined linear coefficients (e.g. `servo0 = φ₁ * (13/18) + 25`).

### Exponential Weighted Moving Average (EWMA) Smoothing

To suppress jitter, incoming X, Y, Z, θ, and φ values are filtered using EWMA with configurable smoothing factors (α = 0.5 for position; α = 0.5 for wrist). A dead-zone threshold prevents updates smaller than 0.5 cm (position) or 2° (wrist) from propagating, reducing servo hunting.

### Rate Limiting

The servos have a maximum step size per control loop for each joint (1–20°), acting as a hardware speed limit to prevent violent motion.

### Design Assumptions

- The hand depth from the side camera is assumed to be approximately 80 cm (`X_constant`) at initialisation, and from the top camera 52 cm (`Z_constant`). These values are updated iteratively from each camera's own estimate.
- The workspace is constrained to a sphere of radius 45 cm to prevent singularities.
- The wrist pitch servo compensation accounts for shoulder joint angle to maintain a stable end-effector orientation.

---

## Known Issues and Future Improvements

**Known issues:**

- Camera index assignments (`cv2.VideoCapture`) are hardcoded and must be manually verified on each machine.
- Serial COM port strings are Windows-format (`COM3`, `COM6`) and require manual editing on Linux/macOS.
- The Y-coordinate is estimated independently by both cameras and a discrepancy (`Y_error`) is logged but not corrected — the top-camera estimate is used exclusively.
- Camera calibration `.npy` files must exist before running; the script will crash if they are absent.
- The MATLAB positional experiment scripts (`MATLAB_Positional_Experiment.zip`) and raw CSV data (`Raw Data Files_CSV.zip`) are not documented and cannot be run without additional context.
- Calibration error detection in the Nano (`calibration_error_flag`) is computed but never set to 1, so errors are silently ignored.

**Potential future improvements:**

- Automate camera index detection by comparing frame content rather than relying on fixed indices.
- Add cross-validation of the two Y estimates to improve positional accuracy.
- Implement a proper Kalman filter in place of the complementary filter for the IMU.
- Replace hardcoded workspace constants with a calibration routine that measures actual camera-to-workspace geometry.
- Add a GUI on the Raspberry Pi for real-time monitoring of joint angles and system status.
- Extend to a full 6-DOF kinematics model to account for wrist offsets in the position solution.
- Port to ROS2 for modularity and compatibility with standard robotics tooling.

---

## Repository Contents

| File | Description |
|------|-------------|
| `Raspberry_Pi_Code.py` | Main vision and communication pipeline (Python) |
| `Arduino_Mega_Code.ino` | Robot controller: IK, servo driving, OLED display (C++) |
| `Arduino_Nano_Code.ino` | Glove IMU reader: complementary filter, serial output (C++) |
| `Distortion Matrices.zip` | Pre-computed camera calibration `.npy` files |
| `MATLAB_Positional_Experiment.zip` | MATLAB scripts used for positional accuracy experiments |
| `Raw Data Files_CSV.zip` | Raw experimental data in CSV format |

---

## License

This project is released under the MIT License. See `LICENSE` for details.
