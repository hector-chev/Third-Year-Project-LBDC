
import cv2
import mediapipe as mp
import numpy as np
import math
import serial
import time

# -------------------------------
# Load calibration
# -------------------------------
K_side = np.load("side_camera_matrix.npy")
dist_side = np.load("side_distortion.npy")
K_top = np.load("top_camera_matrix.npy")
dist_top = np.load("top_distortion.npy")

fx_side = K_side[0,0]
fy_side = K_side[1,1]
cx_side = K_side[0,2]
cy_side = K_side[1,2]

fx_top = K_top[0,0]
fy_top = K_top[1,1]
cx_top = K_top[0,2]
cy_top = K_top[1,2]

# Estimated Z distance of hand (cm)
X_HAND_prev = 0
Y_HAND_prev = 0
Z_HAND_prev = 0
X_constant = 80
Z_constant = 52
X_message = 0
Y_message = 0
Z_message = 0
d_message = 10

# -------------------------------
# MediaPipe setup
# -------------------------------
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    max_num_hands=2,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)

mp_draw = mp.solutions.drawing_utils

# -------------------------------
# Camera
# -------------------------------
cap_side = cv2.VideoCapture(2)
cap_top = cv2.VideoCapture(1)

if not cap_side.isOpened():
    print("ERROR: Side camera not found")
if not cap_top.isOpened():
    print("ERROR: Top camera not found")

print("Press Q to quit")

arduino = None
arduino_nano = None
try:
    arduino = serial.Serial('COM3', 115200, timeout=0)
    time.sleep(2)
    print("Arduino Uno connected.")
except:
    print("Arduino Uno not found. Running without serial output/input.")


try:
    arduino_nano = serial.Serial('COM6', 115200, timeout=0)
    time.sleep(2)
    print("Arduino Nano connected.")
except:
    print("Arduino Nano not found. Running without serial output/input.")

theta_glove = 0
phi_glove = 0
t_glove = 0
while True:
    line = None
    if arduino_nano and arduino_nano.is_open:
        try:
            while arduino_nano.in_waiting > 0:
                line = arduino_nano.readline().decode().strip()

            if line.startswith("T:") and ",P:" in line and ",t:" in line:
                t_part = line.split(",t:")[1]
                t_glove = t_part.strip()
                print(t_glove)
                main_part = line.split(",t:")[0]
                parts = main_part.split(",P:")
                theta_glove = float(parts[0].replace("T:", ""))
                phi_glove = float(parts[1])
        except:
            pass

        
    ret_side, frame_side = cap_side.read()
    ret_top, frame_top = cap_top.read()
    #print(f"Side camera: {ret_side}, Top camera: {ret_top}")
    if not ret_side:
        print("Side camera failed")
        continue

    if not ret_top:
        print("Top camera failed")
        continue

    # -------------------------------
    # Undistort frame using calibration
    # -------------------------------
    frame_side = cv2.undistort(frame_side, K_side, dist_side)
    frame_top = cv2.undistort(frame_top, K_top, dist_top)

    frame_side = cv2.flip(frame_side, 1)
    frame_top = cv2.flip(frame_top, 1)

    h_side, w_side, _ = frame_side.shape
    h_top, w_top, _ = frame_top.shape

    rgb_side = cv2.cvtColor(frame_side, cv2.COLOR_BGR2RGB)
    rgb_top = cv2.cvtColor(frame_top, cv2.COLOR_BGR2RGB)

    results_side = hands.process(rgb_side)
    results_top = hands.process(rgb_top)

    if results_side.multi_hand_landmarks:
        
        hand_side = results_side.multi_hand_landmarks[0]
        
        mp_draw.draw_landmarks(frame_side, hand_side, mp_hands.HAND_CONNECTIONS)
        #SIDE
        # Use index fingertip
        index_side = hand_side.landmark[8]
        thumb_side = hand_side.landmark[4]
        hand_base_side = hand_side.landmark[0]
        #lm0 = hand_side.landmark[0]
        #lm5 = hand_side.landmark[5]
        #pitch = (180/3.14159265) * math.atan((lm5.x - lm0.x)/(lm5.y - lm0.y))
        thumb_px_side = np.array([int(thumb_side.x*w_side), int(thumb_side.y*h_side)])
        index_px_side = np.array([int(index_side.x*w_side), int(index_side.y*h_side)])
        lm_side = np.array([int(hand_base_side.x*w_side), int(hand_base_side.y*h_side)])#(index_px_side + thumb_px_side)/2
        px_side = int(lm_side[0]) # represents Y in world frame
        py_side = int(lm_side[1]) # represents Z in world frame
        distance_side = np.linalg.norm(index_px_side - thumb_px_side)
        norm_fx_fy_side = math.sqrt(fx_side**2 + fy_side**2)
        distance_side = distance_side*(X_constant + X_HAND_prev)/norm_fx_fy_side
        
        cv2.line(frame_side, tuple(thumb_px_side), tuple(index_px_side), (255,0,0), 2)
        cv2.circle(frame_side, (px_side, py_side), 8, (0,255,255), -1)

        # Convert pixel -> real world
        Y_side = (px_side - cx_side) * (X_constant + X_HAND_prev) / fx_side
        Z_side = -((py_side - cy_side) * (X_constant + X_HAND_prev) / fy_side)
        Z_message = Z_side
        Z_HAND_prev = Z_side
        
        #SIDE TEXT
        cv2.putText(frame_side, f"Z: {Z_side:.2f} cm", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        cv2.putText(frame_side, f"Y side: {Y_side:.2f} cm", (20,80), cv2.FONT_HERSHEY_SIMPLEX, 0.8,(0,255,255), 2)
        cv2.putText(frame_side, f"d_side: {distance_side:.2f} cm", (20,120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        #cv2.putText(frame_side, f"pitch: {pitch:.2f} deg", (20,160), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

        
    if results_top.multi_hand_landmarks:
        
        hand_top = results_top.multi_hand_landmarks[0]

        mp_draw.draw_landmarks(frame_top, hand_top, mp_hands.HAND_CONNECTIONS)
        #TOP
        index_top = hand_top.landmark[8]
        thumb_top = hand_top.landmark[4]
        hand_base_top = hand_top.landmark[0]
        thumb_px_top = np.array([int(thumb_top.x*w_top), int(thumb_top.y*h_top)])
        index_px_top = np.array([int(index_top.x*w_top), int(index_top.y*h_top)])
        lm_top = np.array([int(hand_base_top.x*w_top), int(hand_base_top.y*h_top)])#(index_px_top + thumb_px_top)/2
        px_top = int(lm_top[0]) 
        py_top = int(lm_top[1])
        distance_top = np.linalg.norm(index_px_top - thumb_px_top)
        norm_fx_fy_top = math.sqrt(fx_top**2 + fy_top**2)
        distance_top = distance_top*(Z_constant - Z_HAND_prev)/norm_fx_fy_top
        cv2.line(frame_top, tuple(thumb_px_top), tuple(index_px_top), (255,0,0), 2)
        cv2.circle(frame_top, (px_top, py_top), 8, (0,255,255), -1)

        # Convert pixel -> real world
        X_top = (px_top - cx_top) * (Z_constant - Z_HAND_prev) / fx_top
        X_message = X_top
        Y_top = (py_top - cy_top) * (Z_constant + Z_HAND_prev) / fy_top
        Y_message = Y_top + 5
        X_HAND_prev = X_top

        #TOP TEXT
        cv2.putText(frame_top, f"X: {X_top:.2f} cm", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        cv2.putText(frame_top, f"Y top: {Y_top:.2f} cm", (20,80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        cv2.putText(frame_top, f"d_top: {distance_top:.2f} cm", (20,120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

    if results_side.multi_hand_landmarks and results_top.multi_hand_landmarks:
        
        Y_error = Y_top - Y_side
        distance_norm = math.sqrt(distance_side**2 + distance_top**2)
        d_message = distance_norm
        print(f"distance: {distance_norm}")

    if arduino and arduino.is_open:
        msg = f"{X_message:.2f},{Y_message:.2f},{Z_message:.2f},{d_message:.2f},{theta_glove:.2f},{phi_glove:.2f},{t_glove}\n"
        print("Sending: ", msg)
        arduino.write(msg.encode())

    cv2.imshow("Hand XY Tracking, SIDE", frame_side)
    cv2.imshow("Hand XY Tracking, TOP", frame_top)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap_side.release()
cap_top.release()
cv2.destroyAllWindows()
if arduino:
    arduino.close()
if arduino_nano:
    arduino_nano.close()
