import cv2
import torch
import time
import signal
import sys
from pathlib import Path
from datetime import datetime

# === Paths ===
HOME       = Path.home()
YOLO_DIR   = HOME / "yolov5"
WEIGHTS    = YOLO_DIR / "yolov5s_v3.1.pt"

# === Try importing DOFBOT SDK ===
try:
    from Arm_Lib import Arm_Device
    arm = Arm_Device()
    ARM_CONNECTED = True
    print("✅ Arm connected!")
except ImportError:
    arm = None
    ARM_CONNECTED = False
    print("⚠️ Arm not found. Movement will be skipped.")

# === Load YOLOv5 ===
print("🔄 Loading YOLOv5 model...")
sys.path.insert(0, str(YOLO_DIR))
from models.experimental import attempt_load
from utils.general import non_max_suppression
import torch

model = attempt_load(str(WEIGHTS), map_location='cpu')
model.eval()
print("✅ Model loaded!")

# === Arm positions ===
# [s1_base, s2_shoulder, s3_elbow, s4_wrist_pitch, s5_wrist_roll, s6_gripper]
POS_HOME      = (90, 90,  90,  0, 90,  90)
POS_WAVE_UP   = (90, 45,  90,  45, 90,  90)
POS_WAVE_DOWN = (90, 70,  90,  70, 90,  90)
POS_CAT_REACH = (90, 30,  60,  30, 90,  90)
POS_CAT_PINCH = (90, 30,  60,  30, 90, 160)

SCAN_ANGLES  = [45, 67, 90, 112, 135]
SCAN_SPEED   = 800
ACTION_SPEED = 600

# === Movement helpers ===
def move_all(position, speed=ACTION_SPEED):
    if not ARM_CONNECTED:
        print(f"⚠️ (no arm) would move to {position}")
        return
    arm.Arm_serial_servo_write6(*position, speed)
    time.sleep(speed / 1000 + 0.1)

def move_servo(servo_id, angle, speed=ACTION_SPEED):
    if not ARM_CONNECTED:
        print(f"⚠️ (no arm) servo {servo_id} → {angle}°")
        return
    arm.Arm_serial_servo_write(servo_id, angle, speed)
    time.sleep(speed / 1000 + 0.1)

def go_home():
    print("🏠 Returning home...")
    move_all(POS_HOME, SCAN_SPEED)

# === Actions ===
def wave():
    print("👋 Human detected — waving!")
    for _ in range(3):
        move_all(POS_WAVE_UP)
        move_all(POS_WAVE_DOWN)
    go_home()

def extend_and_pinch():
    print("🐱 Cat detected — extending and pinching!")
    move_all(POS_CAT_REACH)
    time.sleep(0.3)
    move_all(POS_CAT_PINCH)
    time.sleep(1.0)
    go_home()

# === Detection ===
def get_labels(frame):
    """Run YOLOv5 on a frame, return list of detected label names."""
    import numpy as np
    img = cv2.resize(frame, (640, 640))
    img = img[:, :, ::-1].transpose(2, 0, 1)
    img = np.ascontiguousarray(img)
    img = torch.from_numpy(img).float() / 255.0
    img = img.unsqueeze(0)

    with torch.no_grad():
        pred = model(img)[0]
    pred = non_max_suppression(pred, conf_thres=0.50)

    labels = []
    for det in pred:
        if det is not None and len(det):
            for *_, conf, cls in det:
                labels.append(model.names[int(cls)])
    return labels

def scan_angle(angle):
    """Point base to angle, capture frame, run detection."""
    move_servo(1, angle, SCAN_SPEED)
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Camera read failed")
        return None, None
    labels = get_labels(frame)
    print(f"👁️  Angle {angle}° → {labels if labels else 'nothing'}")
    if 'person' in labels:
        return 'person', frame
    if 'cat' in labels:
        return 'cat', frame
    return None, frame

# === Clean exit ===
def signal_handler(sig, frame_arg):
    print("\n[INFO] Exiting...")
    go_home()
    if 'cap' in globals() and cap.isOpened():
        cap.release()
        cv2.destroyAllWindows()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# === Main ===
if __name__ == "__main__":
    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    if not cap.isOpened():
        print("❌ Cannot open camera")
        sys.exit()

    print("🚀 Starting DOFBOT vision patrol...")
    go_home()

    while True:
        for angle in SCAN_ANGLES:
            print(f"\n🔍 Scanning at {angle}°")
            detected, frame = scan_angle(angle)

            if detected == 'person':
                wave()
                break
            elif detected == 'cat':
                extend_and_pinch()
                break

        time.sleep(0.1)
