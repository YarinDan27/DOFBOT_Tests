import cv2
import time
import signal
import sys

# Try importing Yahboom DOFBOT SDK
try:
    from Arm_Lib import Arm_Device
    arm = Arm_Device()
except ImportError:
    arm = None
    print("❌ Arm control library not found. Rotation skipped.")

# Clean shutdown
def signal_handler(sig, frame):
    print("\n[INFO] Stopping 360 scan...")
    if arm:
        # Return to center position on exit
        arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 800)
    cap.release()
    cv2.destroyAllWindows()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Open camera
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
if not cap.isOpened():
    print("❌ Cannot open camera")
    sys.exit()

cv2.namedWindow("360° Scan", cv2.WINDOW_NORMAL)
cv2.resizeWindow("360° Scan", 640, 480)

print("[INFO] Starting 360-degree scan...")

# Scan range for base rotation (servo 1)
base_angles = list(range(60, 121, 10)) + list(range(120, 59, -10))  # 60° ➝ 120° ➝ 60° repeat

while True:
    for angle in base_angles:
        if arm:
            # Move base (servo1 = index 0)
            arm.Arm_serial_servo_write6(angle, 90, 90, 90, 90, 90, 600)
            print(f"[INFO] Rotating to {angle}°")
            time.sleep(0.8)  # Let camera settle

        # Capture and display frame
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Frame capture failed.")
            continue

        cv2.imshow("360° Scan", frame)

        # Exit on ESC
        if cv2.waitKey(1) & 0xFF == 27:
            signal_handler(None, None)
