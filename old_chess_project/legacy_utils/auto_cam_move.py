import cv2
import time
import signal
import sys

# Import Yahboom DOFBOT SDK if installed
try:
    from Arm_Lib import Arm_Device
    arm = Arm_Device()
except ImportError:
    arm = None
    print("Arm control library not found. Movement will be skipped.")

# === Signal handler for Ctrl+C clean exit ===
def signal_handler(sig, frame):
    print("\n[INFO] Exiting...")
    if arm:
        arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 500)
    cap.release()
    cv2.destroyAllWindows()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# === Open the camera ===
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
if not cap.isOpened():
    print("[ERROR] Cannot open camera")
    sys.exit()

print("[INFO] Starting camera and arm motion loop...")

# === Start autonomous loop ===
start_time = time.time()
while True:
    ret, frame = cap.read()
    if not ret:
        print("[WARNING] Frame capture failed.")
        break

    # Show frame
    cv2.imshow("Live Camera", frame)

    # Every 3 seconds, do a movement
    if arm and int(time.time() - start_time) % 6 == 0:
        arm.Arm_serial_servo_write6(90, 50, 90, 90, 90, 90, 500)
        time.sleep(0.5)
        arm.Arm_serial_servo_write6(90, 130, 90, 90, 90, 90, 500)
        time.sleep(0.5)

    # Exit with ESC
    if cv2.waitKey(1) & 0xFF == 27:
        break

# === Cleanup ===
signal_handler(None, None)
