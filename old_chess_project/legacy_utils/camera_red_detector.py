import cv2
import numpy as np
import subprocess

def detect_red_object(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define red range in HSV
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    # Create masks and combine them
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    return red_mask

def move_arm():
    try:
        subprocess.run(["python3", "wave.py"])
    except Exception as e:
        print("Failed to run wave.py:", e)

# Open camera
cap = cv2.VideoCapture(0)  # or try 1 if you have multiple cameras

if not cap.isOpened():
    print("❌ Cannot open camera")
    exit()

cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Camera", 640, 480)

triggered = False

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Can't receive frame. Exiting...")
        break

    mask = detect_red_object(frame)
    red_pixels = cv2.countNonZero(mask)

    # Draw a red mask overlay
    result = cv2.bitwise_and(frame, frame, mask=mask)
    combined = np.hstack((frame, result))
    cv2.imshow("Camera", combined)

    # Auto trigger on red detection
    if red_pixels > 1000 and not triggered:
        print("🚨 Red object detected! Moving arm...")
        move_arm()
        triggered = True

    key = cv2.waitKey(1) & 0xFF
    if key == ord('w'):
        print("🔧 Manual wave triggered.")
        move_arm()
    elif key == ord('q'):
        break
    elif key == ord('r'):
        triggered = False
        print("🔁 Reset auto trigger.")

cap.release()
cv2.destroyAllWindows()
