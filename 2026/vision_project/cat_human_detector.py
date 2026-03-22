#!/usr/bin/env python3
"""
DOFBOT Vision Patrol 2026 - YOLOv8 Edition
- Prioritizes cats over humans
- Cat detected: extend arm and pinch
- Human detected: wave
- Web preview at http://10.0.0.173:8080/
"""

import sys, time, pathlib, threading, signal
import numpy as np, cv2
from datetime import datetime
from http.server import BaseHTTPRequestHandler, HTTPServer
import logging

# === LOGGING ===
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
log = logging.getLogger(__name__)

# === PATHS ===
HOME     = pathlib.Path.home()
WEIGHTS  = HOME / "yolov5" / "yolov8n.pt"

# === ARM ===
try:
    from Arm_Lib import Arm_Device
    arm = Arm_Device()
    ARM_CONNECTED = True
    log.info("✅ Arm connected!")
except Exception:
    arm = None
    ARM_CONNECTED = False
    log.warning("⚠️ Arm not found. Movement skipped.")

# === YOLOv8 ===
YOLO_AVAILABLE = False
try:
    from ultralytics import YOLO
    model = YOLO(str(WEIGHTS))
    YOLO_AVAILABLE = True
    log.info(f"✅ YOLOv8 loaded: {WEIGHTS.name}")
except Exception as e:
    log.warning(f"⚠️ YOLOv8 failed: {e}")
    model = None

# === HOG FALLBACK (persons only) ===
HOG_AVAILABLE = False
try:
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    HOG_AVAILABLE = True
    log.info("✅ HOG backup detector ready")
except Exception as e:
    log.warning(f"⚠️ HOG failed: {e}")

# === SETTINGS ===
CONF_THRESH   = 0.35   # minimum confidence to count a detection (0-1)
CONSEC_NEEDED = 2      # consecutive frames needed before triggering action
COOLDOWN      = 8.0    # seconds to wait between actions
SCAN_SPEED    = 400    # ms per servo move during scanning
ACTION_SPEED  = 500    # ms per servo move during actions
SCAN_ANGLES   = list(range(20, 161, 10))  # [20, 30, 40 ... 160]
WEB_FPS_CAP   = 15     # max frames per second sent to browser

# === ARM POSITIONS ===
# [servo1_base, servo2_shoulder, servo3_elbow,
#  servo4_wrist_pitch, servo5_wrist_roll, servo6_gripper]
POS_HOME      = [90,  90,  90,  0, 90,  90]
POS_WAVE_UP   = [90,  45,  90, 45, 90,  90]
POS_WAVE_DOWN = [90,  70,  90, 70, 90,  90]
POS_CAT_REACH = [90,  30,  60, 30, 90,  90]
POS_CAT_PINCH = [90,  30,  60, 30, 90, 160]

# === ARM HELPERS ===
def move_all(position, speed=ACTION_SPEED):
    """Move all 6 servos at once to a position."""
    if not ARM_CONNECTED:
        return
    arm.Arm_serial_servo_write6(*position, speed)
    time.sleep(speed / 1000 + 0.1)

def move_servo(servo_id, angle, speed=SCAN_SPEED):
    """Move a single servo to an angle."""
    if not ARM_CONNECTED:
        return
    angle = max(0, min(180, int(angle)))
    arm.Arm_serial_servo_write(servo_id, angle, speed)
    time.sleep(speed / 1000 + 0.05)

def go_home():
    log.info("🏠 Going home")
    move_all(POS_HOME, 1000)

# === ACTIONS ===
def wave():
    """Wave at a human — 3 up/down cycles."""
    log.info("👋 Waving at human!")
    for _ in range(3):
        move_all(POS_WAVE_UP)
        move_all(POS_WAVE_DOWN)
    go_home()

def extend_and_pinch():
    """Reach forward toward a cat and close the claw."""
    log.info("🐱 Cat! Extending and pinching!")
    move_all(POS_CAT_REACH)
    time.sleep(0.3)
    move_all(POS_CAT_PINCH)
    time.sleep(1.0)
    go_home()

# === DETECTION ===
def detect_yolo(frame):
    """
    Run YOLOv8 on a frame.
    Returns: ('cat' | 'person' | None, annotated_frame)
    Cat is prioritized over person.
    COCO class IDs: 0=person, 15=cat
    """
    if not YOLO_AVAILABLE:
        return None, frame
    try:
        results = model(frame, conf=CONF_THRESH, verbose=False)[0]

        found_cat    = False
        found_person = False

        for box in results.boxes:
            cls   = int(box.cls[0])
            conf  = float(box.conf[0])
            label = model.names[cls]
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]

            # Orange for cat, green for person
            color = (0, 165, 255) if label == "cat" else (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
            cv2.putText(frame, f"{label} {conf:.2f}",
                        (x1, max(0, y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            if label == "cat":    found_cat    = True
            if label == "person": found_person = True

        # Cat takes priority
        if found_cat:    return "cat",    frame
        if found_person: return "person", frame
        return None, frame

    except Exception as e:
        log.error(f"YOLOv8 error: {e}")
        return None, frame

def detect_hog(frame):
    """
    HOG fallback — only detects persons.
    Used automatically if YOLOv8 fails to load.
    """
    if not HOG_AVAILABLE:
        return None, frame
    try:
        small = cv2.resize(frame, (320, 240))
        rects, weights = hog.detectMultiScale(
            small, winStride=(4, 4), padding=(8, 8), scale=1.05
        )
        if len(rects) == 0:
            return None, frame

        best = rects[np.argmax(weights)]
        x, y, w, h = [int(v * 2) for v in best]
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 3)
        cv2.putText(frame, "person (HOG)", (x, max(0, y-10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        return "person", frame
    except Exception as e:
        log.error(f"HOG error: {e}")
        return None, frame

def detect(frame):
    """Use YOLOv8 if available, otherwise fall back to HOG."""
    if YOLO_AVAILABLE:
        return detect_yolo(frame)
    return detect_hog(frame)

# === WEB PREVIEW ===
class FrameBuffer:
    """
    Thread-safe container for the latest JPEG frame.
    Main loop writes here, web server reads from here.
    Lock prevents both threads accessing it simultaneously.
    """
    def __init__(self):
        self.lock  = threading.Lock()
        self.frame = None

    def update(self, bgr_frame):
        ok, buf = cv2.imencode(".jpg", bgr_frame,
                               [cv2.IMWRITE_JPEG_QUALITY, 50])
        if ok:
            with self.lock:
                self.frame = buf.tobytes()

    def get(self):
        with self.lock:
            return self.frame

frame_buffer = FrameBuffer()

class StreamHandler(BaseHTTPRequestHandler):
    """Streams MJPEG to browser — sequence of JPEGs sent continuously."""
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        min_interval = 1.0 / WEB_FPS_CAP
        last_sent = 0
        try:
            while True:
                now = time.time()
                if now - last_sent < min_interval:
                    time.sleep(0.01)
                    continue
                jpg = frame_buffer.get()
                if jpg is None:
                    time.sleep(0.01)
                    continue
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                self.wfile.write(jpg + b"\r\n")
                last_sent = now
        except Exception:
            pass

    def log_message(self, *args):
        pass

def start_web_server(port=8080):
    """Start MJPEG server in a background thread."""
    server = HTTPServer(("0.0.0.0", port), StreamHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    log.info(f"🌐 Web preview → http://10.0.0.173:{port}/")

# === CLEAN EXIT ===
def signal_handler(sig, frame_arg):
    log.info("👋 Exiting...")
    go_home()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# === MAIN ===
if __name__ == "__main__":
    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        log.error("❌ Cannot open camera")
        sys.exit(1)

    start_web_server()
    go_home()

    log.info("🚀 Patrol started!")
    log.info("🌐 Open http://10.0.0.173:8080/ in your browser")

    # === STATE ===
    scanning       = True   # is arm currently scanning?
    scan_index     = 0      # current position in SCAN_ANGLES list
    scan_direction = 1      # 1 = moving right, -1 = moving left
    last_action    = 0      # timestamp of last triggered action
    consec         = 0      # consecutive frames with same detection
    last_label     = None   # what was detected last frame

    while True:
        ret, frame = cap.read()
        if not ret:
            log.warning("⚠️ Camera read failed")
            time.sleep(0.05)
            continue

        # run detection
        label, frame = detect(frame)

        # count consecutive detections of same label
        if label and label == last_label:
            consec += 1
        else:
            consec     = 0
            last_label = label

        # trigger action if confirmed and cooldown passed
        now = time.time()
        if consec >= CONSEC_NEEDED and now - last_action > COOLDOWN:
            last_action = now
            consec      = 0
            scanning    = False

            if label == "cat":
                threading.Thread(target=extend_and_pinch, daemon=True).start()
            elif label == "person":
                threading.Thread(target=wave, daemon=True).start()

            # resume scanning after 10 seconds
            def resume():
                global scanning
                time.sleep(10.0)
                scanning = True
            threading.Thread(target=resume, daemon=True).start()

        # advance scan position
        if scanning:
            move_servo(1, SCAN_ANGLES[scan_index])
            scan_index += scan_direction
            if scan_index >= len(SCAN_ANGLES):
                scan_index     = len(SCAN_ANGLES) - 2
                scan_direction = -1
            elif scan_index < 0:
                scan_index     = 1
                scan_direction = 1

        # draw HUD
        ts = datetime.now().strftime("%H:%M:%S")
        cv2.putText(frame, f"DOFBOT 2026 YOLOv8 | {ts}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame,
                    f"Detected: {label or 'nothing'} ({consec}/{CONSEC_NEEDED})",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame,
                    f"Scanning: {'YES' if scanning else 'NO (acting)'}",
                    (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        frame_buffer.update(frame)
