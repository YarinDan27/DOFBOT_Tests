#!/usr/bin/env python3
"""
DOFBOT Vision Patrol 2026
- Prioritizes cats over humans
- Cat detected: extend arm and pinch
- Human detected: wave
- Web preview at http://10.0.0.173:8080/
"""

import os, sys, time, pathlib, threading, signal
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
YOLO_DIR = HOME / "yolov5"
WEIGHTS  = YOLO_DIR / "yolov5s_v3.1.pt"

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

# === YOLO ===
YOLO_AVAILABLE = False
try:
    import torch
    sys.path.insert(0, str(YOLO_DIR))
    from models.experimental import attempt_load
    from utils.general import non_max_suppression, scale_coords
    from utils.datasets import letterbox
    from utils.torch_utils import select_device

    device = select_device("0" if torch.cuda.is_available() else "cpu")
    half   = device.type != "cpu"
    model  = attempt_load(str(WEIGHTS), map_location=device)
    if half: model.half()
    model.eval()
    YOLO_AVAILABLE = True
    log.info(f"✅ YOLOv5 loaded on {device} | half precision: {half}")
except Exception as e:
    log.warning(f"⚠️ YOLOv5 failed: {e}")
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
CONF_THRESH   = 0.25   # minimum confidence to count a detection
IOU_THRESH    = 0.45   # overlap threshold for deduplicating boxes
CONSEC_NEEDED = 1      # how many frames in a row before triggering action
COOLDOWN      = 8.0    # seconds to wait between actions
SCAN_SPEED    = 400    # ms per servo move during scanning
ACTION_SPEED  = 500    # ms per servo move during actions
SCAN_ANGLES   = list(range(20, 161, 10))  # [20, 30, 40 ... 160]
WEB_FPS_CAP   = 25     # max frames per second sent to browser

# === ARM POSITIONS ===
# Each list is [servo1_base, servo2_shoulder, servo3_elbow,
#               servo4_wrist_pitch, servo5_wrist_roll, servo6_gripper]
POS_HOME      = [90,  90,  90, 0, 90,  90]
POS_WAVE_UP   = [90,  45,  90, 45, 90,  90]
POS_WAVE_DOWN = [90,  70,  90, 70, 90,  90]
POS_CAT_REACH = [90,  30,  60, 30, 90,  90]
POS_CAT_PINCH = [90,  90,  60, 30, 90, 160]

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
    Run YOLOv5 on a frame.
    Returns: ('cat' | 'person' | None, annotated_frame)
    Cat is checked first because cats are prioritized.
    YOLO class IDs: 0=person, 15=cat
    """
    if not YOLO_AVAILABLE:
        return None, frame
    try:
        # Resize frame to 640x640 using letterbox (adds padding to keep aspect ratio)
        img = letterbox(frame, 640)[0]
        # Convert BGR (OpenCV format) to RGB, then to CHW (channels, height, width)
        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)
        # Convert to torch tensor and normalize to 0-1
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()
        img /= 255.0
        img = img.unsqueeze(0)  # add batch dimension: (1, C, H, W)

        with torch.no_grad():
            pred = model(img, augment=False)[0]

        # Filter detections: only person(0) and cat(15), above confidence threshold
        det = non_max_suppression(pred, CONF_THRESH, IOU_THRESH,
                                  classes=[0, 15])[0]

        if det is None or len(det) == 0:
            return None, frame

        # Scale bounding boxes back from 640x640 to original frame size
        det[:, :4] = scale_coords(img.shape[2:], det[:, :4], frame.shape).round()

        found_cat    = False
        found_person = False

        for *xyxy, conf, cls in det:
            x1, y1, x2, y2 = [int(v) for v in xyxy]
            label = model.names[int(cls)]
            # Orange box for cat, green box for person
            color = (0, 165, 255) if label == "cat" else (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
            cv2.putText(frame, f"{label} {conf:.2f}",
                        (x1, max(0, y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            if label == "cat":    found_cat    = True
            if label == "person": found_person = True

        # Cat takes priority over person
        if found_cat:    return "cat",    frame
        if found_person: return "person", frame
        return None, frame

    except Exception as e:
        log.error(f"YOLO error: {e}")
        return None, frame

def detect_hog(frame):
    """
    HOG fallback — only detects persons (no cat support).
    Used when YOLOv5 fails to load.
    """
    if not HOG_AVAILABLE:
        return None, frame
    try:
        # Shrink frame for faster processing
        small = cv2.resize(frame, (320, 240))
        rects, weights = hog.detectMultiScale(
            small, winStride=(4, 4), padding=(8, 8), scale=1.05
        )
        if len(rects) == 0:
            return None, frame

        # Scale the best detection back to full frame size
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
    """Main detection — uses YOLO if available, HOG as fallback."""
    if YOLO_AVAILABLE:
        return detect_yolo(frame)
    return detect_hog(frame)

# === WEB PREVIEW ===
class FrameBuffer:
    """
    Thread-safe container for the latest JPEG frame.
    The main loop writes frames here.
    The web server reads from here and streams to the browser.
    Using a lock prevents both threads from accessing the frame at the same time.
    """
    def __init__(self):
        self.lock  = threading.Lock()
        self.frame = None

    def update(self, bgr_frame):
        # Encode frame as JPEG with quality 65 (lower = faster, smaller)
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
    """
    Handles browser connections.
    Streams MJPEG — a sequence of JPEG images sent continuously.
    The browser renders them as a video.
    """
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        min_interval = 1.0 / WEB_FPS_CAP  # minimum seconds between frames
        last_sent = 0
        try:
            while True:
                now = time.time()
                # Only send a new frame if enough time has passed
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
        pass  # silence request logs

def start_web_server(port=8080):
    """Start the MJPEG server in a background thread."""
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
    # Open camera
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
    log.info("🌐 Open http://10.0.0.173:8080/ in your browser to see the camera")

    # === STATE ===
    scanning       = True   # is the arm currently scanning?
    scan_index     = 0      # which angle in SCAN_ANGLES we're at
    scan_direction = 1      # 1 = moving right, -1 = moving left
    last_action    = 0      # timestamp of last wave/pinch
    consec         = 0      # consecutive frames with same detection
    last_label     = None   # what was detected last frame
    last_web_frame = 0      # timestamp of last frame sent to web buffer

    while True:
        ret, frame = cap.read()
        if not ret:
            log.warning("⚠️ Camera read failed")
            time.sleep(0.05)
            continue

        # --- run detection ---
        label, frame = detect(frame)

        # --- count consecutive detections ---
        # We require CONSEC_NEEDED frames in a row to avoid false triggers
        if label and label == last_label:
            consec += 1
        else:
            consec     = 0
            last_label = label

        # --- trigger action if confirmed detection and cooldown passed ---
        now = time.time()
        if consec >= CONSEC_NEEDED and now - last_action > COOLDOWN:
            last_action = now
            consec      = 0
            scanning    = False  # pause scanning while acting

            if label == "cat":
                # Run action in background thread so camera loop keeps running
                threading.Thread(target=extend_and_pinch, daemon=True).start()
            elif label == "person":
                threading.Thread(target=wave, daemon=True).start()

            # Resume scanning after 10 seconds
            def resume():
                global scanning
                time.sleep(10.0)
                scanning = True
            threading.Thread(target=resume, daemon=True).start()

        # --- advance scan position ---
        if scanning:
            move_servo(1, SCAN_ANGLES[scan_index])
            scan_index += scan_direction
            if scan_index >= len(SCAN_ANGLES):
                scan_index     = len(SCAN_ANGLES) - 2
                scan_direction = -1
            elif scan_index < 0:
                scan_index     = 1
                scan_direction = 1

        # --- draw HUD on frame ---
        ts = datetime.now().strftime("%H:%M:%S")
        cv2.putText(frame, f"DOFBOT 2026 | {ts}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame,
                    f"Detected: {label or 'nothing'} ({consec}/{CONSEC_NEEDED})",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame,
                    f"Scanning: {'YES' if scanning else 'NO (acting)'}",
                    (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # --- push frame to web buffer ---
        frame_buffer.update(frame)
