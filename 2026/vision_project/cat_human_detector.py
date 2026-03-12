#!/usr/bin/env python3
"""
DOFBOT Vision Patrol - 2026
Detects humans (waves) and cats (extends + pinches)
Web preview via browser at http://10.0.0.173:8080/
"""

import os, sys, time, pathlib, threading, signal, queue
import numpy as np, cv2
from datetime import datetime
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

# === ARM SETUP ===
try:
    from Arm_Lib import Arm_Device
    arm = Arm_Device()
    ARM_CONNECTED = True
    log.info("✅ Arm connected!")
except Exception:
    arm = None
    ARM_CONNECTED = False
    log.warning("⚠️ Arm not found. Movement skipped.")

# === YOLO SETUP ===
YOLO_AVAILABLE = False
try:
    import torch
    sys.path.insert(0, str(YOLO_DIR))
    from models.experimental import attempt_load
    from utils.general import non_max_suppression, scale_coords
    from utils.datasets import letterbox
    from utils.torch_utils import select_device

    device   = select_device("0" if torch.cuda.is_available() else "cpu")
    half     = device.type != "cpu"
    model    = attempt_load(str(WEIGHTS), map_location=device)
    if half: model.half()
    model.eval()
    YOLO_AVAILABLE = True
    log.info(f"✅ YOLOv5 loaded on {device} | half={half}")
except Exception as e:
    log.warning(f"⚠️ YOLOv5 failed: {e}")
    model = None

# === HOG FALLBACK ===
HOG_AVAILABLE = False
try:
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    HOG_AVAILABLE = True
    log.info("✅ HOG backup detector ready")
except Exception as e:
    log.warning(f"⚠️ HOG failed: {e}")

# === SETTINGS ===
SCAN_ANGLES   = list(range(20, 161, 10))   # 20° to 160° in steps of 10
SCAN_SPEED    = 400
ACTION_SPEED  = 500
CONF_THRESH   = 0.40
IOU_THRESH    = 0.45
COOLDOWN      = 5.0                         # seconds between actions
CONSEC_NEEDED = 3                           # consecutive detections to confirm

# === ARM POSITIONS ===
POS_HOME      = [90,  90,  90, 0, 90,  90]
POS_WAVE_UP   = [90,  45,  90, 45, 90,  90]
POS_WAVE_DOWN = [90,  70,  90, 70, 90,  90]
POS_CAT_REACH = [90,  30,  60, 30, 90,  90]
POS_CAT_PINCH = [90,  30,  60, 30, 90, 160]

# === ARM MOVEMENT ===
def move_all(position, speed=ACTION_SPEED):
    if not ARM_CONNECTED:
        log.debug(f"(no arm) → {position}")
        return
    arm.Arm_serial_servo_write6(*position, speed)
    time.sleep(speed / 1000 + 0.1)

def move_servo(servo_id, angle, speed=SCAN_SPEED):
    if not ARM_CONNECTED:
        return
    angle = max(0, min(180, int(angle)))
    arm.Arm_serial_servo_write(servo_id, angle, speed)
    time.sleep(speed / 1000 + 0.1)

def go_home():
    log.info("🏠 Going home")
    move_all(POS_HOME, 1000)

# === ACTIONS ===
def wave():
    log.info("👋 Waving at human!")
    for _ in range(3):
        move_all(POS_WAVE_UP)
        move_all(POS_WAVE_DOWN)
    go_home()

def extend_and_pinch():
    log.info("🐱 Cat detected — extending and pinching!")
    move_all(POS_CAT_REACH)
    time.sleep(0.3)
    move_all(POS_CAT_PINCH)
    time.sleep(1.0)
    go_home()

def dance():
    log.info("🎉 Dancing!")
    moves = [
        [90, 180, 0,  60,  45, 90],
        [90, 180, 0, 120, 135, 90],
        [90, 180, 0,  60, 135, 90],
        [90, 180, 0, 120,  45, 90],
        [90, 180, 0,  90,  90, 90],
    ]
    for _ in range(2):
        for move in moves:
            move_all(move, 400)
    go_home()

# === DETECTION ===
def detect_yolo(frame):
    """Returns: (label, annotated_frame) where label is 'person', 'cat', or None"""
    if not YOLO_AVAILABLE:
        return None, frame
    try:
        img = letterbox(frame, 640)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()
        img /= 255.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        with torch.no_grad():
            pred = model(img, augment=False)[0]

        # detect person (0) and cat (15)
        det = non_max_suppression(pred, CONF_THRESH, IOU_THRESH,
                                  classes=[0, 15])[0]

        if det is None or len(det) == 0:
            return None, frame

        det[:, :4] = scale_coords(img.shape[2:], det[:, :4], frame.shape).round()

        found_person = False
        found_cat    = False

        for *xyxy, conf, cls in det:
            x1, y1, x2, y2 = [int(v) for v in xyxy]
            label = model.names[int(cls)]
            color = (0, 255, 0) if label == "person" else (0, 165, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
            cv2.putText(frame, f"{label} {conf:.2f}",
                        (x1, max(0, y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            if label == "person": found_person = True
            if label == "cat":    found_cat    = True

        if found_person: return "person", frame
        if found_cat:    return "cat",    frame
        return None, frame

    except Exception as e:
        log.error(f"YOLO error: {e}")
        return None, frame

def detect_hog(frame):
    """HOG fallback — detects persons only"""
    if not HOG_AVAILABLE:
        return None, frame
    try:
        small = cv2.resize(frame, (320, 240))
        rects, weights = hog.detectMultiScale(small, winStride=(4,4),
                                               padding=(8,8), scale=1.05)
        if len(rects) == 0:
            return None, frame

        best = rects[np.argmax(weights)]
        x, y, w, h = [int(v * 2) for v in best]   # scale back to 640x480
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 3)
        cv2.putText(frame, "person (HOG)", (x, max(0, y-10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        return "person", frame
    except Exception as e:
        log.error(f"HOG error: {e}")
        return None, frame

def detect(frame):
    if YOLO_AVAILABLE:
        return detect_yolo(frame)
    return detect_hog(frame)

# === MJPEG WEB SERVER ===
from http.server import BaseHTTPRequestHandler, HTTPServer

class FrameBuffer:
    def __init__(self):
        self.lock  = threading.Lock()
        self.frame = None

    def update(self, bgr_frame):
        ok, buf = cv2.imencode(".jpg", bgr_frame,
                               [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self.lock:
                self.frame = buf.tobytes()

    def get(self):
        with self.lock:
            return self.frame

frame_buffer = FrameBuffer()

class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                jpg = frame_buffer.get()
                if jpg is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                self.wfile.write(jpg + b"\r\n")
        except Exception:
            pass

    def log_message(self, *args):
        pass   # silence request logs

def start_web_server(port=8080):
    server = HTTPServer(("0.0.0.0", port), StreamHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    log.info(f"🌐 Web preview → http://10.0.0.173:{port}/")

# === MAIN ===
def signal_handler(sig, frame_arg):
    log.info("👋 Exiting...")
    go_home()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":
    # Camera
    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        log.error("❌ Cannot open camera")
        sys.exit(1)

    start_web_server()
    go_home()

    log.info("🚀 Patrol started — open http://10.0.0.173:8080/ in your browser")

    scanning       = True
    scan_index     = 0
    scan_direction = 1
    last_action    = 0
    consec         = 0
    last_label     = None

    while True:
        ret, frame = cap.read()
        if not ret:
            log.warning("⚠️ Camera read failed")
            time.sleep(0.05)
            continue

        # --- detection ---
        label, frame = detect(frame)

        # --- consecutive confirmation ---
        if label and label == last_label:
            consec += 1
        else:
            consec = 0
            last_label = label

        # --- action trigger ---
        now = time.time()
        if consec >= CONSEC_NEEDED and now - last_action > COOLDOWN:
            last_action = now
            consec      = 0
            scanning    = False
            if label == "person":
                threading.Thread(target=wave,              daemon=True).start()
            elif label == "cat":
                threading.Thread(target=extend_and_pinch, daemon=True).start()
            threading.Timer(8.0, lambda: globals().update(scanning=True)).start()

        # --- scanning ---
        if scanning:
            move_servo(1, SCAN_ANGLES[scan_index])
            scan_index += scan_direction
            if scan_index >= len(SCAN_ANGLES):
                scan_index     = len(SCAN_ANGLES) - 2
                scan_direction = -1
            elif scan_index < 0:
                scan_index     = 1
                scan_direction = 1

        # --- HUD ---
        ts = datetime.now().strftime("%H:%M:%S")
        cv2.putText(frame, f"DOFBOT 2026 | {ts}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Detected: {label or 'nothing'} ({consec}/{CONSEC_NEEDED})",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        frame_buffer.update(frame)
