#!/usr/bin/env python3
"""
Camera: grabs frames, streams them to your browser, saves photos.

Carried over from old_chess_project, plus save_photo() for the teach and
self-test footage.

How the stream works: each frame is compressed to a JPEG and pushed down one
never-ending HTTP response. The browser shows it as a slideshow that never
finishes (MJPEG). Simple and universal, but every frame is a whole image.
"""
import glob
import os
import re
import signal
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

import cv2

import config

_latest = {"frame": None, "count": 0}
_lock = threading.Lock()


# ---------------- finding and freeing the camera ----------------
def video_devices():
    """['/dev/video0', '/dev/video1', ...], in number order."""
    return sorted(glob.glob("/dev/video*"), key=lambda p: (len(p), p))


def device_numbers(devices):
    out = []
    for path in devices:
        m = re.match(r"/dev/video(\d+)$", path)
        if m:
            out.append(int(m.group(1)))
    return out


def _holders(device):
    """PIDs of other programs with `device` open (via fuser), or None if fuser is missing."""
    try:
        result = subprocess.run(["fuser", device], stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, universal_newlines=True, timeout=5)
    except (OSError, subprocess.TimeoutExpired):
        return None
    mine = (os.getpid(), os.getppid())
    return [int(p) for p in re.findall(r"\d+", result.stdout) if int(p) not in mine]


def _process_name(pid):
    try:
        with open("/proc/%d/comm" % pid) as fh:
            return fh.read().strip()
    except OSError:
        return "?"


def _alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def free_cameras(devices):
    """
    Stops every other program holding one of these video devices -- the
    "device busy" problem, done automatically (what `sudo fuser -k` would do,
    but only for your own programs, and saying what it stopped).
    """
    for device in devices:
        pids = _holders(device)
        if pids is None:
            print("  (can't check who holds the camera: 'fuser' isn't installed)")
            return
        for pid in pids:
            name = _process_name(pid)
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                continue
            except PermissionError:
                print("  %s is held by '%s' (pid %d), which belongs to another user. "
                      "Run: sudo fuser -k %s" % (device, name, pid, device))
                continue
            deadline = time.time() + 1.5
            while _alive(pid) and time.time() < deadline:
                time.sleep(0.1)
            if _alive(pid):
                try:
                    os.kill(pid, signal.SIGKILL)
                except OSError:
                    pass
            print("  Freed the camera: stopped '%s' (pid %d), which was holding %s"
                  % (name, pid, device))


def _backends():
    """(backend, name) pairs to try, best first."""
    out = []
    if hasattr(cv2, "CAP_V4L2"):
        out.append((cv2.CAP_V4L2, "V4L2"))
    out.append((None, "OpenCV default"))
    return out


def _try_open(index, backend, wait_s=3.0):
    """(capture, first frame) if this device gives a real colour picture, else None."""
    try:
        cap = cv2.VideoCapture(index) if backend is None else cv2.VideoCapture(index, backend)
    except Exception:
        return None
    if not cap.isOpened():
        cap.release()
        return None
    if backend is not None and hasattr(cv2, "VideoWriter_fourcc"):
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # always give us the newest frame
    deadline = time.time() + wait_s
    while time.time() < deadline:
        ok, frame = cap.read()
        if ok and frame is not None and frame.ndim == 3 and frame.shape[2] == 3:
            return cap, frame
        time.sleep(0.05)
    cap.release()
    return None


def local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path not in ("/", "/stream"):
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        last_sent = -1
        try:
            while True:
                with _lock:
                    frame = _latest["frame"]
                    count = _latest["count"]
                if frame is None or count == last_sent:
                    time.sleep(0.01)
                    continue
                last_sent = count
                ok, jpeg = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, config.JPEG_QUALITY])
                if not ok:
                    continue
                data = jpeg.tobytes()
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(("Content-Length: %d\r\n\r\n" % len(data)).encode())
                self.wfile.write(data)
                self.wfile.write(b"\r\n")
        except Exception:
            pass


class Camera:
    def __init__(self, index=config.CAMERA_INDEX):
        self.index = index
        self.cap = None
        self.server = None
        self.running = False
        self.grab_fps = 0.0
        self.description = "not open"

    def start(self, stream=True):
        """
        1. Free the camera: stop any other program holding a /dev/video*
           device (the "device busy" error, errno 16).
        2. Open it: the preferred device first, then every other video device,
           each with V4L2 + MJPG first, then OpenCV's default backend.
        3. Only call it working once a real colour frame has arrived.
        """
        if self.running:
            return
        devices = video_devices()
        if config.CAMERA_FREE_DEVICE:
            free_cameras(devices)

        order = [self.index] + [n for n in device_numbers(devices) if n != self.index]
        tried = []
        for index in order:
            for backend, name in _backends():
                found = _try_open(index, backend)
                tried.append("/dev/video%d via %s" % (index, name))
                if found is not None:
                    self.cap, first = found
                    self.index = index
                    self.description = "/dev/video%d via %s, %dx%d" % (
                        index, name, first.shape[1], first.shape[0])
                    with _lock:
                        _latest["frame"] = first
                        _latest["count"] += 1
                    break
            if self.cap is not None:
                break
        if self.cap is None:
            raise RuntimeError(
                "no camera gave a picture (tried %s). If another program still has it, "
                "run: sudo fuser -k /dev/video%d" % (", ".join(tried) or "nothing -- "
                                                     "no /dev/video* devices", self.index))

        self.running = True
        threading.Thread(target=self._grab_loop, daemon=True).start()
        print("  Camera on: " + self.description)

        if stream:
            try:
                self.server = ThreadedHTTPServer(("0.0.0.0", config.WEB_PORT), Handler)
                threading.Thread(target=self.server.serve_forever, daemon=True).start()
                print("  Watch live: " + self.url())
            except OSError as e:
                self.server = None
                print("  (live stream unavailable: port %d is busy -- %s)" % (config.WEB_PORT, e))

    def _grab_loop(self):
        frames, t0 = 0, time.time()
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.02)
                continue
            with _lock:
                _latest["frame"] = frame
                _latest["count"] += 1
            frames += 1
            if frames >= 30:
                now = time.time()
                self.grab_fps = frames / (now - t0)
                frames, t0 = 0, now
        if self.cap:
            self.cap.release()

    def latest_frame(self):
        with _lock:
            return None if _latest["frame"] is None else _latest["frame"].copy()

    def fresh_frame(self, timeout=2.0):
        """
        A frame captured AFTER this call. Use it right after the arm stops:
        latest_frame() could still be from while it was moving.
        """
        with _lock:
            start_count = _latest["count"]
        deadline = time.time() + timeout
        while time.time() < deadline:
            with _lock:
                if _latest["count"] >= start_count + 2 and _latest["frame"] is not None:
                    return _latest["frame"].copy()
            time.sleep(0.02)
        return None

    def save_photo(self, path):
        """Saves a fresh frame as a JPEG. Returns the path, or None if no frame came."""
        frame = self.fresh_frame()
        if frame is None:
            return None
        folder = os.path.dirname(path)
        if folder and not os.path.isdir(folder):
            os.makedirs(folder)
        cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, config.PHOTO_QUALITY])
        return path

    def url(self):
        return "http://%s:%d" % (local_ip(), config.WEB_PORT)

    def stats(self):
        """Waits ~2 s for a frame-rate measurement, then describes the camera."""
        deadline = time.time() + 2.5
        while self.grab_fps == 0.0 and time.time() < deadline:
            time.sleep(0.1)
        frame = self.latest_frame()
        size = "no frames yet" if frame is None else "%dx%d" % (frame.shape[1], frame.shape[0])
        return "%s: %s at %.1f fps" % (self.description, size, self.grab_fps)

    def stop(self):
        self.running = False
        if self.server:
            threading.Thread(target=self.server.shutdown, daemon=True).start()
            self.server = None
