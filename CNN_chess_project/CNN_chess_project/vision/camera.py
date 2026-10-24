#!/usr/bin/env python3
"""
Camera: grabs frames, streams them to your browser, saves photos.

Carried over from old_chess_project, plus save_photo() for the teach and
self-test footage.

How the stream works: each frame is compressed to a JPEG and pushed down one
never-ending HTTP response. The browser shows it as a slideshow that never
finishes (MJPEG). Simple and universal, but every frame is a whole image.
"""
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

import cv2

import config

_latest = {"frame": None, "count": 0}
_lock = threading.Lock()


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

    def start(self, stream=True):
        if self.running:
            return
        self.cap = cv2.VideoCapture(self.index)
        if not self.cap.isOpened():
            raise RuntimeError(
                "camera %s won't open -- another program may be holding it "
                "(check with: sudo fuser /dev/video0)" % self.index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAM_HEIGHT)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # always give us the newest frame
        self.running = True
        threading.Thread(target=self._grab_loop, daemon=True).start()

        if stream:
            self.server = ThreadedHTTPServer(("0.0.0.0", config.WEB_PORT), Handler)
            threading.Thread(target=self.server.serve_forever, daemon=True).start()

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
        frame = self.latest_frame()
        size = "no frames yet" if frame is None else "%dx%d" % (frame.shape[1], frame.shape[0])
        return "%s at %.1f fps from the camera" % (size, self.grab_fps)

    def stop(self):
        self.running = False
        if self.server:
            threading.Thread(target=self.server.shutdown, daemon=True).start()
            self.server = None
