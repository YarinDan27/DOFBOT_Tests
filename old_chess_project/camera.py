#!/usr/bin/env python3
"""
Camera + MJPEG streaming.

How the stream works: the camera gives raw frames; each one is compressed to a
JPEG; the JPEGs are pushed down a single never-ending HTTP response, separated
by a marker line. The browser treats that as a slideshow that never finishes.
Simple and universal, but every frame is a whole image, so it costs far more
than real video, which only sends what changed.

Three things decide your frame rate:
  capture size   -- a 1280x720 frame is 4x the pixels of 640x360
  JPEG quality   -- compression runs on the CPU, and the Jetson's CPU is slow
  the network    -- whole images, every frame
"""
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
    def __init__(self, index=0):
        self.index = index
        self.cap = None
        self.server = None
        self.running = False
        self.grab_fps = 0.0

    def start(self):
        if self.running:
            return
        self.cap = cv2.VideoCapture(self.index)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"camera {self.index} won't open -- another program may be holding it "
                "(check with: sudo fuser /dev/video0)")
        # smaller frames = less to compress = higher frame rate
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAM_HEIGHT)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # always give us the newest frame
        self.running = True
        threading.Thread(target=self._grab_loop, daemon=True).start()

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

    def url(self):
        return f"http://{local_ip()}:{config.WEB_PORT}"

    def stats(self):
        frame = self.latest_frame()
        size = "no frames yet" if frame is None else f"{frame.shape[1]}x{frame.shape[0]}"
        return f"{size} at {self.grab_fps:.1f} fps from the camera"

    def stop(self):
        self.running = False
        if self.server:
            threading.Thread(target=self.server.shutdown, daemon=True).start()
            self.server = None
