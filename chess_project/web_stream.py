#!/usr/bin/env python3
import time
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
import cv2
import config

global_frame = None
lock = threading.Lock()

class StreamingHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  

    def do_GET(self):
        global global_frame
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    with lock:
                        if global_frame is None:
                            time.sleep(0.05)
                            continue
                        _, jpeg = cv2.imencode('.jpg', global_frame, [cv2.IMWRITE_JPEG_QUALITY, config.JPEG_QUALITY])
                        frame_bytes = jpeg.tobytes()
                    
                    self.wfile.write(b'--frame\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(frame_bytes)))
                    self.end_headers()
                    self.wfile.write(frame_bytes)
                    self.wfile.write(b'\r\n')
                    time.sleep(1.0 / config.WEB_FPS_CAP)
            except Exception:
                pass

class WebStreamServer:
    def __init__(self):
        self.server = None
        self.thread = None
        self.cap = None
        self.is_running = False

    def _stream_loop(self):
        global global_frame
        self.cap = cv2.VideoCapture(0)
        while self.is_running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.05)
                continue
            with lock:
                global_frame = frame.copy()
        if self.cap:
            self.cap.release()

    def start_stream(self):
        if self.is_running:
            return
        self.is_running = True
        self.thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.thread.start()
        try:
            self.server = HTTPServer(('0.0.0.0', config.WEB_PORT), StreamingHandler)
            server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
            server_thread.start()
            print(f"[Network] Stream engine engaged dynamically.")
        except Exception as e:
            print(f"[Network Error] Bind failure: {e}")

    def stop_stream(self):
        self.is_running = False
        # Run shutdown in a background thread so it can NEVER deadlock main.py
        if self.server:
            t = threading.Thread(target=self.server.shutdown, daemon=True)
            t.start()
        print("[Network] Streaming infrastructure signaled shutdown.")
