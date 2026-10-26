#!/usr/bin/env python3
"""
The live 3D view, served by run.py while it runs:

    http://<jetson>:8081        (config.VIEW_PORT; run.py prints the address)

Open it in a browser on your laptop: it shows the arm as the program sees it
-- its pose (twice a second), the move it's making, the board with the
self-test colours, the electronics box -- using the same collision model the
program checks every move against. It only shows; nothing on the page moves
the arm. (Fonts and the 3D library come from the internet, so the laptop
needs a connection; the Jetson doesn't.)
"""
import json
import os
import socketserver
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import config

HERE = os.path.dirname(os.path.abspath(__file__))
SKELETON = ("<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">"
            "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1, "
            "viewport-fit=cover\"><style>:root{color-scheme:light dark}body{margin:0}"
            "[hidden]{display:none!important}</style></head><body>%s</body></html>")


def page(data=None):
    """The viewer as a complete HTML document; `data` embedded (None = live mode)."""
    with open(os.path.join(HERE, "viewer.html")) as fh:
        html = fh.read()
    if data is not None:
        blob = json.dumps(data, separators=(",", ":")).replace("</", "<\\/")
        html = html.replace("/*SCENE_DATA*/null", blob)
    return SKELETON % html


class _Server(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def start(get_scene, port=None):
    """Serves the view in a background thread. get_scene() -> dict. Returns the server."""
    port = port or config.VIEW_PORT
    html = page(None).encode("utf-8")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, body, kind):
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                self._send(html, "text/html; charset=utf-8")
            elif path == "/scene.json":
                try:
                    body = json.dumps(get_scene()).encode("utf-8")
                except Exception as e:                  # never take the program down
                    body = json.dumps({"error": str(e)}).encode("utf-8")
                self._send(body, "application/json")
            else:
                self.send_error(404)

    server = _Server(("0.0.0.0", port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server
