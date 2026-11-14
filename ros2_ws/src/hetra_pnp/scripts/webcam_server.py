#!/usr/bin/env python3
"""Run this on the WINDOWS HOST (not in Docker): serves the webcam as an MJPEG stream.

Docker Desktop can't pass /dev/video0 into the container, so the container reads
    http://host.docker.internal:8081/video
instead. Port 8080 is already used by the noVNC container, hence 8081.

    pip install opencv-python
    python webcam_server.py                      # camera 0, 640x480 @ 30
    python webcam_server.py --device 1 --width 1280 --height 800 --fps 60

Open http://localhost:8081/ in a browser to check it works. Ctrl+C to stop.

Binds to 127.0.0.1 by default: Docker Desktop forwards host.docker.internal to the
host's localhost, so the stream is reachable from the container but not from your
network. Only use --host 0.0.0.0 if a container/VM really can't reach it otherwise.
"""
import argparse
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

BOUNDARY = b"frame"


class Camera:
    def __init__(self, a):
        backend = {"dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF, "any": cv2.CAP_ANY}[a.backend]
        self.cap = cv2.VideoCapture(a.device, backend)
        if not self.cap.isOpened():
            sys.exit(f"could not open camera {a.device} with backend {a.backend}")
        if a.mjpg:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
        self.cap.set(cv2.CAP_PROP_FPS, a.fps)
        if a.settings:
            self.cap.set(cv2.CAP_PROP_SETTINGS, 1)  # opens the driver dialog (exposure etc.)
        self.quality = a.quality
        self.jpeg = None
        self.seq = 0
        self.cond = threading.Condition()
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        n, t0 = 0, time.monotonic()
        while True:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if not ok:
                continue
            with self.cond:
                self.jpeg, self.seq = buf.tobytes(), self.seq + 1
                self.cond.notify_all()
            n += 1
            if time.monotonic() - t0 > 5:
                print(f"capture {n / (time.monotonic() - t0):.1f} fps "
                      f"{frame.shape[1]}x{frame.shape[0]}", flush=True)
                n, t0 = 0, time.monotonic()

    def wait_next(self, last_seq, timeout=2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq, timeout)
            return self.jpeg, self.seq


def make_handler(cam: Camera):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path == "/":
                body = b'<html><body style="margin:0;background:#111">' \
                       b'<img src="/video" style="max-width:100%"></body></html>'
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path != "/video":
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=" + BOUNDARY.decode())
            self.end_headers()
            print(f"client connected: {self.client_address[0]}", flush=True)
            seq = -1
            try:
                while True:
                    jpg, seq = cam.wait_next(seq)
                    if jpg is None:
                        continue
                    self.wfile.write(b"--" + BOUNDARY + b"\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n")
                    self.wfile.write(jpg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                print(f"client disconnected: {self.client_address[0]}", flush=True)

    return Handler


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--device", type=int, default=0)
    p.add_argument("--backend", choices=["dshow", "msmf", "any"], default="dshow")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=float, default=30)
    p.add_argument("--mjpg", action="store_true", help="ask the camera for MJPG (needed for high fps on USB2)")
    p.add_argument("--quality", type=int, default=90, help="JPEG quality (higher = sharper corners)")
    p.add_argument("--settings", action="store_true", help="open the camera driver settings dialog")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8081)
    a = p.parse_args()

    cam = Camera(a)
    srv = ThreadingHTTPServer((a.host, a.port), make_handler(cam))
    srv.daemon_threads = True
    print(f"streaming camera {a.device} on http://localhost:{a.port}/  "
          f"(container: http://host.docker.internal:{a.port}/video)", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
