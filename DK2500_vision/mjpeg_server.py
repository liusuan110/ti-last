import argparse
import threading
import time
import socketserver
from http.server import BaseHTTPRequestHandler, HTTPServer


def _require(mod_name: str, pip_name: str):
    try:
        return __import__(mod_name)
    except Exception:
        raise RuntimeError(f"缺少依赖: {mod_name}. 请执行: python3 -m pip install {pip_name}")


cv2 = _require("cv2", "opencv-python")


class FrameBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self._jpeg = None
        self._ts = 0.0

    def set(self, jpeg_bytes: bytes):
        with self._lock:
            self._jpeg = jpeg_bytes
            self._ts = time.time()

    def get(self):
        with self._lock:
            return self._jpeg, self._ts


class CameraThread(threading.Thread):
    def __init__(
        self,
        fb: FrameBuffer,
        device: str,
        width: int,
        height: int,
        fps: int,
        fourcc: str,
        quality: int,
        rotate: int,
    ):
        super().__init__(daemon=True)
        self.fb = fb
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.fourcc = fourcc
        self.quality = quality
        self.rotate = rotate
        self._stop = threading.Event()
        self._cap = None

    def stop(self):
        self._stop.set()

    def run(self):
        dev = self.device
        if isinstance(dev, str) and dev.isdigit():
            dev = int(dev)
        backend = cv2.CAP_V4L2 if hasattr(cv2, "CAP_V4L2") else 0
        cap = cv2.VideoCapture(dev, backend)
        self._cap = cap
        # Select the compressed transport first.  Several UVC cameras reset
        # width/height/FPS when FOURCC changes, silently falling back to
        # 25 fps if MJPG is selected last.
        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        cap.set(cv2.CAP_PROP_FPS, float(self.fps))
        actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        actual_fourcc_text = "".join(
            chr((actual_fourcc >> (8 * index)) & 0xFF) for index in range(4)
        )
        print(
            "camera_active "
            f"fourcc={actual_fourcc_text} "
            f"size={int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
            f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} "
            f"fps={cap.get(cv2.CAP_PROP_FPS):.3f}",
            flush=True,
        )

        params = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.quality)]

        while not self._stop.is_set():
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.02)
                continue
            if self.rotate == 90:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            elif self.rotate == 180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            elif self.rotate == 270:
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            ok2, enc = cv2.imencode(".jpg", frame, params)
            if not ok2:
                continue
            self.fb.set(enc.tobytes())

        try:
            cap.release()
        except Exception:
            pass


class Handler(BaseHTTPRequestHandler):
    server_version = "mjpeg/1.0"

    def do_HEAD(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            return
        if self.path.startswith("/stream.mjpg"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            return
        if self.path.startswith("/frame.jpg"):
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.end_headers()
            return
        self.send_response(404)
        self.end_headers()

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(
                (
                    "<!doctype html><html><head><meta charset='utf-8'>"
                    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                    "<title>MJPEG</title></head><body style='margin:0;background:#111;'>"
                    "<img src='/stream.mjpg' style='width:100%;height:auto;display:block;'/>"
                    "</body></html>"
                ).encode("utf-8")
            )
            return

        if self.path.startswith("/stream.mjpg"):
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Connection", "close")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            fb: FrameBuffer = self.server.fb  # type: ignore[attr-defined]
            last_ts = 0.0
            try:
                while True:
                    jpeg, ts = fb.get()
                    if jpeg is None or ts == last_ts:
                        time.sleep(0.01)
                        continue
                    last_ts = ts
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except Exception:
                return

        if self.path.startswith("/frame.jpg"):
            fb: FrameBuffer = self.server.fb  # type: ignore[attr-defined]
            jpeg, _ = fb.get()
            if jpeg is None:
                self.send_response(503)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                self.wfile.write("no frame".encode("utf-8"))
                return
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)
            return

        self.send_response(404)
        self.end_headers()

    def log_message(self, fmt, *args):
        return


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--fourcc", default="MJPG")
    ap.add_argument("--quality", type=int, default=70)
    ap.add_argument("--rotate", type=int, choices=(0, 90, 180, 270), default=0)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    fb = FrameBuffer()
    cam = CameraThread(
        fb=fb,
        device=args.device,
        width=args.width,
        height=args.height,
        fps=args.fps,
        fourcc=args.fourcc,
        quality=args.quality,
        rotate=args.rotate,
    )
    cam.start()

    class _ThreadingHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
        daemon_threads = True

    httpd = _ThreadingHTTPServer((args.host, args.port), Handler)
    httpd.fb = fb  # type: ignore[attr-defined]

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            httpd.server_close()
        except Exception:
            pass
        cam.stop()
        cam.join(timeout=1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
