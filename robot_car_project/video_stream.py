"""
video_stream.py
-----------------
Đọc luồng video MJPEG từ camera (ESP32-S3) và luôn giữ FRAME MỚI NHẤT.
"""

import threading
import urllib.request
import cv2
import numpy as np

_MAX_BUFFER_BYTES = 300_000


class MJPEGReader:
    def __init__(self, url, on_error=None):
        self.url = url
        self.on_error = on_error
        self._running = False
        self._thread = None
        self._lock = threading.Lock()
        self._latest_frame = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def get_latest_frame(self):
        with self._lock:
            return self._latest_frame

    def _run(self):
        try:
            stream = urllib.request.urlopen(self.url, timeout=5)
            buf = b""
            while self._running:
                chunk = stream.read(32768)  # Tăng kích thước đọc lên 32KB để mượt hơn
                if not chunk:
                    if self._running and self.on_error:
                        self.on_error("Mất kết nối tới stream.")
                    break
                buf += chunk

                # Cải thiện logic chống tràn bộ đệm: Tìm header JPEG cuối cùng
                if len(buf) > _MAX_BUFFER_BYTES:
                    idx = buf.rfind(b"\xff\xd8")
                    if idx != -1:
                        buf = buf[idx:]
                    else:
                        buf = b""

                while True:
                    start = buf.find(b"\xff\xd8")
                    end = buf.find(b"\xff\xd9")

                    if start == -1 or end == -1 or end <= start:
                        break

                    jpg_bytes = buf[start : end + 2]
                    buf = buf[end + 2 :]

                    arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
                    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

                    if frame is not None:
                        with self._lock:
                            # KHUNG HÌNH RAW NÀY (frame) LÀ ĐỂ DÀNH CHO AI SAU NÀY
                            self._latest_frame = frame

        except Exception as e:
            if self._running and self.on_error:
                self.on_error(str(e))
