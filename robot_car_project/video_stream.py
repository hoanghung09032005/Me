"""
video_stream.py
---------------
Doc luong MJPEG tu ESP32-S3/OV5640 va luon giu frame moi nhat.
Thiet ke uu tien latency: frame cu bi bo, GUI/AI chi lay frame gan nhat.
"""

import threading
import time
import urllib.request

import cv2
import numpy as np


_MAX_BUFFER_BYTES = 220_000
_READ_CHUNK_BYTES = 16_384


class MJPEGReader:
    def __init__(self, url, on_error=None):
        self.url = url
        self.on_error = on_error
        self._running = False
        self._thread = None
        self._lock = threading.Lock()
        self._latest_frame = None
        self._latest_seq = 0
        self._last_read_seq = 0
        self.frames_received = 0
        self.frames_dropped = 0
        self.last_frame_time = 0.0

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def get_latest_frame(self):
        with self._lock:
            if self._latest_frame is None or self._latest_seq == self._last_read_seq:
                return None
            self._last_read_seq = self._latest_seq
            return self._latest_frame

    def _set_latest_frame(self, frame):
        with self._lock:
            if self._latest_seq != self._last_read_seq:
                self.frames_dropped += 1
            self._latest_frame = frame
            self._latest_seq += 1
            self.frames_received += 1
            self.last_frame_time = time.monotonic()

    def _run(self):
        try:
            request = urllib.request.Request(
                self.url,
                headers={
                    "User-Agent": "Robot-MJPEG-LowLatency/1.0",
                    "Accept": "multipart/x-mixed-replace,image/jpeg,*/*",
                    "Cache-Control": "no-cache",
                    "Pragma": "no-cache",
                    "Connection": "close",
                },
            )
            stream = urllib.request.urlopen(request, timeout=5)
            buf = b""

            while self._running:
                chunk = stream.read1(_READ_CHUNK_BYTES)
                if not chunk:
                    if self._running and self.on_error:
                        self.on_error("Mat ket noi toi stream.")
                    break

                buf += chunk
                if len(buf) > _MAX_BUFFER_BYTES:
                    start = buf.rfind(b"\xff\xd8")
                    buf = buf[start:] if start != -1 else b""

                latest = None
                search_from = 0
                while True:
                    start = buf.find(b"\xff\xd8", search_from)
                    if start == -1:
                        break
                    end = buf.find(b"\xff\xd9", start + 2)
                    if end == -1:
                        break
                    latest = (start, end + 2)
                    search_from = end + 2

                if latest is None:
                    continue

                start, end = latest
                jpg_bytes = buf[start:end]
                buf = buf[end:]

                arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    self._set_latest_frame(frame)

        except Exception as e:
            if self._running and self.on_error:
                self.on_error(str(e))
        finally:
            self._running = False
