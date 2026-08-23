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
_READ_CHUNK_BYTES = 65_536  # 64KB - đọc lớn hơn để giảm số lần syscall, mượt hơn


class MJPEGReader:
    def __init__(self, url, on_error=None):
        self.url = url
        self.on_error = on_error
        self._running = False
        self._thread = None
        self._lock = threading.Lock()
        self._latest_frame = None
        # Đếm số frame đã GIẢI MÃ THÀNH CÔNG (không liên quan gì tới việc
        # GUI có vẽ kịp hay không) - dùng để đo khách quan tốc độ "nguồn"
        # (mạng + decode) khi so sánh Tkinter với PySide6, tách bạch với
        # tốc độ hiển thị của từng framework.
        self.frames_received = 0

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
                # QUAN TRỌNG: dùng read1() chứ KHÔNG dùng read().
                # read(n) của Python sẽ CHỜ GOM ĐỦ n byte rồi mới trả về
                # (hoặc EOF) - với luồng MJPEG gửi từng khung nhỏ (~3-8KB)
                # mỗi ~30ms, gọi read(65536) có thể phải CHỜ 300-500ms mới
                # đủ 64KB, dù dữ liệu mới đã sẵn sàng từ lâu! Đây là
                # nguyên nhân trễ/giật hình THẬT SỰ - hoàn toàn nằm ở tầng
                # đọc mạng, KHÔNG liên quan gì tới Tkinter hay framework
                # GUI nào cả. read1(n) trả về NGAY khi có dữ liệu (tối đa
                # n byte, có thể ít hơn) sau đúng 1 lần đọc hệ thống - đo
                # thực tế cho thấy tốc độ nhận khớp chính xác tốc độ
                # camera gửi, thay vì bị dồn cục theo lô như read().
                chunk = stream.read1(_READ_CHUNK_BYTES)
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

                # CHỈ GIỮ FRAME JPEG HOÀN CHỈNH MỚI NHẤT TRONG BUF:
                # Nếu trong 1 lần đọc mạng nhận về nhiều frame liền nhau
                # (do WiFi/GUI xử lý chậm hơn tốc độ camera gửi), ta chỉ
                # decode frame CUỐI CÙNG và bỏ qua các frame cũ hơn - tránh
                # GUI phải "đuổi kịp" hàng loạt frame trễ, giúp video mượt
                # hơn thay vì bị giật/khựng khi dồn frame.
                last_start = None
                search_from = 0
                while True:
                    s = buf.find(b"\xff\xd8", search_from)
                    if s == -1:
                        break
                    e = buf.find(b"\xff\xd9", s)
                    if e == -1:
                        break
                    last_start = (s, e)
                    search_from = e + 2

                if last_start is not None:
                    s, e = last_start
                    jpg_bytes = buf[s : e + 2]
                    buf = buf[e + 2 :]

                    arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
                    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

                    if frame is not None:
                        with self._lock:
                            # KHUNG HÌNH RAW NÀY (frame) LÀ ĐỂ DÀNH CHO AI SAU NÀY
                            self._latest_frame = frame
                            self.frames_received += 1

        except Exception as e:
            if self._running and self.on_error:
                self.on_error(str(e))