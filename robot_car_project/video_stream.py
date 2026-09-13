"""
video_stream.py
-----------------
Đọc luồng video MJPEG từ camera (ESP32-S3) và luôn giữ FRAME MỚI NHẤT.

CẬP NHẬT (xoay ảnh): thêm tuỳ chọn xoay ảnh (rotate) ở phía PC. Lý do: thư
viện esp32-camera trên ESP32 CHỈ hỗ trợ set_vflip()/set_hmirror() (lật, tối
đa 180°), KHÔNG hỗ trợ xoay 90°/270° cho ảnh JPEG. Nếu module camera mới bị
lắp lệch hướng 90° so với module cũ (vấn đề cơ khí, không phải phần mềm),
cách nhanh nhất không đụng firmware là xoay ở đây bằng cv2.rotate().

Cách dùng: đặt rotate khi tạo MJPEGReader, ví dụ:
    MJPEGReader(url, rotate=cv2.ROTATE_90_CLOCKWISE)
Giá trị hợp lệ: None (không xoay), cv2.ROTATE_90_CLOCKWISE,
cv2.ROTATE_90_COUNTERCLOCKWISE, cv2.ROTATE_180.

QUAN TRỌNG - KHÔNG đổi lại thành stream.read(n) (blocking):
read(n) của Python CHỜ GOM ĐỦ n byte rồi mới trả về (hoặc EOF) - với luồng
MJPEG gửi từng khung nhỏ (~3-8KB) mỗi ~30ms, gọi read(32768) có thể phải
CHỜ hàng trăm ms mới đủ byte dù dữ liệu mới đã sẵn sàng từ lâu - đây là
nguyên nhân trễ/giật hình THẬT SỰ, đã từng đo và xác nhận trên phần cứng
thật. read1(n) trả về NGAY khi có dữ liệu (tối đa n byte, có thể ít hơn)
sau đúng 1 lần đọc hệ thống - đúng nguyên tắc ưu tiên latency của module này.
"""

import threading
import urllib.request
import cv2
import numpy as np

_MAX_BUFFER_BYTES = 300_000
_READ_CHUNK_BYTES = 65_536  # kích thước ĐỌC TỐI ĐA mỗi lần - không phải kích
                             # thước CHỜ ĐỦ, vì dùng read1() chứ không phải read()


class MJPEGReader:
    def __init__(self, url, on_error=None, rotate=None):
        self.url = url
        self.on_error = on_error
        self.rotate = rotate  # None hoặc cv2.ROTATE_90_CLOCKWISE / _COUNTERCLOCKWISE / 180
        self._running = False
        self._thread = None
        self._lock = threading.Lock()
        self._latest_frame = None
        # Đếm số frame đã GIẢI MÃ THÀNH CÔNG - dùng để đo khách quan tốc độ
        # nguồn (mạng + decode), tách bạch với tốc độ hiển thị của GUI.
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
                # BẮT BUỘC dùng read1() - xem ghi chú ở đầu file. Đừng đổi
                # lại thành read(), kể cả khi "tăng kích thước đọc" nghe có
                # vẻ giúp mượt hơn - thực tế làm NGƯỢC LẠI với read() thường.
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

                # CHỈ GIẢI MÃ FRAME JPEG HOÀN CHỈNH MỚI NHẤT TRONG BUF: nếu 1
                # lần đọc mạng nhận về nhiều frame liền nhau (WiFi/GUI xử lý
                # chậm hơn tốc độ camera gửi), chỉ decode frame CUỐI CÙNG,
                # bỏ qua các frame cũ hơn thay vì decode hết rồi vứt - tránh
                # tốn CPU vô ích cho những frame sẽ bị ghi đè ngay lập tức.
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

                if last_start is None:
                    continue

                s, e = last_start
                jpg_bytes = buf[s : e + 2]
                buf = buf[e + 2 :]

                arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

                if frame is not None:
                    if self.rotate is not None:
                        frame = cv2.rotate(frame, self.rotate)

                    with self._lock:
                        # KHUNG HÌNH RAW NÀY (frame) LÀ ĐỂ DÀNH CHO AI SAU NÀY
                        self._latest_frame = frame
                        self.frames_received += 1

        except Exception as e:
            if self._running and self.on_error:
                self.on_error(str(e))
