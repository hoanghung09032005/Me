"""
config.py
----------
Nơi chứa TẤT CẢ các thông số/cấu hình chung của project.
Muốn đổi IP xe, kích thước cửa sổ, tên trạm đo... thì sửa ở ĐÂY,
không cần lục trong code giao diện.
"""

WINDOW_TITLE = "Xe Tự Lái - Điều Khiển & Giám Sát"
WINDOW_SIZE = "1080x640"

# Kích thước khung hiển thị video - NÊN đặt TRÙNG với độ phân giải
# camera đang cấu hình trên ESP32-S3 (vd FRAMESIZE_QVGA = 320x240).
# Nếu 2 số này khác độ phân giải thật của camera, Python sẽ phải
# resize từng frame -> tốn CPU -> tăng độ trễ hiển thị.
VIDEO_W, VIDEO_H = 320, 240

# Số khung hình/giây tối đa mà GIAO DIỆN sẽ cập nhật lên màn hình.
UI_FPS = 15

# =====================================================================
# KẾT NỐI TỚI XE
# ---------------------------------------------------------------------
# Từ nay chỉ còn MỘT board ESP32-S3 CAM lo cả 2 việc (bỏ ESP8266):
#   - Port 81 : luồng video MJPEG
#   - Port 8080 : cầu nối TCP <-> UART xuống STM32 (xem car_bridge.cpp)
# Nên CAR_IP và IP trong DEFAULT_STREAM_URL là CÙNG một địa chỉ.
# IP này in ra ở Serial Monitor lúc ESP32-S3 khởi động.
# =====================================================================
ESP32_IP = "192.168.1.121"

DEFAULT_STREAM_URL = f"http://{ESP32_IP}:81/stream"
CAR_IP = ESP32_IP
CAR_PORT = 8080

# Thời gian chờ tối đa khi bấm nút "Kết nối xe" (giây)
CONNECT_TIMEOUT = 4.0

# Giữ nút lái thì cứ mỗi ngần này (ms) lại gửi lại lệnh xuống xe.
# PHẢI nhỏ hơn hẳn MANUAL_TIMEOUT bên STM32 (đang là 400ms), nếu không
# xe sẽ tự phanh giữa chừng dù bạn vẫn đang giữ nút.
MANUAL_REPEAT_MS = 120

# Bàn phím khi giữ phím sẽ tự động lặp lại KeyPress/KeyRelease liên tục.
# Nhận được KeyRelease thì đợi ngần này (ms) rồi mới thực sự phanh -
# nếu trong lúc đợi lại có KeyPress của cùng phím thì biết là do tự lặp,
# không phải người dùng nhả tay ra.
KEY_RELEASE_GRACE_MS = 60

# Nhịp giao diện đọc dữ liệu xe gửi về (ms)
TELEMETRY_POLL_MS = 50

# Danh sách các vị trí đo cố định dùng cho Mode 2 (tự lái)
CHECKPOINTS = [
    {"id": 1, "name": "Trạm 1"},
    {"id": 2, "name": "Trạm 2"},
    {"id": 3, "name": "Trạm 3"},
]

# Bảng màu dùng chung cho giao diện (đổi ở đây sẽ đổi màu toàn bộ app)
COLORS = {
    "bg": "#1e1e1e",
    "panel_bg": "#252526",
    "btn_bg": "#333336",
    "btn_active": "#4fc3f7",
    "text": "#e0e0e0",
    "accent": "#4fc3f7",
    "ok": "#66bb6a",
    "warn": "#ffb300",
    "error": "#ef5350",
}
