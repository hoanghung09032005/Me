"""
config.py
----------
Nơi chứa TẤT CẢ các thông số/cấu hình chung của project.
"""

WINDOW_TITLE = "Xe Tự Lái - Điều Khiển & Giám Sát"
WINDOW_SIZE = "1280x720"

VIDEO_W, VIDEO_H = 640, 480
VIDEO_DISPLAY_W, VIDEO_DISPLAY_H = 640, 480

# Tốc độ vẽ lại giao diện (fps). 30 -> ~33ms mỗi frame, mượt hơn 16fps cũ.
UI_FPS = 30

# =====================================================================
# HẬU KỲ ẢNH PHÍA PC
# =====================================================================
IMAGE_ENHANCE_ENABLED = False
IMAGE_DENOISE_ENABLED = True
IMAGE_DENOISE_D = 5
IMAGE_DENOISE_SIGMA_COLOR = 40
IMAGE_DENOISE_SIGMA_SPACE = 40
IMAGE_SHARPEN_AMOUNT = 0.8
IMAGE_SHARPEN_SIGMA = 1.5
IMAGE_CLAHE_CLIP_LIMIT = 2.0

# =====================================================================
# KẾT NỐI TỚI XE
# =====================================================================
ESP32_IP = "192.168.1.106"
DEFAULT_STREAM_URL = f"http://{ESP32_IP}:81/stream"
CAR_IP = ESP32_IP
CAR_PORT = 8080
CONNECT_TIMEOUT = 4.0
MANUAL_REPEAT_MS = 120
KEY_RELEASE_GRACE_MS = 60
TELEMETRY_POLL_MS = 50

# =====================================================================
# CẤU HÌNH MODE 4: AI NHẬN DIỆN LINE
# ---------------------------------------------------------------------
# Phía PC dùng OpenCV threshold để tìm line đen.
# =====================================================================
AI_LINE_THRESHOLD = 80  # Ngưỡng grayscale (0-255), dưới ngưỡng = đen
AI_LINE_CROP_TOP = 0.6  # Cắt bỏ % trên cùng của frame (chỉ xem nửa dưới)
AI_LINE_BLUR_KSIZE = 5  # Gaussian blur trước khi threshold

# Giá trị "sentinel" báo KHÔNG THẤY LINE, gửi qua lệnh "I <error_x100>".
# QUAN TRỌNG: KHÔNG dùng 0 cho việc này - error_x100 = 0 là giá trị HỢP LỆ
# (line nằm đúng giữa khung hình, chạy thẳng đúng tâm - trạng thái mong
# muốn nhất) - nếu lẫn 2 trường hợp này, STM32 sẽ hiểu nhầm "đang bám line
# hoàn hảo" thành "mất line" và bắt đầu pivot tìm kiếm giữa lúc đang chạy
# tốt nhất. Giá trị dưới đây nằm HẲN NGOÀI dải error hợp lệ (-400..400)
# nên không thể trùng với bất kỳ error thật nào. PHẢI khớp CHÍNH XÁC với
# AI_NO_LINE_SENTINEL trong Inc/mode4_ai.h - đổi 1 bên mà quên đổi bên kia
# sẽ làm mất hẳn tính năng phát hiện mất line.
AI_LINE_LOST_SENTINEL = 9999

# ---------------------------------------------------------------------
# MODE 4: AI MODEL THẬT (neural network, thay cho threshold cổ điển)
# ---------------------------------------------------------------------

AI_DATASET_DIR = "dataset_ai_line"  # nơi lưu ảnh + label khi ghi dữ liệu
AI_MODEL_PATH = "line_model.pth"  # model đã train xong, đặt cùng thư mục
AI_MODEL_IMG_W = 200  # kích thước ảnh đưa vào mạng
AI_MODEL_IMG_H = 66
AI_RECORD_EVERY_N_FRAMES = 3  # giãn cách khi ghi log, tránh ảnh liền kề trùng lặp

# =====================================================================
# BẢNG MÀU GIAO DIỆN
# =====================================================================

COLORS = {
    "bg": "#1e1e1e",
    "panel_bg": "#252526",
    "btn_bg": "#333336",
    "btn_active": "#4fc3f7",
    "text": "#e0e0e0",
    "accent": "#4fc3f7",
    "side_on": "#ffb300",
    "ok": "#66bb6a",
    "warn": "#ffb300",
    "error": "#ef5350",
}
