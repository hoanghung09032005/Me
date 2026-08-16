"""
config.py
----------
Nơi chứa TẤT CẢ các thông số/cấu hình chung của project.
Muốn đổi IP xe, kích thước cửa sổ, tên trạm đo... thì sửa ở ĐÂY,
không cần lục trong code giao diện.
"""
WINDOW_TITLE = "Xe Tự Lái - Điều Khiển & Giám Sát"
# Tăng theo cùng tỉ lệ với ô video (xem VIDEO_DISPLAY_W/H bên dưới) để còn
# đủ chỗ cho cột điều khiển bên trái + video lớn hơn bên phải không bị vỡ
# layout. Nếu màn hình bạn nhỏ hơn, có thể hạ lại WINDOW_SIZE và
# VIDEO_DISPLAY_W/H theo đúng tỉ lệ tương ứng.
WINDOW_SIZE = "1280x720"
# Độ phân giải THẬT của camera - PHẢI khớp với config.frame_size đang đặt
# trong ESP32_Camera.ino, chỉ dùng để biết tỉ lệ khung hình (vd tính toán
# letterbox khi hiển thị).
# FRAMESIZE_VGA = 640x480 (trước đây FRAMESIZE_CIF = 400x296).
VIDEO_W, VIDEO_H = 640, 480
# Kích thước Ô HIỂN THỊ video trên giao diện - TÁCH RIÊNG khỏi VIDEO_W/H ở
# trên. mode1_manual.py và mode2_auto.py dùng 2 biến này để dựng khung
# hiển thị (letterbox, giữ đúng tỉ lệ khung hình, không bị méo dù kéo giãn
# cửa sổ).
VIDEO_DISPLAY_W, VIDEO_DISPLAY_H = 640, 480
# Số khung hình/giây tối đa mà GIAO DIỆN sẽ cập nhật lên màn hình.
# Dùng CHUNG cho cả Mode 1 và Mode 2.
# LƯU Ý QUAN TRỌNG: đây là tốc độ VẼ LẠI Ở PHÍA PC, hoàn toàn KHÔNG điều
# khiển tốc độ camera chụp/nén/gửi bên ESP32 - 2 thứ độc lập nhau. Hạ số
# này giúp PC đỡ tốn CPU vẽ vô ích khi nguồn chỉ thực sự ra được ~15fps,
# nhưng KHÔNG làm ESP32/camera đỡ nóng hay đỡ tốn dòng - muốn giảm tải
# thật sự bên camera phải sửa ở code ESP32 (app_httpd.cpp / camera_config).
UI_FPS = 16

# =====================================================================
# HẬU KỲ ẢNH PHÍA PC (image_enhance.py)
# ---------------------------------------------------------------------
# Camera là module FIXED-FOCUS (không chỉnh được tiêu cự vật lý), nên đây
# là hướng cải thiện độ nét/tương phản còn lại KHÔNG tốn thêm CPU/nhiệt/
# băng thông của ESP32-S3 - toàn bộ xử lý chạy trên PC, sau khi đã nhận
# xong khung hình. Thứ tự xử lý: denoise -> CLAHE -> sharpen (xem
# image_enhance.py để biết lý do thứ tự này quan trọng).
# =====================================================================
IMAGE_ENHANCE_ENABLED = True

# Bilateral filter - làm mượt nhiễu hạt/khối JPEG NHƯNG giữ nguyên các
# cạnh tương phản thật sự (chữ, viền vật thể). THÊM SAU KHI ghi nhận CLAHE
# + sharpen một mình làm lộ rõ kết cấu lưới ô vuông (nhiễu khối JPEG bị
# khuếch đại) trên vùng phẳng như tường - denoise trước sẽ giảm hẳn hiện
# tượng này. 0 = tắt hẳn bước này.
IMAGE_DENOISE_ENABLED = True
# Đường kính vùng lân cận mỗi pixel xét tới khi lọc (px). Tăng làm mượt
# vùng rộng hơn nhưng CHẬM ĐI RÕ RỆT (đây là tham số ảnh hưởng tốc độ
# nhiều nhất trong 3 tham số denoise) - nếu PC yếu và giật hình sau khi
# bật, hạ số này trước tiên (vd xuống 3).
IMAGE_DENOISE_D = 5
# Mức chênh lệch MÀU tối đa vẫn coi là "cùng một vùng" để làm mượt chung -
# tăng làm mượt mạnh hơn (giảm nhiễu tốt hơn) nhưng dễ mất chi tiết nhỏ
# nếu quá cao.
IMAGE_DENOISE_SIGMA_COLOR = 40
# Mức chênh lệch KHOẢNG CÁCH (không gian) tối đa vẫn coi là "cùng một
# vùng". Cùng logic tăng/giảm như trên.
IMAGE_DENOISE_SIGMA_SPACE = 40

# Cường độ làm nét viền (unsharp mask). 0 = tắt hẳn sharpen.
# Bắt đầu ở mức vừa phải (0.8) - tăng dần nếu muốn nét hơn nữa, nhưng quá
# ~1.5 dễ sinh viền quầng (halo) và nhiễu hạt rõ rệt quanh các cạnh tương
# phản mạnh, nhìn "giả" thay vì nét thật.
IMAGE_SHARPEN_AMOUNT = 0.8
# Bán kính (sigma) của lớp làm mờ dùng để tính viền cần làm nét. Nhỏ hơn
# (~1.0) làm nét chi tiết mịn/nhỏ hơn, lớn hơn (~2.0) làm nét các mảng
# khối lớn hơn. 1.5 là điểm khởi đầu cân bằng cho khung 640x480.
IMAGE_SHARPEN_SIGMA = 1.5
# Giới hạn tương phản CLAHE. 0 = tắt hẳn CLAHE, chỉ còn sharpen.
# Càng cao tương phản cục bộ càng mạnh - dễ sinh nhiễu ở vùng thiếu sáng
# nếu vượt quá ~4.0. 2.0 phù hợp cho ảnh trong nhà thiếu sáng như hiện tại.
IMAGE_CLAHE_CLIP_LIMIT = 2.0

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
# Bảng màu dùng chung cho giao diện (đổi ở đây sẽ đổi màu toàn bộ app)
COLORS = {
    "bg": "#1e1e1e",
    "panel_bg": "#252526",
    "btn_bg": "#333336",
    "btn_active": "#4fc3f7",
    "text": "#e0e0e0",
    "accent": "#4fc3f7",
    # "side_on": mode1_manual.py và mode2_auto.py đều gọi
    # config.COLORS["side_on"] khi vẽ 2 cảm biến hông (đã xác nhận qua
    # grep trực tiếp trên file GUI mới nhất) - PHẢI đúng tên khoá này,
    # không phải "side_accent".
    "side_on": "#ffb300",
    "ok": "#66bb6a",
    "warn": "#ffb300",
    "error": "#ef5350",
}