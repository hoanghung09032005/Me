"""
image_enhance.py
------------------
Hậu kỳ ảnh phía PC - tăng cảm giác sắc nét/tương phản cho luồng video từ
camera FIXED-FOCUS (không chỉnh được tiêu cự vật lý). Toàn bộ xử lý chạy
NGAY TRÊN MÁY TÍNH, sau khi đã nhận và giải mã JPEG - không tốn thêm bất
kỳ CPU/RAM/nhiệt/băng thông nào của ESP32-S3, tách bạch hoàn toàn với các
rủi ro nóng máy đã ghi chú ở ESP32_Camera.ino.

CỐ TÌNH KHÔNG áp dụng hàm này trong video_stream.py (nơi lưu frame "raw"
gốc dùng cho AI/CV sau này) - chỉ gọi enhance_frame() ở bước HIỂN THỊ
trong mode1_manual.py/mode2_auto.py, để dữ liệu thô cho các module nhận
diện line/vật cản sau này không bị ảnh hưởng bởi hậu kỳ mang tính thẩm mỹ.

THỨ TỰ XỬ LÝ (quan trọng): denoise -> CLAHE -> sharpen.
Lần đầu chỉ có CLAHE + sharpen, kết quả cho thấy vùng phẳng ít chi tiết
(tường, mảng màu đồng nhất) xuất hiện kết cấu hạt/lưới ô vuông rõ rệt -
đây là nhiễu khối nén JPEG (8x8 pixel) và nhiễu cảm biến trong điều kiện
thiếu sáng bị chính CLAHE (khuếch đại tương phản cục bộ) và unsharp mask
(khuếch đại biên tần số cao) LÀM LỘ RÕ THÊM, không phải do camera tệ hơn.
Thêm bilateral filter (làm mượt NHƯNG GIỮ NGUYÊN các cạnh tương phản thật
sự, ví dụ mép chữ/mép hộp) ngay TRƯỚC 2 bước kia để lọc bớt nhiễu này
trước khi bị khuếch đại lên, thay vì sharpen thẳng lên ảnh còn nhiễu.

Bật/tắt và chỉnh cường độ ở config.py (IMAGE_ENHANCE_ENABLED,
IMAGE_DENOISE_*, IMAGE_SHARPEN_*, IMAGE_CLAHE_CLIP_LIMIT).
"""

import cv2

import config


def enhance_frame(bgr_frame):
    """Nhận 1 frame BGR (từ cv2.imdecode), trả về frame đã hậu kỳ.
    An toàn để gọi trên mọi kích thước ảnh - không giả định resolution cụ
    thể, nên không cần sửa lại nếu sau này đổi FRAMESIZE trong ESP32."""
    if not config.IMAGE_ENHANCE_ENABLED:
        return bgr_frame

    frame = bgr_frame

    if config.IMAGE_DENOISE_ENABLED:
        # Bilateral filter: làm mượt vùng phẳng (nhiễu hạt/khối JPEG) NHƯNG
        # giữ nguyên các cạnh có chênh lệch màu lớn (chữ, viền vật thể) -
        # khác Gaussian blur thường (sẽ làm mờ luôn cả chữ, phản tác dụng
        # với mục đích làm ảnh rõ hơn). Đây là bước nặng nhất trong toàn bộ
        # pipeline - nếu PC yếu và thấy giật hình rõ rệt, hạ IMAGE_DENOISE_D
        # trước tiên (ảnh hưởng tốc độ nhiều nhất trong 3 tham số).
        frame = cv2.bilateralFilter(
            frame,
            config.IMAGE_DENOISE_D,
            config.IMAGE_DENOISE_SIGMA_COLOR,
            config.IMAGE_DENOISE_SIGMA_SPACE,
        )

    if config.IMAGE_CLAHE_CLIP_LIMIT > 0:
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l_channel, a_channel, b_channel = cv2.split(lab)
        clahe = cv2.createCLAHE(
            clipLimit=config.IMAGE_CLAHE_CLIP_LIMIT, tileGridSize=(8, 8)
        )
        l_channel = clahe.apply(l_channel)
        frame = cv2.cvtColor(
            cv2.merge((l_channel, a_channel, b_channel)), cv2.COLOR_LAB2BGR
        )

    if config.IMAGE_SHARPEN_AMOUNT > 0:
        # Unsharp mask cổ điển: làm mờ nhẹ rồi trừ ngược lại để khuếch đại
        # phần "biên" (chi tiết tần số cao) - KHÔNG phải AI, chỉ là một
        # phép lọc ảnh thông thường, chạy tức thời kể cả trên máy yếu.
        blurred = cv2.GaussianBlur(frame, (0, 0), config.IMAGE_SHARPEN_SIGMA)
        frame = cv2.addWeighted(
            frame,
            1 + config.IMAGE_SHARPEN_AMOUNT,
            blurred,
            -config.IMAGE_SHARPEN_AMOUNT,
            0,
        )

    return frame
