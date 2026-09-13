"""
ai_model.py
-----------
Kiến trúc mạng + hàm tiền xử lý dùng CHUNG cho train.py (train một lần,
chạy tách biệt) và mode4_ai.py (load model đã train, chạy real-time).

Output của model: 1 số thực trong khoảng xấp xỉ [-4.0, 4.0], TƯƠNG THÍCH
TRỰC TIẾP với `error` mà pipeline OpenCV cổ điển trong mode4_ai.py đang
tính (deviation * 4.0) - nhờ vậy không cần đổi giao thức gửi xuống STM32.
"""

import numpy as np
import cv2

try:
    import torch
    import torch.nn as nn
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


IMG_W = 200
IMG_H = 66


if TORCH_AVAILABLE:
    class LineErrorNet(nn.Module):
        """PilotNet thu nhỏ - 5 lớp conv + 3 lớp fully-connected,
        xuất thẳng 1 giá trị error (không cần bước tính centroid)."""

        def __init__(self):
            super().__init__()
            self.conv = nn.Sequential(
                nn.Conv2d(3, 24, 5, stride=2), nn.ELU(),
                nn.Conv2d(24, 36, 5, stride=2), nn.ELU(),
                nn.Conv2d(36, 48, 5, stride=2), nn.ELU(),
                nn.Conv2d(48, 64, 3), nn.ELU(),
                nn.Conv2d(64, 64, 3), nn.ELU(),
            )
            # Với input 66x200, conv phía trên ra đúng shape (64, 1, 18)
            self.fc = nn.Sequential(
                nn.Flatten(),
                nn.Linear(64 * 1 * 18, 100), nn.ELU(),
                nn.Linear(100, 50), nn.ELU(),
                nn.Linear(50, 10), nn.ELU(),
                nn.Linear(10, 1),
            )

        def forward(self, x):
            x = self.conv(x)
            x = self.fc(x)
            return x.squeeze(1)


def preprocess_frame(bgr_frame, crop_top_ratio):
    """
    Tiền xử lý MỘT ảnh BGR (từ camera) thành tensor sẵn sàng đưa vào model.
    PHẢI dùng giống hệt hàm này ở cả lúc lưu dataset (nếu lưu ảnh đã crop)
    và lúc inference, nếu không model sẽ học/đoán sai lệch phân bố ảnh.

    crop_top_ratio: dùng chung config.AI_LINE_CROP_TOP để ROI nhất quán
    với pipeline OpenCV cổ điển (cùng vùng ảnh, dễ so sánh 2 phương pháp).
    """
    h, w = bgr_frame.shape[:2]
    crop_y = int(h * crop_top_ratio)
    roi = bgr_frame[crop_y:h, 0:w]

    yuv = cv2.cvtColor(roi, cv2.COLOR_BGR2YUV)
    resized = cv2.resize(yuv, (IMG_W, IMG_H))
    normalized = resized.astype(np.float32) / 255.0
    chw = np.transpose(normalized, (2, 0, 1))  # HWC -> CHW
    return chw


def frame_to_tensor(bgr_frame, crop_top_ratio):
    chw = preprocess_frame(bgr_frame, crop_top_ratio)
    return torch.from_numpy(chw).unsqueeze(0)  # thêm batch dimension