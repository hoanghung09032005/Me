"""
test_ai_model.py
-----------------
CHẠY TÁCH BIỆT, KHÔNG LIÊN QUAN đến GUI hay xe thật - giống train.py.
Mục đích: kiểm chứng LineErrorNet (đã train ra config.AI_MODEL_PATH) có
hoạt động tốt hay chưa, TRƯỚC KHI đụng vào phần cứng (đổi cam, cho xe chạy
AI thật ở mode4_ai.py).

3 chế độ, tăng dần mức độ phụ thuộc phần cứng - nên chạy lần lượt A -> B:

  A) --mode dataset (mặc định, KHÔNG cần cam/xe)
     Chạy model qua toàn bộ ảnh đã có trong config.AI_DATASET_DIR, so sánh
     error dự đoán với label thật (labels.csv) - cùng cách train.py đánh
     giá val_loss, nhưng chi tiết hơn: MAE, RMSE, và liệt kê ra những ảnh
     model đoán SAI NHIỀU NHẤT để xem bằng mắt xem sai ở tình huống nào
     (cua gấp? ánh sáng khác? line mờ?).
     Lưu thêm 1 ảnh biểu đồ scatter (dự đoán vs thực tế) để nhìn tổng quan.

     Cách chạy:
         cd robot_car_project
         python3 test_ai_model.py --mode dataset

  B) --mode live (CẦN ESP32-CAM đang stream, KHÔNG cần kết nối STM32/xe)
     Đọc luồng MJPEG y hệt mode4_ai.py, chạy model theo thời gian thực,
     CHỈ IN ra error dự đoán + FPS - KHÔNG gọi communication.link, không
     gửi bất kỳ lệnh nào xuống xe. An toàn 100% để quan sát độ ổn định
     (nhiễu/giật giữa các frame liên tiếp) trước khi cho xe chạy thật.

     Cách chạy:
         python3 test_ai_model.py --mode live
         python3 test_ai_model.py --mode live --url http://192.168.1.121:81/stream

Sau khi cả 2 bước trên đều ổn, mới nên bật checkbox "Dùng AI Model" trong
mode4_ai.py và cho xe chạy thật.
"""

import argparse
import csv
import os
import time

import cv2
import numpy as np

import config
from ai_model import LineErrorNet, preprocess_frame, TORCH_AVAILABLE

if TORCH_AVAILABLE:
    import torch


def _load_model():
    if not TORCH_AVAILABLE:
        raise RuntimeError("Chưa cài PyTorch (pip install torch --break-system-packages).")
    if not os.path.exists(config.AI_MODEL_PATH):
        raise RuntimeError(
            f"Không tìm thấy model tại {config.AI_MODEL_PATH} - chạy train.py trước."
        )
    model = LineErrorNet()
    model.load_state_dict(torch.load(config.AI_MODEL_PATH, map_location="cpu"))
    model.eval()
    return model


# ======================================================================
# A) ĐÁNH GIÁ TRÊN DATASET - KHÔNG CẦN CAM/XE
# ======================================================================
def evaluate_dataset(worst_n=15):
    model = _load_model()

    label_path = os.path.join(config.AI_DATASET_DIR, "labels.csv")
    samples = []
    with open(label_path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            samples.append((row["file"], float(row["error"])))

    print(f"Tổng số mẫu: {len(samples)}")
    if not samples:
        print("Dataset rỗng - không có gì để đánh giá.")
        return

    preds, labels, errs, files = [], [], [], []

    with torch.no_grad():
        for filename, label_error in samples:
            img_path = os.path.join(config.AI_DATASET_DIR, filename)
            bgr = cv2.imread(img_path)
            if bgr is None:
                print(f"  [BỎ QUA] Không đọc được ảnh: {filename}")
                continue
            chw = preprocess_frame(bgr, config.AI_LINE_CROP_TOP)
            tensor = torch.from_numpy(chw).unsqueeze(0)
            pred_error = float(model(tensor).item())

            preds.append(pred_error)
            labels.append(label_error)
            errs.append(pred_error - label_error)
            files.append(filename)

    preds = np.array(preds)
    labels = np.array(labels)
    errs = np.array(errs)

    mae = np.mean(np.abs(errs))
    rmse = np.sqrt(np.mean(errs ** 2))
    max_abs_err = np.max(np.abs(errs))
    corr = np.corrcoef(preds, labels)[0, 1] if len(preds) > 1 else float("nan")

    print("\n===== KẾT QUẢ ĐÁNH GIÁ =====")
    print(f"MAE  (sai số tuyệt đối trung bình) : {mae:.4f}")
    print(f"RMSE                                : {rmse:.4f}")
    print(f"Sai số lớn nhất                      : {max_abs_err:.4f}")
    print(f"Hệ số tương quan (pred vs label)      : {corr:.4f}  "
          f"(càng gần 1.0 càng tốt)")

    # Liệt kê những ảnh model đoán sai nhiều nhất - xem bằng mắt để hiểu
    # NGUYÊN NHÂN (cua gấp, ánh sáng, line mờ, nhiễu nền...), không chỉ
    # nhìn con số tổng quát.
    order = np.argsort(-np.abs(errs))[:worst_n]
    print(f"\n===== {min(worst_n, len(order))} ẢNH MODEL ĐOÁN SAI NHIỀU NHẤT =====")
    print(f"{'file':<20}{'label':>10}{'predicted':>12}{'sai số':>10}")
    for i in order:
        print(f"{files[i]:<20}{labels[i]:>10.3f}{preds[i]:>12.3f}{errs[i]:>10.3f}")

    # Biểu đồ tổng quan - lưu ra file để mở xem, không cần GUI hiện tại
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 2, figsize=(11, 5))

        axes[0].scatter(labels, preds, s=8, alpha=0.5)
        lim = max(np.max(np.abs(labels)), np.max(np.abs(preds)), 4.0)
        axes[0].plot([-lim, lim], [-lim, lim], "r--", linewidth=1)
        axes[0].set_xlabel("Label thật (từ labels.csv)")
        axes[0].set_ylabel("Model dự đoán")
        axes[0].set_title("Dự đoán vs Thực tế")
        axes[0].grid(alpha=0.3)

        axes[1].hist(errs, bins=40)
        axes[1].set_xlabel("Sai số (predicted - label)")
        axes[1].set_ylabel("Số lượng mẫu")
        axes[1].set_title("Phân bố sai số")
        axes[1].grid(alpha=0.3)

        fig.tight_layout()
        out_path = "ai_model_eval.png"
        fig.savefig(out_path, dpi=120)
        print(f"\nĐã lưu biểu đồ: {out_path}")
    except ImportError:
        print("\n(Chưa cài matplotlib nên bỏ qua bước vẽ biểu đồ - "
              "pip install matplotlib --break-system-packages nếu muốn có.)")


# ======================================================================
# B) TEST REAL-TIME QUA STREAM - CHỈ QUAN SÁT, KHÔNG GỬI LỆNH XUỐNG XE
# ======================================================================
def live_stream_test(url):
    # Import trễ - chỉ cần khi thật sự chạy chế độ live, tránh việc chạy
    # --mode dataset cũng phải có thư viện cv2 GUI/threading không cần thiết.
    from video_stream import MJPEGReader

    model = _load_model()
    print(f"Đang mở stream: {url}")
    print("CHỈ QUAN SÁT - không gửi bất kỳ lệnh nào xuống xe. Ctrl+C để dừng.\n")

    errors_seen = []

    def on_error(msg):
        print(f"[LỖI STREAM] {msg}")

    reader = MJPEGReader(url, on_error=on_error)
    reader.start()

    last_frame_count = 0
    last_report_time = time.time()

    try:
        while True:
            frame = reader.get_latest_frame()
            if frame is not None:
                chw = preprocess_frame(frame, config.AI_LINE_CROP_TOP)
                tensor = torch.from_numpy(chw).unsqueeze(0)
                with torch.no_grad():
                    pred_error = float(model(tensor).item())
                errors_seen.append(pred_error)
                if len(errors_seen) > 200:
                    errors_seen.pop(0)

                now = time.time()
                if now - last_report_time >= 0.5:
                    fps = (reader.frames_received - last_frame_count) / (now - last_report_time)
                    last_frame_count = reader.frames_received
                    last_report_time = now
                    jitter = np.std(errors_seen[-30:]) if len(errors_seen) >= 2 else 0.0
                    print(f"error={pred_error:+.3f}   "
                          f"độ dao động 30 mẫu gần nhất={jitter:.3f}   "
                          f"~{fps:.1f} fps nguồn")
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("\nDừng.")
    finally:
        reader.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["dataset", "live"], default="dataset")
    parser.add_argument("--url", default=config.DEFAULT_STREAM_URL,
                         help="URL stream MJPEG (chỉ dùng khi --mode live)")
    parser.add_argument("--worst-n", type=int, default=15,
                         help="Số ảnh sai nhiều nhất cần liệt kê (chỉ dùng khi --mode dataset)")
    args = parser.parse_args()

    if args.mode == "dataset":
        evaluate_dataset(worst_n=args.worst_n)
    else:
        live_stream_test(args.url)
