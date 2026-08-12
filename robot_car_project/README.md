# Robot Car Project - Giao diện điều khiển & giám sát

Giao diện Python (Tkinter) cho xe tự lái dùng ESP32-S3 quay video.
Chia làm 2 chế độ:

- **Mode 1 - Điều khiển thủ công**: bấm nút / phím mũi tên để lái xe,
  đồng thời xem video trực tiếp từ xe.
- **Mode 2 - Tự động dò line + tránh vật cản**: xe tự chạy, vẫn xem
  được stream, kèm đo nhiệt độ/độ ẩm tại vài trạm cố định. *(Hiện mới
  là khung sườn giao diện, logic tự lái/cảm biến thật sẽ bổ sung sau.)*

## Cài đặt

```bash
pip install -r requirements.txt
```

(Tkinter có sẵn trong Python chuẩn, không cần cài thêm - trừ khi máy
bạn cài Python bản rút gọn thì cần cài thêm gói `python3-tk`.)

## Chạy chương trình

```bash
python main.py
```

Chưa có xe thật vẫn chạy được nhờ nút **"Test Mode (giả lập)"** ở cả
2 mode - hiện hình ảnh giả động để kiểm tra giao diện.

## Cấu trúc project

```
robot_car_project/
├── main.py            # Chạy chương trình, chuyển đổi giữa Mode 1 / Mode 2
├── config.py           # Mọi thông số cấu hình (IP, kích thước, màu, trạm đo...)
├── communication.py     # Gửi lệnh xuống xe (hiện chỉ log, sẽ nối phần cứng sau)
├── video_stream.py      # Đọc luồng MJPEG từ camera ESP32-S3 (dùng chung 2 mode)
├── mode1_manual.py      # Giao diện + logic Mode 1
├── mode2_auto.py        # Giao diện Mode 2 (khung sườn, sẽ hoàn thiện sau)
└── requirements.txt
```

## Khi ráp phần cứng thật, cần sửa gì?

1. **`config.py`**
   - `DEFAULT_STREAM_URL`: đổi thành địa chỉ MJPEG thật của ESP32-S3
     (thường dạng `http://<ip>:81/stream`).
   - `CAR_IP`: địa chỉ IP board điều khiển xe.
   - `CHECKPOINTS`: đặt tên/thêm bớt trạm đo cho đúng sa bàn thật.

2. **`communication.py`**
   - `send_drive_command()`: mở phần TODO, gửi HTTP/UART thật xuống xe.
   - `start_auto_patrol()` / `stop_auto_patrol()`: gửi lệnh bật/tắt
     chế độ tự động xuống board.
   - `read_sensor_data()`: đọc dữ liệu cảm biến nhiệt độ/độ ẩm thật
     (vd DHT11/DHT22 qua ESP32) thay vì trả về `None`.

3. **`mode2_auto.py`**
   - Hiện dùng nút "Giả lập đo dữ liệu (test)" để test giao diện bảng
     trạm đo. Sau này thay bằng việc: khi xe báo đã tới 1 trạm (qua
     HTTP callback hoặc polling), gọi `communication.read_sensor_data()`
     rồi cập nhật lên `checkpoint_labels` giống hệt hàm
     `_simulate_sensor_reading()` đang làm - chỉ đổi nguồn dữ liệu từ
     giả lập sang thật.
   - Thêm logic dò line + tránh vật cản thật (có thể chạy trên board,
     giao diện Python chỉ cần hiển thị trạng thái xe gửi về).

Vì `video_stream.py` và `communication.py` dùng chung cho cả 2 mode,
sửa 1 chỗ là áp dụng cho cả 2 giao diện, không phải sửa lặp lại.

## Về độ trễ khi xem stream (đã tối ưu)

`video_stream.py` tự tách từng frame JPEG từ luồng MJPEG (giống cách
làm gốc), nhưng giải mã bằng `cv2.imdecode` (OpenCV) thay vì PIL - nhanh
hơn đáng kể. *(Lưu ý: không dùng `cv2.VideoCapture(url)` trực tiếp vì
luồng MJPEG kiểu `multipart/x-mixed-replace` của ESP32-CAM/S3 khiến
backend FFmpeg trên Windows bị lỗi demuxer.)* Luôn ghi đè lên **1 frame
duy nhất** trong bộ nhớ - nếu giao diện xử lý không kịp tốc độ camera
gửi, các frame cũ bị bỏ thẳng chứ không dồn lại, nên không bị trễ tăng
dần theo thời gian. Giao diện chỉ lấy frame mới nhất theo nhịp cố định
`config.UI_FPS` (mặc định 15 FPS) và không resize ảnh trong Python.

Để đạt hiệu quả tốt nhất, phía ESP32-S3 (firmware) cũng nên chỉnh:
- `config.VIDEO_W`, `config.VIDEO_H` trong `config.py` phải TRÙNG với
  độ phân giải camera đang cấu hình (vd `FRAMESIZE_QVGA` = 320x240).
- Có thể hạ xuống `FRAMESIZE_QQVGA` (160x120) nếu ưu tiên độ trễ thấp
  hơn là hình đẹp.
- Giảm `jpeg_quality` (vd 15-20) để ảnh nhẹ hơn, gửi nhanh hơn.
- Ưu tiên dùng WiFi 2.4GHz ít nhiễu, hạn chế tải mạng khác cùng lúc.
