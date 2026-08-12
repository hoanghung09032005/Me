"""
communication.py
-------------------
Nơi gom TẤT CẢ code giao tiếp với xe thật.

ĐÃ CẬP NHẬT theo giao thức MỚI của main.c (khác bản cũ):
    - Vào Auto      : "A\\n"                  (trước đây "START\\n")
    - Vào Manual    : "M\\n"                  (trước đây "MANUAL\\n")
    - Dừng khẩn cấp : "S\\n"                  (giữ nguyên "STOP" nhưng chỉ 1 ký tự "S")
    - Lái tay       : "<F/B/L/R> <0-100>\\n"  (trước đây "M,<F/B/L/R>,<0-999>\\n")
      Ví dụ: "F 80\\n" = tiến với 80% tốc độ.

LƯU Ý QUAN TRỌNG: firmware STM32 hiện tại (main.c) CHƯA gửi bất kỳ phản hồi
nào về PC (chưa có UART_SendString, phần telemetry còn là TODO rỗng). Tức
là dù gửi lệnh đúng giao thức, ACK/LOG sẽ KHÔNG xuất hiện cho tới khi firmware
được bổ sung phần gửi UART - đây là việc cần làm tiếp theo bên STM32, không
phải lỗi của file này.

VỀ ĐA LUỒNG (quan trọng):
Việc đọc socket chạy ở luồng nền, nhưng Tkinter KHÔNG an toàn khi bị gọi
từ luồng khác. Nên luồng nền chỉ bỏ dữ liệu vào một hàng đợi (queue), còn
giao diện tự lấy ra bằng poll_events() trong vòng lặp after() của mình.
"""

import queue
import socket
import threading
import time

import config

# Bảng dịch tên hướng (dễ đọc trong code giao diện) sang ký tự STM32 hiểu.
# LƯU Ý: không còn "STOP" trong bảng này - 'S' giờ dành riêng cho lệnh dừng
# khẩn cấp toàn cục (thoát về MODE_IDLE), không phải 1 hướng lái.
DIR_CODE = {
    "FORWARD": "F",
    "BACKWARD": "B",
    "LEFT": "L",
    "RIGHT": "R",
}


class CarLink:
    """Một đường dây TCP tới ESP32-S3, dùng CHUNG cho cả Mode 1 và Mode 2."""

    def __init__(self):
        self._sock = None
        self._thread = None
        self._send_lock = threading.Lock()
        self.connected = False
        self.events = queue.Queue()

        self.sent_count = 0
        self.recv_count = 0

    # ==================================================================
    # KẾT NỐI
    # ==================================================================
    def connect(self, ip=None, port=None):
        """Mở kết nối. Trả về True nếu thành công."""
        if self.connected:
            return True

        ip = ip or config.CAR_IP
        port = port or config.CAR_PORT

        try:
            sock = socket.create_connection((ip, port), timeout=config.CONNECT_TIMEOUT)
            sock.settimeout(0.2)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError as e:
            self._emit("error", f"Không nối được tới {ip}:{port} - {e}")
            self._emit("error", self._diagnose(e))
            self._emit("state", "disconnected")
            return False

        self._sock = sock
        self.connected = True
        self.sent_count = 0
        self.recv_count = 0
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

        self._emit("info", f"Đã kết nối xe tại {ip}:{port}")
        self._emit("state", "connected")
        return True

    def connect_async(self, ip=None, port=None):
        """Kết nối ở luồng nền, dùng cho nút bấm trên giao diện."""
        threading.Thread(target=self.connect, args=(ip, port), daemon=True).start()

    @staticmethod
    def _diagnose(err):
        text = str(err).lower()
        if "refused" in text or getattr(err, "errno", None) == 111:
            return ("→ Máy tính TỚI ĐƯỢC ESP32 nhưng cổng 8080 đang đóng. "
                    "Nhiều khả năng ESP32 chưa được nạp code mới có car_bridge.")
        if "timed out" in text or "timeout" in text or "unreachable" in text:
            return ("→ Không thấy ESP32 ở địa chỉ này. Kiểm tra IP in ở Serial Monitor, "
                    "và xem máy tính có đang chung mạng WiFi với xe không.")
        return "→ Chạy chan_doan.py để biết chính xác tầng nào đang hỏng."

    def close(self, send_stop=True):
        """Đóng kết nối. Mặc định phanh xe trước khi ngắt cho an toàn."""
        if not self.connected:
            return
        if send_stop:
            self.send_raw("S")   # lệnh dừng khẩn cấp mới, 1 ký tự
            time.sleep(0.05)

        self.connected = False
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self._sock.close()
        except OSError:
            pass
        self._sock = None
        self._emit("info", "Đã ngắt kết nối xe.")
        self._emit("state", "disconnected")

    # ==================================================================
    # GỬI LỆNH
    # ==================================================================
    def send_raw(self, text):
        """Gửi một dòng lệnh thô. Tự thêm '\\n' vì STM32 chốt lệnh ở ký tự này."""
        if not self.connected or self._sock is None:
            return False
        try:
            with self._send_lock:
                self._sock.sendall((text + "\n").encode("ascii", errors="ignore"))
            self.sent_count += 1
            return True
        except OSError as e:
            self._emit("error", f"Mất kết nối khi gửi lệnh: {e}")
            self._drop()
            return False

    def enter_manual(self):
        """Chuyển xe sang chế độ lái tay."""
        return self.send_raw("M")

    def drive(self, direction, speed):
        """
        Lệnh lái tay.
        direction : "FORWARD" | "BACKWARD" | "LEFT" | "RIGHT"
        speed     : 0-100 (%) - ĐÚNG ĐƠN VỊ % như trước, main.c tự quy đổi
                    sang PWM 0-999 bên trong Mode1_Apply_Command().
        """
        code = DIR_CODE.get(direction)
        if code is None:
            return False
        return self.send_raw(f"{code} {int(speed)}")

    def brake(self):
        """
        Phanh nhưng vẫn giữ ở chế độ lái tay (KHÔNG thoát về IDLE).
        Gửi hướng bất kỳ (dùng 'F') với tốc độ 0 - vì tốc độ 0 thì hướng
        không còn ý nghĩa, motor về 0 nhưng car_mode vẫn là MANUAL.
        LƯU Ý: không dùng "S 0" ở đây - 'S' đứng đầu dòng bị main.c chặn ưu
        tiên để chuyển thẳng về MODE_IDLE, sẽ thoát khỏi chế độ lái tay chứ
        không đơn thuần phanh tại chỗ.
        """
        return self.send_raw("F 0")

    def start_auto(self):
        """Bật chế độ tự động dò line (Mode 2)."""
        return self.send_raw("A")

    def set_auto_speed(self, speed):
        """Đặt tốc độ cơ bản cho Mode 2 theo phần trăm (0-100)."""
        speed = max(0, min(100, int(speed)))
        return self.send_raw(f"V {speed}")

    def stop_all(self):
        """Dừng khẩn cấp - thoát về MODE_IDLE, hoạt động ở bất kỳ mode nào."""
        return self.send_raw("S")

    def ping(self):
        """
        CHƯA HỖ TRỢ ở firmware hiện tại - main.c không xử lý ký tự 'P'.
        Nếu gọi lúc đang ở MANUAL, 'P' sẽ rơi vào nhánh lái tay và bị hiểu
        nhầm thành 1 hướng không xác định (mode1.c mặc định về PWM 0) -
        tức là VÔ TÌNH PHANH XE. Không gọi hàm này cho tới khi main.c được
        bổ sung xử lý lệnh PING/PONG.
        """
        raise NotImplementedError(
            "ping() chưa được firmware hiện tại hỗ trợ - có thể gây phanh "
            "nhầm nếu đang ở chế độ MANUAL. Bỏ qua cho tới khi main.c được cập nhật."
        )

    # ==================================================================
    # NHẬN DỮ LIỆU (chạy ở luồng nền)
    # ==================================================================
    def _rx_loop(self):
        buf = ""
        while self.connected:
            try:
                data = self._sock.recv(1024)
            except socket.timeout:
                continue
            except OSError as e:
                if self.connected:
                    self._emit("error", f"Lỗi đọc dữ liệu: {e}")
                    self._drop()
                return

            if not data:
                if self.connected:
                    self._emit("error", "Xe đã đóng kết nối.")
                    self._drop()
                return

            buf += data.decode("utf-8", errors="replace")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                self._handle_line(line.strip())

            if len(buf) > 4096:
                buf = ""

    def _handle_line(self, line):
        if not line:
            return

        if not line.startswith("[ESP32-S3]"):
            self.recv_count += 1

        if line.startswith("LOG,"):
            parts = line.split(",")
            if len(parts) < 5:
                return
            try:
                telemetry = {
                    # STM32 sends the five-sensor mask in decimal, not hexadecimal.
                    "sensor": int(parts[1]),
                    "error": int(parts[2]) / 100.0,
                    "pwm_l": int(parts[3]),
                    "pwm_r": int(parts[4]),
                    "t": time.time(),
                }

                # New STM32 protocol:
                # LOG,mask,error_x100,pwm_l,pwm_r,distance_x10,temp_x10,humidity_x10
                if len(parts) >= 6:
                    distance_x10 = int(parts[5])
                    telemetry["dist"] = (
                        distance_x10 / 10.0 if distance_x10 >= 0 else None
                    )
                if len(parts) >= 8:
                    temp_x10 = int(parts[6])
                    humidity_x10 = int(parts[7])
                    telemetry["temp_c"] = (
                        temp_x10 / 10.0 if temp_x10 > -32768 else None
                    )
                    telemetry["humidity_rh"] = (
                        humidity_x10 / 10.0 if humidity_x10 > -32768 else None
                    )

                self._emit("telemetry", telemetry)
            except ValueError:
                pass
        else:
            self._emit("info", line)

    def _drop(self):
        self.connected = False
        try:
            if self._sock:
                self._sock.close()
        except OSError:
            pass
        self._sock = None
        self._emit("state", "disconnected")

    # ==================================================================
    # HÀNG ĐỢI SỰ KIỆN CHO GIAO DIỆN
    # ==================================================================
    def _emit(self, kind, payload):
        self.events.put((kind, payload))

    def poll_events(self):
        items = []
        while True:
            try:
                items.append(self.events.get_nowait())
            except queue.Empty:
                break
        return items


link = CarLink()


# ---------------------------------------------------------------------
# Các hàm bọc ngoài, giữ nguyên tên cũ để code cũ gọi vẫn chạy
# ---------------------------------------------------------------------
def send_drive_command(direction, speed, log_func=None):
    ok = link.drive(direction, speed)
    if log_func and not ok:
        log_func(f"[{time.strftime('%H:%M:%S')}] Chưa kết nối xe - lệnh {direction} bị bỏ.")
    return ok


def start_auto_patrol(log_func=None):
    ok = link.start_auto()
    if log_func:
        log_func("[LỆNH] Bắt đầu chế độ tự động" if ok else "[LỖI] Chưa kết nối xe.")
    return ok


def stop_auto_patrol(log_func=None):
    ok = link.stop_all()
    if log_func:
        log_func("[LỆNH] Dừng xe" if ok else "[LỖI] Chưa kết nối xe.")
    return ok
