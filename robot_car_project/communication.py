"""
communication.py
-------------------
Nơi gom TẤT CẢ code giao tiếp với xe thật.

GIAO THỨC hiện tại của main.c:
    - Vào Auto      : "A\\n"
    - Vào Manual    : "M\\n"
    - Dừng khẩn cấp : "S\\n"
    - Lái tay       : "<F/B/L/R> <0-100>\\n"   Ví dụ: "F 80\\n" = tiến 80%.
    - Đặt tốc độ Auto: "V <0-100>\\n"  (KHÔNG còn được GUI gọi - Mode 2 dùng
      tốc độ cố định DEFAULT_BASE_SPEED trong firmware, xem mode2_obstacle.c)
    - Heartbeat/giữ kết nối: "H\\n"  (BẮT BUỘC gửi định kỳ trong lúc Mode 2
      đang chạy - xem heartbeat() bên dưới và AUTO_LINK_TIMEOUT_TICKS trong
      main.c. Không đổi mode, không reset trạng thái PID/né vật cản, chỉ
      chứng minh đường truyền còn sống.)

Firmware STM32 gửi phản hồi về PC: "ACK,S" / "ACK,M" / "ACK,A" / "ACK,V" /
"ACK,DRIVE" / "ACK,H", dòng telemetry "LOG,...", và khi khởi động sẽ gửi
"STM32,BOOT,<xHz>" cùng "STM32,RESET_CAUSE,<lý do>" (dòng này rất hữu ích để
biết STM32 có đang tự reset do sụt áp nguồn giữa lúc chạy hay không).

Định dạng LOG (đã có thêm 2 mắt biên so với bản trước):
    LOG,line_mask,side_left,side_right,error_x100,pwm_l,pwm_r,
        distance_x10,temp_x10,humidity_x10

VỀ AN TOÀN KHI MẤT KẾT NỐI (quan trọng - đọc trước khi đụng vào Mode 2):
Mode 2 (Auto) chạy HOÀN TOÀN TỰ ĐỘNG trên STM32 sau khi nhận "A" - không
cần lệnh liên tục từ PC để tiếp tục hoạt động. Nếu WiFi rớt "âm thầm" (mất
sóng, không phải người dùng chủ động bấm ngắt), việc ESP32 phát hiện TCP đã
chết có thể không tức thời, nên lệnh "S" tự động từ car_bridge.cpp có thể
đến chậm hoặc không đến. Để xe KHÔNG chạy tự động vô thời hạn khi mất giám
sát, firmware tự dừng nếu không nhận được BẤT KỲ lệnh nào (kể cả heartbeat)
trong ~1.5 giây lúc đang ở Mode 2. VÌ VẬY: bất kỳ giao diện nào chạy Mode 2
đều PHẢI tự gọi link.heartbeat() định kỳ (khuyến nghị mỗi 200-400ms) trong
suốt thời gian xe đang tự động chạy - xem mode2_auto.py để biết cách làm.

VỀ ĐA LUỒNG (quan trọng):
Việc đọc socket chạy ở luồng nền, nhưng Tkinter KHÔNG an toàn khi bị gọi
từ luồng khác. Nên luồng nền chỉ bỏ dữ liệu vào một hàng đợi (queue), còn
giao diện tự lấy ra bằng poll_events() trong vòng lặp after() của mình.

VỀ TỰ ĐỘNG NỐI LẠI:
Nếu kết nối đang chạy tốt bỗng bị rớt NGOÀI Ý MUỐN (lỗi mạng, ESP32 tự
reset...), CarLink sẽ tự thử nối lại vài lần. Nếu bạn CHỦ ĐỘNG bấm "Ngắt kết
nối", nó sẽ KHÔNG tự nối lại.
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

# Số lần và khoảng chờ (giây) mỗi lần CarLink tự thử nối lại sau khi bị rớt
# kết nối ngoài ý muốn.
_RECONNECT_DELAYS = (1.5, 3.0, 5.0, 8.0, 10.0)


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

        self._last_ip = None
        self._last_port = None
        # True nghĩa là người dùng chủ động ngắt (hoặc chưa từng nối) -> KHÔNG
        # tự nối lại. Chỉ đặt False khi nối thành công.
        self._manual_disconnect = True
        self._reconnect_enabled = True

    # ==================================================================
    # KẾT NỐI
    # ==================================================================
    def connect(self, ip=None, port=None):
        """Mở kết nối. Trả về True nếu thành công."""
        if self.connected:
            return True

        ip = ip or config.CAR_IP
        port = port or config.CAR_PORT
        self._last_ip = ip
        self._last_port = port

        try:
            sock = socket.create_connection((ip, port), timeout=config.CONNECT_TIMEOUT)
            sock.settimeout(0.2)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._enable_keepalive(sock)
        except OSError as e:
            self._emit("error", f"Không nối được tới {ip}:{port} - {e}")
            self._emit("error", self._diagnose(e))
            self._emit("state", "disconnected")
            return False

        self._sock = sock
        self.connected = True
        self._manual_disconnect = False
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
    def _enable_keepalive(sock):
        """
        Bật TCP keepalive để phát hiện xe bị mất kết nối "âm thầm" (ví dụ
        ESP32 bị treo/tự reset do sụt áp nhưng không kịp gửi gói đóng kết
        nối) nhanh hơn nhiều so với mặc định hệ điều hành (Windows mặc định
        chờ tới 2 GIỜ trước khi phát hiện).

        LƯU Ý: đây là bảo vệ ở phía PC (giúp PC phát hiện xe rớt mạng nhanh
        hơn). Còn chiều ngược lại - STM32 tự phát hiện MẤT PC - dựa vào cơ
        chế heartbeat() độc lập bên dưới, không phụ thuộc TCP keepalive này.
        """
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except OSError:
            return
        try:
            if hasattr(socket, "SIO_KEEPALIVE_VALS"):
                # Windows: (bật, thời gian rảnh trước khi probe ms, chu kỳ probe ms)
                sock.ioctl(socket.SIO_KEEPALIVE_VALS, (1, 3000, 1000))
            else:
                # Linux / macOS
                if hasattr(socket, "TCP_KEEPIDLE"):
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 3)
                if hasattr(socket, "TCP_KEEPINTVL"):
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)
                if hasattr(socket, "TCP_KEEPCNT"):
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
        except OSError:
            pass

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
        self._manual_disconnect = True   # người dùng chủ động ngắt -> đừng tự nối lại
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
        """Bật chế độ tự động dò line (Mode 2). Tốc độ cố định theo firmware
        (DEFAULT_BASE_SPEED trong mode2_obstacle.c) - GUI không còn chỉnh
        tốc độ được nữa.

        QUAN TRỌNG: sau khi gọi hàm này, giao diện PHẢI gọi heartbeat() đều
        đặn (khuyến nghị mỗi 200-400ms) trong suốt thời gian Mode 2 đang
        chạy, nếu không firmware sẽ tự coi là mất kết nối và dừng hẳn sau
        ~1.5s (xem AUTO_LINK_TIMEOUT_TICKS trong main.c)."""
        return self.send_raw("A")

    def heartbeat(self):
        """
        Gửi tín hiệu "còn sống" - BẮT BUỘC gọi định kỳ trong lúc Mode 2
        (Auto) đang chạy. Không đổi car_mode, không reset trạng thái PID/né
        vật cản trên firmware - chỉ chứng minh đường truyền GUI<->STM32
        chưa bị đứt. Nếu STM32 không nhận được lệnh nào (kể cả lệnh này)
        trong ~1.5 giây lúc đang Auto, nó sẽ TỰ dừng hẳn (về IDLE, tắt motor
        + servo + ngừng scan vật cản) - xem AUTO_LINK_TIMEOUT_TICKS và case
        'H' trong Process_Command() ở main.c.

        Không cần gọi hàm này khi đang ở Mode 1 (Manual) - manual_watchdog
        trong mode1.c đã tự bảo vệ theo cơ chế riêng (PWM tự về 0 nếu quá
        400ms không có lệnh lái mới).
        """
        return self.send_raw("H")

    def set_auto_speed(self, speed):
        """Đặt tốc độ cơ bản cho Mode 2 theo phần trăm (0-100).
        GIỮ LẠI để tương thích/gỡ lỗi thủ công, nhưng GUI hiện KHÔNG gọi
        hàm này nữa (tốc độ Mode 2 đã cố định)."""
        speed = max(0, min(100, int(speed)))
        return self.send_raw(f"V {speed}")

    def stop_all(self):
        """Dừng khẩn cấp - thoát về MODE_IDLE, hoạt động ở bất kỳ mode nào."""
        return self.send_raw("S")

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
                self._emit("error", "Bộ đệm nhận dữ liệu tràn - có thể mất vài dòng log/telemetry.")
                buf = ""

    def _handle_line(self, line):
        if not line:
            return

        if not line.startswith("[ESP32-S3]"):
            self.recv_count += 1

        if line.startswith("LOG,"):
            # LOG,line_mask,side_left,side_right,error_x100,pwm_l,pwm_r,
            #     distance_x10,temp_x10,humidity_x10
            parts = line.split(",")
            if len(parts) < 7:
                return
            try:
                telemetry = {
                    "sensor": int(parts[1]),        # mặt nạ 5 bit - 5 mắt dò line giữa
                    # Khoá "side_l"/"side_r": đã đối chiếu trực tiếp với
                    # mode1_manual.py và mode2_auto.py bản mới nhất - cả 2
                    # đều đọc bằng data.get("side_l", 0) / data.get("side_r", 0)
                    # (self.side_cells = [T, P], không còn side_left_cell/
                    # side_right_cell riêng như bản cũ). Tên khoá ở đây phải
                    # khớp CHÍNH XÁC với GUI, không suy đoán.
                    "side_l": int(parts[2]),        # mắt biên trái
                    "side_r": int(parts[3]),        # mắt biên phải
                    "error": int(parts[4]) / 100.0,
                    "pwm_l": int(parts[5]),
                    "pwm_r": int(parts[6]),
                    "t": time.time(),
                }

                if len(parts) >= 8:
                    distance_x10 = int(parts[7])
                    telemetry["dist"] = (
                        distance_x10 / 10.0 if distance_x10 >= 0 else None
                    )
                if len(parts) >= 10:
                    temp_x10 = int(parts[8])
                    humidity_x10 = int(parts[9])
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
        self._maybe_auto_reconnect()

    # ==================================================================
    # TỰ ĐỘNG NỐI LẠI
    # ==================================================================
    def _maybe_auto_reconnect(self):
        """
        Chỉ tự nối lại khi kết nối bị RỚT NGOÀI Ý MUỐN (lỗi mạng, xe tự
        reset do sụt áp, v.v...). Nếu người dùng chủ động bấm "Ngắt kết
        nối" thì self._manual_disconnect đã được đặt True trong close() và
        hàm này sẽ không làm gì cả.
        """
        if self._manual_disconnect or not self._reconnect_enabled:
            return
        ip, port = self._last_ip, self._last_port
        if not ip:
            return

        def _retry_loop():
            for delay in _RECONNECT_DELAYS:
                time.sleep(delay)
                if self._manual_disconnect or self.connected:
                    return
                self._emit("info", f"[TỰ ĐỘNG] Đang thử nối lại xe tại {ip}:{port}...")
                if self.connect(ip, port):
                    return
            self._emit(
                "error",
                "Tự động nối lại thất bại nhiều lần - kiểm tra nguồn/WiFi xe "
                "rồi bấm 'Kết nối xe' lại.",
            )

        threading.Thread(target=_retry_loop, daemon=True).start()

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