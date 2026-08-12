"""
mode2_auto.py
---------------
Giao diện MODE 2: Xe tự động dò line (thuật toán PID chạy trên STM32),
có xem camera hành trình và bảng trạm đo nhiệt độ/độ ẩm.

Mode này DÙNG CHUNG đường kết nối với Mode 1 (communication.link), nên
nối ở mode nào thì mode kia cũng dùng được luôn, không phải nối 2 lần.

Phần dò line + PID nằm hoàn toàn trong STM32 (main.c). Giao diện chỉ:
  - gửi "START" / "STOP"
  - vẽ lại dữ liệu xe bắn về (5 mắt cảm biến, error, PWM 2 bánh)

Bảng trạm đo vẫn là khung sườn vì mạch chưa gắn cảm biến DHT nào -
xem ghi chú trong communication.read_sensor_data().
"""

import cv2
import tkinter as tk
from tkinter import ttk

from PIL import Image, ImageDraw, ImageTk

import config
import communication
from video_stream import MJPEGReader


class AutoPatrolFrame(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent, bg=config.COLORS["bg"])

        self.is_active = False
        self.cam_connected = False
        self.auto_running = False
        self.mjpeg = None
        self._video_after_id = None
        self._link_after_id = None

        self.sensor_readings = {}
        self.current_speed = tk.IntVar(value=60)

        self._build_layout()
        self._show_placeholder("Chưa có tín hiệu video")

    # ==================================================================
    # DỰNG GIAO DIỆN
    # ==================================================================
    def _build_layout(self):
        # CỘT TRÁI: Video & Nút chạy tự động (mở rộng tối đa)
        left = tk.Frame(self, bg=config.COLORS["bg"])
        left.pack(side="left", fill="both", expand=True, padx=10, pady=10)

        # CỘT PHẢI: Bảng điều khiển (cố định chiều rộng 360px để đủ chứa dữ liệu)
        right = tk.Frame(self, bg=config.COLORS["bg"], width=360)
        right.pack(side="right", fill="y", padx=10, pady=10)
        right.pack_propagate(False)

        # ---------------- CỘT TRÁI ----------------
        tk.Label(
            left,
            text="MÀN HÌNH AI / CAMERA HÀNH TRÌNH",
            bg=config.COLORS["bg"],
            fg=config.COLORS["accent"],
            font=("Segoe UI", 12, "bold"),
        ).pack(anchor="w")

        # Ô đen giữ chỗ cố định như Mode 1
        new_w = config.VIDEO_W * 2
        new_h = config.VIDEO_H * 2
        self.placeholder_img = tk.PhotoImage(width=new_w, height=new_h)
        self.video_label = tk.Label(left, image=self.placeholder_img, bg="black")
        self.video_label.pack(pady=(6, 8))
        self.video_label.config(width=new_w, height=new_h)
        self.video_label.bind("<Configure>", self._on_video_label_resize)
        self._video_display_size = (new_w, new_h)
        self._video_last_frame = None

        self.auto_btn = ttk.Button(
            left, text="▶ BẮT ĐẦU TỰ ĐỘNG", command=self.toggle_auto
        )
        self.auto_btn.pack(fill="x", ipady=5)

        tk.Label(
            left,
            text="LOG TRẠNG THÁI",
            bg=config.COLORS["bg"],
            fg=config.COLORS["text"],
        ).pack(anchor="w", pady=(10, 0))
        log_frame = tk.Frame(left, bg=config.COLORS["bg"])
        log_frame.pack(fill="both", expand=True)
        self.log_box = tk.Listbox(
            log_frame,
            height=8,
            bg=config.COLORS["panel_bg"],
            fg=config.COLORS["text"],
            font=("Consolas", 9),
            highlightthickness=0,
            bd=0,
        )
        scrollbar = ttk.Scrollbar(
            log_frame, orient="vertical", command=self.log_box.yview
        )
        self.log_box.configure(yscrollcommand=scrollbar.set)
        self.log_box.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # ---------------- CỘT PHẢI ----------------
        tk.Label(
            right,
            text="ĐIỀU KHIỂN & KẾT NỐI",
            bg=config.COLORS["bg"],
            fg=config.COLORS["accent"],
            font=("Segoe UI", 11, "bold"),
        ).pack(anchor="w")

        conn = tk.Frame(right, bg=config.COLORS["bg"])
        conn.pack(fill="x", pady=(4, 6))

        tk.Label(
            conn, text="URL stream:", bg=config.COLORS["bg"], fg=config.COLORS["text"]
        ).pack(anchor="w")
        self.url_entry = ttk.Entry(conn)
        self.url_entry.insert(0, config.DEFAULT_STREAM_URL)
        self.url_entry.pack(fill="x", pady=2)

        btn_row = tk.Frame(conn, bg=config.COLORS["bg"])
        btn_row.pack(fill="x", pady=2)
        self.cam_btn = ttk.Button(
            btn_row, text="Bật camera", command=self.toggle_camera
        )
        self.cam_btn.pack(side="left", fill="x", expand=True, padx=(0, 3))
        self.car_btn = ttk.Button(btn_row, text="Kết nối xe", command=self.toggle_car)
        self.car_btn.pack(side="left", fill="x", expand=True, padx=(3, 0))

        self.status_label = tk.Label(
            right,
            text="● Xe: chưa kết nối\n● Camera: tắt",
            bg=config.COLORS["bg"],
            fg=config.COLORS["error"],
            anchor="w",
            justify="left",
        )
        self.status_label.pack(anchor="w", pady=(2, 0))

        ttk.Separator(right, orient="horizontal").pack(fill="x", pady=6)

        tk.Label(
            right,
            text="TỐC ĐỘ (%)",
            bg=config.COLORS["bg"],
            fg=config.COLORS["accent"],
            font=("Segoe UI", 11, "bold"),
        ).pack(anchor="w")
        speed_frame = tk.Frame(right, bg=config.COLORS["bg"])
        speed_frame.pack(fill="x", pady=(4, 6))
        self.speed_scale = tk.Scale(
            speed_frame,
            from_=0,
            to=100,
            orient="horizontal",
            variable=self.current_speed,
            command=self._on_speed_change,
            bg=config.COLORS["bg"],
            fg=config.COLORS["text"],
            troughcolor=config.COLORS["panel_bg"],
            highlightthickness=0,
            activebackground=config.COLORS["accent"],
            bd=0,
        )
        self.speed_scale.pack(fill="x")
        self.speed_display_label = tk.Label(
            right,
            text=f"Tốc độ: {self.current_speed.get()}%",
            bg=config.COLORS["bg"],
            fg=config.COLORS["text"],
            font=("Consolas", 10),
            anchor="w",
            justify="left",
        )
        self.speed_display_label.pack(anchor="w", pady=(2, 4))
        tk.Label(
            right,
            text="Tốc độ dùng cho chế độ tự động",
            bg=config.COLORS["bg"],
            fg="#999999",
            font=("Segoe UI", 8),
        ).pack(anchor="w")

        ttk.Separator(right, orient="horizontal").pack(fill="x", pady=6)

        # Bảng dữ liệu xe
        tk.Label(
            right,
            text="TELEMETRY (DỮ LIỆU XE)",
            bg=config.COLORS["bg"],
            fg=config.COLORS["accent"],
            font=("Segoe UI", 11, "bold"),
        ).pack(anchor="w")
        tele = tk.Frame(right, bg=config.COLORS["bg"])
        tele.pack(fill="x", pady=(4, 6))

        cells = tk.Frame(tele, bg=config.COLORS["bg"])
        cells.pack(anchor="w", pady=(0, 4))
        self.sensor_cells = []
        for _ in range(5):
            cell = tk.Label(
                cells, text=" ", width=3, bg=config.COLORS["panel_bg"], relief="flat"
            )
            cell.pack(side="left", padx=2)
            self.sensor_cells.append(cell)
        self.telemetry_label = tk.Label(
            right,
            text="error = --    PWM L: ----   R: ----",
            bg=config.COLORS["bg"],
            fg=config.COLORS["text"],
            font=("Consolas", 10),
            anchor="w",
            justify="left",
        )
        self.telemetry_label.pack(anchor="w")
        self.dist_label = tk.Label(
            tele,
            text="Khoảng cách: -- cm",
            bg=config.COLORS["bg"],
            fg="#ffcc00",  # Màu vàng nổi bật
            font=("Consolas", 10, "bold"),
            justify="left",
        )
        self.dist_label.pack(anchor="w", pady=(4, 0))
        self.env_label = tk.Label(
            tele,
            text="Nhiệt độ: -- °C   |   Độ ẩm: -- %RH",
            bg=config.COLORS["bg"],
            fg=config.COLORS["text"],
            font=("Consolas", 10),
            justify="left",
        )
        self.env_label.pack(anchor="w", pady=(3, 0))

        # Log moved under video; keep right column slimmer
        ttk.Separator(right, orient="horizontal").pack(fill="x", pady=6)

        # Trạm đo
        tk.Label(
            right,
            text="TRẠM ĐO MÔI TRƯỜNG",
            bg=config.COLORS["bg"],
            fg=config.COLORS["accent"],
            font=("Segoe UI", 11, "bold"),
        ).pack(anchor="w")
        self.checkpoint_labels = {}
        for cp in config.CHECKPOINTS:
            row = tk.Frame(right, bg=config.COLORS["panel_bg"])
            row.pack(fill="x", pady=2)
            tk.Label(
                row,
                text=cp["name"],
                bg=config.COLORS["panel_bg"],
                fg=config.COLORS["text"],
                font=("Segoe UI", 10, "bold"),
                width=8,
                anchor="w",
            ).pack(side="left", padx=6, pady=4)
            temp_label = tk.Label(
                row,
                text="-- °C",
                bg=config.COLORS["panel_bg"],
                fg=config.COLORS["text"],
            )
            temp_label.pack(side="left", padx=4)
            humid_label = tk.Label(
                row,
                text="-- %RH",
                bg=config.COLORS["panel_bg"],
                fg=config.COLORS["text"],
            )
            humid_label.pack(side="left", padx=4)
            self.checkpoint_labels[cp["id"]] = (temp_label, humid_label)

    # Thêm hàm resize động và cập nhật lại _display_frame giống Mode 1
    def _on_video_label_resize(self, event):
        self._video_display_size = (event.width, event.height)
        if hasattr(self, "_video_last_frame") and self._video_last_frame is not None:
            self._display_frame(self._video_last_frame)

    # ==================================================================
    # VÀO / RỜI MODE
    # ==================================================================
    def on_enter(self):
        self.is_active = True
        self._start_link_polling()

    def on_leave(self):
        self.is_active = False
        if self.auto_running:
            self.toggle_auto()  # rời mode mà xe đang chạy -> phanh
        if self._link_after_id is not None:
            self.after_cancel(self._link_after_id)
            self._link_after_id = None
        if self.cam_connected:
            self._stop_camera()

    # ==================================================================
    # CHẾ ĐỘ TỰ ĐỘNG
    # ==================================================================
    def toggle_auto(self):
        if self.auto_running:
            self.auto_running = False
            self.auto_btn.config(text="▶ Bắt đầu tự động")
            communication.link.stop_all()
            self._log("[LỆNH] STOP")
        else:
            if not communication.link.connected:
                self._log("[LỖI] Chưa kết nối xe.")
                return
            self.auto_running = True
            self.auto_btn.config(text="■ Dừng tự động")
            communication.link.set_auto_speed(self.current_speed.get())
            communication.link.start_auto()
            self._log("[LỆNH] START - đặt xe lên vạch đen để nhả khoá ga.")

    # ==================================================================
    # KẾT NỐI XE
    # ==================================================================
    def toggle_car(self):
        if communication.link.connected:
            communication.link.close()
        else:
            communication.link.connect(config.CAR_IP, config.CAR_PORT)

    def _start_link_polling(self):
        if not self.is_active:
            return

        for kind, payload in communication.link.poll_events():
            if kind == "telemetry":
                self._update_telemetry(payload)
            elif kind == "state":
                self._refresh_status()
                if payload == "disconnected" and self.auto_running:
                    self.auto_running = False
                    self.auto_btn.config(text="▶ Bắt đầu tự động")
            else:
                self._log(payload if kind == "info" else f"[LỖI] {payload}")

        self._link_after_id = self.after(
            config.TELEMETRY_POLL_MS, self._start_link_polling
        )

    def _update_telemetry(self, data):
        sensor = data["sensor"]
        for i, cell in enumerate(self.sensor_cells):
            cell.config(
                bg=(
                    config.COLORS["accent"]
                    if (sensor >> i) & 1
                    else config.COLORS["panel_bg"]
                )
            )
        self.telemetry_label.config(
            text=f"  error = {data['error']:+5.2f}   PWM L:{data['pwm_l']:5d} R:{data['pwm_r']:5d}"
        )
        self.speed_display_label.config(text=f"Tốc độ: {self.current_speed.get()}%")
        # THÊM DÒNG CẬP NHẬT KHOẢNG CÁCH
        dist = data.get("dist")

        # Cảnh báo màu đỏ nếu vật cản quá gần (<20cm)
        if dist is not None and 0 < dist < 20:
            self.dist_label.config(
                text=f"Vật cản: {dist:.1f} cm (ĐANG NÉ!)", fg=config.COLORS["error"]
            )
        elif dist is not None:
            self.dist_label.config(text=f"Khoảng cách: {dist:.1f} cm", fg="#ffcc00")
        else:
            self.dist_label.config(text="Khoảng cách: không đo được", fg="#999999")

        temp_c = data.get("temp_c")
        humidity_rh = data.get("humidity_rh")
        if temp_c is None or humidity_rh is None:
            self.env_label.config(text="Nhiệt độ/độ ẩm: chưa có dữ liệu", fg="#999999")
        else:
            self.env_label.config(
                text=f"Nhiệt độ: {temp_c:.1f} °C   |   Độ ẩm: {humidity_rh:.1f} %RH",
                fg=config.COLORS["text"],
            )

    def _on_speed_change(self, value):
        speed = int(float(value))
        self.speed_display_label.config(text=f"Tốc độ: {speed}%")
        if communication.link.connected:
            communication.link.set_auto_speed(speed)

    def _refresh_status(self):
        car_txt = "đã kết nối" if communication.link.connected else "chưa kết nối"
        cam_txt = "đang stream" if self.cam_connected else "tắt"
        color = (
            config.COLORS["ok"]
            if communication.link.connected
            else config.COLORS["error"]
        )
        self.status_label.config(
            text=f"● Xe: {car_txt}   |   Camera: {cam_txt}", fg=color
        )
        self.car_btn.config(
            text="Ngắt kết nối xe" if communication.link.connected else "Kết nối xe"
        )

    # ==================================================================
    # CAMERA (giống hệt Mode 1, dùng chung MJPEGReader)
    # ==================================================================
    def toggle_camera(self):
        if self.cam_connected:
            self._stop_camera()
            return

        url = self.url_entry.get().strip()
        if not url:
            self._log("[LỖI] Vui lòng nhập URL stream.")
            return

        self.mjpeg = MJPEGReader(url, on_error=self._on_stream_error)
        self.mjpeg.start()
        self.cam_connected = True
        self.cam_btn.config(text="Tắt camera")
        self._refresh_status()
        self._log(f"[INFO] Đang mở stream {url}")
        self._start_video_polling()

    def _stop_camera(self):
        if self._video_after_id is not None:
            self.after_cancel(self._video_after_id)
            self._video_after_id = None
        if self.mjpeg:
            self.mjpeg.stop()
            self.mjpeg = None
        self.cam_connected = False
        self.cam_btn.config(text="Bật camera")
        self._refresh_status()
        self._show_placeholder("Camera đã tắt")

    def _start_video_polling(self):
        if not self.cam_connected or self.mjpeg is None:
            return
        frame = self.mjpeg.get_latest_frame()
        if frame is not None:
            self._display_frame(frame)
        self._video_after_id = self.after(
            int(1000 / config.UI_FPS), self._start_video_polling
        )

    def _display_frame(self, bgr_frame):
        raw_frame = self._process_frame(bgr_frame)
        self._video_last_frame = raw_frame
        target_w, target_h = self._video_display_size
        if target_w <= 0 or target_h <= 0:
            return

        frame_h, frame_w = raw_frame.shape[:2]
        scale = min(target_w / frame_w, target_h / frame_h)
        new_w = max(1, int(frame_w * scale))
        new_h = max(1, int(frame_h * scale))

        interpolation = cv2.INTER_AREA if scale < 1 else cv2.INTER_CUBIC
        resized = cv2.resize(raw_frame, (new_w, new_h), interpolation=interpolation)
        rgb_frame = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb_frame)

        if new_w != target_w or new_h != target_h:
            background = Image.new("RGB", (target_w, target_h), "#000000")
            x = (target_w - new_w) // 2
            y = (target_h - new_h) // 2
            background.paste(img, (x, y))
            img = background

        tk_img = ImageTk.PhotoImage(img)
        self.video_label.config(image=tk_img)
        self.video_label.image = tk_img

    def _process_frame(self, bgr_frame):
        # Giữ raw frame để dễ bổ sung AI / CV sau này.
        # Hiện tại không thay đổi, nhưng đây là điểm hook để thêm:
        # - dò line
        # - phát hiện chướng ngại vật
        # - lọc ảnh trước
        return bgr_frame

    def _on_stream_error(self, msg):
        self.after(0, lambda: self._log(f"[LỖI STREAM] {msg}"))
        self.after(0, self._stop_camera)

    def _show_placeholder(self, text):
        target_w, target_h = self._video_display_size
        if target_w <= 0 or target_h <= 0:
            target_w, target_h = config.VIDEO_W * 2, config.VIDEO_H * 2

        img = Image.new("RGB", (target_w, target_h), "#111111")
        draw = ImageDraw.Draw(img)
        text_x = 20
        text_y = target_h // 2 - 10
        draw.text((text_x, text_y), text, fill="#777777")
        tk_img = ImageTk.PhotoImage(img)
        self.video_label.config(image=tk_img)
        self.video_label.image = tk_img

    # ==================================================================
    def _log(self, text):
        self.log_box.insert("end", text)
        self.log_box.see("end")
        if self.log_box.size() > 200:
            self.log_box.delete(0)
