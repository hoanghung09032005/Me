import time
import ctypes
import cv2
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

import config
import communication
from video_stream import MJPEGReader

KEY_MAP = {
    "Up": "FORWARD",
    "w": "FORWARD",
    "W": "FORWARD",
    "Down": "BACKWARD",
    "s": "BACKWARD",
    "S": "BACKWARD",
    "Left": "LEFT",
    "a": "LEFT",
    "A": "LEFT",
    "Right": "RIGHT",
    "d": "RIGHT",
    "D": "RIGHT",
}


class ManualControlFrame(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent, bg=config.COLORS["bg"])

        self.is_active = False
        self.cam_connected = False
        self.mjpeg = None
        self._video_after_id = None
        self._link_after_id = None
        self._border_removed = False

        self.current_speed = tk.IntVar(value=60)
        self._typing = False
        self._held_dir = None
        self._repeat_id = None
        self._dir_buttons = {}

        self._build_layout()
        self.bind_all("<KeyPress>", self._on_key_press)

    # ==================================================================
    # DỰNG GIAO DIỆN (Đã ép gọn lề để Telemetry không bị rớt khỏi màn hình)
    # ==================================================================
    def _build_layout(self):
        left = tk.Frame(self, bg=config.COLORS["bg"], width=340)
        left.pack(side="left", fill="y", padx=5, pady=5)

        right = tk.Frame(self, bg=config.COLORS["bg"])
        right.pack(side="left", fill="both", expand=True, padx=5, pady=5)

        self._build_connection_panel(left)
        self._build_dpad(left)
        self._build_speed(left)
        self._build_telemetry(left)
        self._build_video_panel(right)

    def _watch_entry(self, entry):
        entry.bind("<FocusIn>", lambda e: self._set_typing(True))
        entry.bind("<FocusOut>", lambda e: self._set_typing(False))
        entry.bind("<Return>", lambda e: self._leave_entry())
        entry.bind("<Escape>", lambda e: self._leave_entry())

    def _set_typing(self, value):
        self._typing = value
        self._refresh_kb_status()

    def _leave_entry(self):
        self.focus_set()
        self._set_typing(False)

    def _refresh_kb_status(self):
        if not hasattr(self, "kb_status"):
            return
        if self._typing:
            self.kb_status.config(
                text="Bàn phím: TẮT (đang gõ trong ô nhập)", fg=config.COLORS["warn"]
            )
        else:
            self.kb_status.config(text="Bàn phím: BẬT", fg=config.COLORS["ok"])

    def _build_connection_panel(self, parent):
        tk.Label(
            parent,
            text="MODE 1 - LÁI BẰNG TAY",
            bg=config.COLORS["bg"],
            fg=config.COLORS["accent"],
            font=("Segoe UI", 12, "bold"),
        ).pack(anchor="w")

        box = tk.Frame(parent, bg=config.COLORS["bg"])
        box.pack(fill="x", pady=(2, 2))

        tk.Label(
            box, text="IP ESP32:", bg=config.COLORS["bg"], fg=config.COLORS["text"]
        ).grid(row=0, column=0, sticky="w")
        self.ip_entry = ttk.Entry(box, width=16)
        self.ip_entry.insert(0, config.CAR_IP)
        self.ip_entry.grid(row=0, column=1, sticky="ew", padx=(4, 0))

        self.car_btn = ttk.Button(box, text="Kết nối xe", command=self.toggle_car)
        self.car_btn.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(2, 0))

        self.car_status = tk.Label(
            box,
            text="● Xe: chưa kết nối",
            bg=config.COLORS["bg"],
            fg=config.COLORS["error"],
            anchor="w",
        )
        self.car_status.grid(row=2, column=0, columnspan=2, sticky="w", pady=(0, 0))

        self.link_stats = tk.Label(
            box,
            text="Đã gửi: -   |   STM32 đáp: -",
            bg=config.COLORS["bg"],
            fg="#999999",
            font=("Consolas", 9),
            anchor="w",
        )
        self.link_stats.grid(row=3, column=0, columnspan=2, sticky="w", pady=(0, 4))

        tk.Label(
            box, text="URL stream:", bg=config.COLORS["bg"], fg=config.COLORS["text"]
        ).grid(row=4, column=0, columnspan=2, sticky="w")
        self.url_entry = ttk.Entry(box)
        self.url_entry.insert(0, config.DEFAULT_STREAM_URL)
        self.url_entry.grid(row=5, column=0, columnspan=2, sticky="ew")

        self.cam_btn = ttk.Button(box, text="Bật camera", command=self.toggle_camera)
        self.cam_btn.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(2, 0))

        self.cam_status = tk.Label(
            box,
            text="● Camera: tắt",
            bg=config.COLORS["bg"],
            fg=config.COLORS["error"],
            anchor="w",
        )
        self.cam_status.grid(row=7, column=0, columnspan=2, sticky="w", pady=(0, 0))

        box.columnconfigure(1, weight=1)
        self._watch_entry(self.ip_entry)
        self._watch_entry(self.url_entry)
        ttk.Separator(parent, orient="horizontal").pack(fill="x", pady=4)

    def _build_dpad(self, parent):
        pad = tk.Frame(parent, bg=config.COLORS["bg"])
        pad.pack()

        def make(text, direction, r, c):
            btn = tk.Button(
                pad,
                text=text,
                width=4,
                height=2,
                bg=config.COLORS["btn_bg"],
                fg=config.COLORS["text"],
                activebackground=config.COLORS["btn_active"],
                relief="flat",
                font=("Segoe UI", 12),
                bd=0,
                highlightthickness=0,
            )
            btn.grid(row=r, column=c, padx=3, pady=3)
            # Chạm 1 lần để chạy
            btn.bind("<ButtonPress-1>", lambda e, d=direction: self._press_dir(d))
            self._dir_buttons[direction] = btn
            return btn

        make("▲", "FORWARD", 0, 1)
        make("◀", "LEFT", 1, 0)
        make("▶", "RIGHT", 1, 2)
        make("▼", "BACKWARD", 2, 1)

        # NÚT STOP ĐÃ QUAY TRỞ LẠI
        stop_btn = tk.Button(
            pad,
            text="■",
            width=4,
            height=2,
            bg="#5a2222",
            fg="#ffcccc",
            activebackground=config.COLORS["error"],
            relief="flat",
            font=("Segoe UI", 12),
            bd=0,
            highlightthickness=0,
            command=self._stop_drive,
        )
        stop_btn.grid(row=1, column=1, padx=3, pady=3)

        tk.Label(
            parent,
            text="BẤM 1 LẦN để chuyển hướng\n(Nhấn nút ■ hoặc Space để PHANH)",
            bg=config.COLORS["bg"],
            fg="#999999",
            font=("Segoe UI", 8),
            justify="center",
        ).pack(pady=(2, 0))

        self.kb_status = tk.Label(
            parent,
            text="Bàn phím: BẬT",
            bg=config.COLORS["bg"],
            fg=config.COLORS["ok"],
            font=("Segoe UI", 8, "bold"),
        )
        self.kb_status.pack(pady=(0, 4))

    def _build_speed(self, parent):
        frame = tk.Frame(parent, bg=config.COLORS["bg"])
        frame.pack(fill="x", pady=(0, 2))

        tk.Label(
            frame, text="Tốc độ (%)", bg=config.COLORS["bg"], fg=config.COLORS["text"]
        ).pack(anchor="w")
        tk.Scale(
            frame,
            from_=0,
            to=100,
            orient="horizontal",
            variable=self.current_speed,
            bg=config.COLORS["bg"],
            fg=config.COLORS["text"],
            troughcolor=config.COLORS["panel_bg"],
            highlightthickness=0,
            activebackground=config.COLORS["accent"],
            bd=0,
        ).pack(fill="x")

        tk.Label(
            frame,
            text="Dưới ~30% motor có thể không đủ lực quay",
            bg=config.COLORS["bg"],
            fg="#999999",
            font=("Segoe UI", 8),
        ).pack(anchor="w")
        ttk.Separator(parent, orient="horizontal").pack(fill="x", pady=4)

    def _build_telemetry(self, parent):
        tk.Label(
            parent,
            text="XE GỬI VỀ",
            bg=config.COLORS["bg"],
            fg=config.COLORS["accent"],
            font=("Segoe UI", 10, "bold"),
        ).pack(anchor="w")

        sensor_row = tk.Frame(parent, bg=config.COLORS["bg"])
        sensor_row.pack(anchor="w", pady=2)
        self.sensor_cells = []
        for i in range(5):
            cell = tk.Label(
                sensor_row,
                text=" ",
                width=3,
                height=1,
                bg=config.COLORS["panel_bg"],
                relief="flat",
            )
            cell.pack(side="left", padx=2)
            self.sensor_cells.append(cell)

        self.telemetry_label = tk.Label(
            parent,
            text="error = --    PWM  L: ----   R: ----",
            bg=config.COLORS["bg"],
            fg=config.COLORS["text"],
            font=("Consolas", 10),
            anchor="w",
            justify="left",
        )
        self.telemetry_label.pack(anchor="w")

    def _build_video_panel(self, parent):
        tk.Label(
            parent,
            text="CAMERA HÀNH TRÌNH",
            bg=config.COLORS["bg"],
            fg=config.COLORS["accent"],
            font=("Segoe UI", 12, "bold"),
        ).pack(anchor="w")

        # Ô đen giữ chỗ cố định
        new_w = config.VIDEO_W * 2
        new_h = config.VIDEO_H * 2
        self.placeholder_img = tk.PhotoImage(width=new_w, height=new_h)
        self.video_label = tk.Label(parent, image=self.placeholder_img, bg="black")
        self.video_label.pack(pady=(4, 6))
        self.video_label.config(width=new_w, height=new_h)
        self.video_label.bind("<Configure>", self._on_video_label_resize)
        self._video_display_size = (new_w, new_h)
        self._video_last_frame = None

        tk.Label(
            parent, text="Log:", bg=config.COLORS["bg"], fg=config.COLORS["text"]
        ).pack(anchor="w")
        log_frame = tk.Frame(parent, bg=config.COLORS["bg"])
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

    # ==================================================================
    # VÀO / RỜI MODE
    # ==================================================================
    def on_enter(self):
        self.is_active = True
        if communication.link.connected:
            communication.link.enter_manual()
        self._refresh_car_status()
        self._start_link_polling()

    def on_leave(self):
        self.is_active = False
        self._stop_drive()
        if communication.link.connected:
            communication.link.brake()
        if self._link_after_id is not None:
            self.after_cancel(self._link_after_id)
            self._link_after_id = None
        if self.cam_connected:
            self._stop_camera()

    # ==================================================================
    # LÁI XE (CƠ CHẾ CHẠM 1 LẦN)
    # ==================================================================
    def _press_dir(self, direction):
        if self._typing:
            self._leave_entry()
        self._start_drive(direction)

    def _start_drive(self, direction):
        # Nếu đang chạy đúng hướng này thì bỏ qua (Không spam Log)
        if self._held_dir == direction:
            return

        if self._held_dir is not None:
            self._clear_highlight(self._held_dir)

        self._held_dir = direction
        self._highlight(direction)

        if self._repeat_id is not None:
            self.after_cancel(self._repeat_id)
            self._repeat_id = None

        self._send_held(first=True)

    def _send_held(self, first=False):
        if self._held_dir is None:
            return
        if not communication.link.connected:
            if first:
                self._log("[LỖI] Chưa kết nối xe - bấm 'Kết nối xe' trước.")
            self._stop_drive()
            return

        speed = self.current_speed.get()
        communication.link.drive(self._held_dir, speed)

        # Chỉ in Log đúng 1 lần khi chuyển lệnh
        if first:
            code = communication.DIR_CODE.get(self._held_dir, "S")
            name = {
                "FORWARD": "TIẾN",
                "BACKWARD": "LÙI",
                "LEFT": "XOAY TRÁI",
                "RIGHT": "XOAY PHẢI",
            }.get(self._held_dir, "?")
            self._log(f'{self._now()} ▶ LỆNH MỚI: {name:<10} → gửi "M,{code},{speed}"')

        self._repeat_id = self.after(config.MANUAL_REPEAT_MS, self._send_held)

    def _stop_drive(self):
        if self._repeat_id is not None:
            self.after_cancel(self._repeat_id)
            self._repeat_id = None

        # Nếu đã phanh rồi thì không làm gì cả (Chống spam Log khi bấm Stop nhiều lần)
        if self._held_dir is None:
            return

        self._clear_highlight(self._held_dir)
        self._held_dir = None

        communication.link.brake()
        self._log(f'{self._now()} ■ PHANH LẠI            → gửi "M,S,0" (phanh)')

    def _highlight(self, direction):
        btn = self._dir_buttons.get(direction)
        if btn:
            btn.config(bg=config.COLORS["btn_active"], fg="#000000")

    def _clear_highlight(self, direction):
        btn = self._dir_buttons.get(direction)
        if btn:
            btn.config(bg=config.COLORS["btn_bg"], fg=config.COLORS["text"])

    def _keyboard_blocked(self):
        if not self.is_active:
            return True
        if self._typing:
            self._log("[CHÚ Ý] Con trỏ đang trong ô nhập nên phím lái bị bỏ qua.")
            return True
        return False

    def _on_key_press(self, event):
        if self._keyboard_blocked():
            return
        if event.keysym == "space":
            self._stop_drive()
            return

        direction = KEY_MAP.get(event.keysym)
        if direction is None:
            return

        self._start_drive(direction)

    # ==================================================================
    # KẾT NỐI XE & TELEMETRY
    # ==================================================================
    def toggle_car(self):
        if communication.link.connected:
            self._stop_drive()
            communication.link.close()
            return

        ip = self.ip_entry.get().strip() or config.CAR_IP
        self.car_status.config(text="● Xe: đang kết nối...", fg=config.COLORS["warn"])
        communication.link.connect_async(ip, config.CAR_PORT)

    def _on_link_state(self, state):
        self._refresh_car_status()
        if state == "connected":
            self._log(f"{self._now()} ✓ Đã nối. Đang chờ xe trả lời ACK...")
            communication.link.enter_manual()
        else:
            self._stop_drive()

    def _refresh_car_status(self):
        if communication.link.connected:
            self.car_btn.config(text="Ngắt kết nối xe")
            self.car_status.config(text="● Xe: đã kết nối", fg=config.COLORS["ok"])
        else:
            self.car_btn.config(text="Kết nối xe")
            self.car_status.config(text="● Xe: chưa kết nối", fg=config.COLORS["error"])

    def _start_link_polling(self):
        if not self.is_active:
            return
        try:
            for kind, payload in communication.link.poll_events():
                if kind == "telemetry":
                    self._update_telemetry(payload)
                elif kind == "state":
                    self._on_link_state(payload)
                else:
                    self._log(payload if kind == "info" else f"[LỖI] {payload}")
            self._refresh_link_stats()
        except Exception as e:
            self._log(f"[LỖI GIAO DIỆN] {e}")
        finally:
            if self.is_active:
                self._link_after_id = self.after(
                    config.TELEMETRY_POLL_MS, self._start_link_polling
                )

    def _now(self):
        return time.strftime("%H:%M:%S")

    def _refresh_link_stats(self):
        link = communication.link
        sent, recv = link.sent_count, link.recv_count

        if not link.connected:
            self.link_stats.config(text="Đã gửi: -   |   STM32 đáp: -", fg="#999999")
            return

        color = config.COLORS["ok"] if recv > 0 else config.COLORS["error"]
        self.link_stats.config(text=f"Đã gửi: {sent}   |   STM32 đáp: {recv}", fg=color)

    def _update_telemetry(self, data):
        sensor = data["sensor"]
        for i, cell in enumerate(self.sensor_cells):
            on_line = (sensor >> i) & 1
            cell.config(
                bg=config.COLORS["accent"] if on_line else config.COLORS["panel_bg"]
            )

        # Đoạn này sẽ cập nhật liên tục dòng chữ error và PWM
        self.telemetry_label.config(
            text=f"error = {data['error']:+5.2f}    PWM  L:{data['pwm_l']:5d}   R:{data['pwm_r']:5d}"
        )

    # ==================================================================
    # CAMERA VÀ OPENCV XÓA VIỀN TUYỆT ĐỐI
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
        self.cam_status.config(
            text="● Camera: đang kết nối...", fg=config.COLORS["warn"]
        )
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
        self.cam_status.config(text="● Camera: tắt", fg=config.COLORS["error"])

        cv2.destroyAllWindows()
        self._border_removed = False

    def _start_video_polling(self):
        if not self.cam_connected or self.mjpeg is None:
            return

        frame = self.mjpeg.get_latest_frame()
        if frame is not None:
            self._display_frame(frame)
            self.cam_status.config(text="● Camera: đang stream", fg=config.COLORS["ok"])

        self._video_after_id = self.after(33, self._start_video_polling)

    def _display_frame(self, bgr_frame):
        self._video_last_frame = bgr_frame
        target_w, target_h = self._video_display_size
        if target_w <= 0 or target_h <= 0:
            return

        resized = cv2.resize(bgr_frame, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
        rgb_frame = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb_frame)
        tk_img = ImageTk.PhotoImage(img)

        self.video_label.config(image=tk_img)
        self.video_label.image = tk_img

    def _on_video_label_resize(self, event):
        self._video_display_size = (event.width, event.height)
        if self._video_last_frame is not None:
            self._display_frame(self._video_last_frame)

    def _on_stream_error(self, msg):
        self.after(0, lambda: self._log(f"[LỖI STREAM] {msg}"))
        self.after(0, self._stop_camera)

    def _log(self, text):
        self.log_box.insert("end", text)
        self.log_box.see("end")
        if self.log_box.size() > 200:
            self.log_box.delete(0)
