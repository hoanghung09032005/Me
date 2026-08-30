"""
mode3_follow.py
---------------
MODE 3: Bám vật thể bằng cảm biến siêu âm:
  • "Siêu âm"  : STM32 tự quét servo + HC-SR04 (gửi lệnh 'O')
"""

import cv2
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageDraw, ImageTk

import config
import communication
import image_enhance
from video_stream import MJPEGReader


class FollowObjectFrame(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent, bg=config.COLORS["bg"])

        self.is_active = False
        self.cam_connected = False
        self.follow_running = False
        self.mjpeg = None
        self._video_after_id = None
        self._link_after_id = None

        self._build_layout()
        self._show_placeholder("Chưa có tín hiệu video")

    def _build_layout(self):
        left = tk.Frame(self, bg=config.COLORS["bg"], width=340)
        left.pack(side="left", fill="y", padx=5, pady=5)

        right = tk.Frame(self, bg=config.COLORS["bg"])
        right.pack(side="left", fill="both", expand=True, padx=5, pady=5)

        self._build_connection_panel(left)
        self._build_follow_controls(left)
        self._build_distance_display(left)
        self._build_video_panel(right)

    def _build_connection_panel(self, parent):
        tk.Label(
            parent,
            text="MODE 3 - BÁM VẬT THỂ (HC-SR04)",
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
        ttk.Separator(parent, orient="horizontal").pack(fill="x", pady=4)

    def _build_follow_controls(self, parent):
        tk.Label(
            parent,
            text="CHẾ ĐỘ BÁM",
            bg=config.COLORS["bg"],
            fg=config.COLORS["accent"],
            font=("Segoe UI", 10, "bold"),
        ).pack(anchor="w")

        tk.Label(
            parent,
            text="STM32 tự quét servo và đo HC-SR04",
            bg=config.COLORS["bg"],
            fg=config.COLORS["text"],
            font=("Segoe UI", 9, "bold"),
        ).pack(anchor="w", pady=(3, 0))

        self.follow_btn = ttk.Button(
            parent, text="▶ BẮT ĐẦU BÁM", command=self.toggle_follow
        )
        self.follow_btn.pack(fill="x", ipady=5, pady=(4, 0))

        tk.Label(
            parent,
            text="Đặt vật thể trước xe, sau đó bấm bắt đầu.\n"
            "Camera chỉ dùng để quan sát, không điều khiển xe.",
            bg=config.COLORS["bg"],
            fg="#999999",
            font=("Segoe UI", 8),
            justify="left",
        ).pack(anchor="w", pady=(2, 0))

        self.follow_info = tk.Label(
            parent,
            text="Trạng thái: đứng yên",
            bg=config.COLORS["bg"],
            fg="#999999",
            font=("Consolas", 9),
            anchor="w",
        )
        self.follow_info.pack(anchor="w", pady=(4, 0))

        ttk.Separator(parent, orient="horizontal").pack(fill="x", pady=4)

    def _build_distance_display(self, parent):
        """Chỉ hiển thị khoảng cách — dùng cho chế độ siêu âm."""
        tk.Label(
            parent,
            text="KHOẢNG CÁCH",
            bg=config.COLORS["bg"],
            fg=config.COLORS["accent"],
            font=("Segoe UI", 10, "bold"),
        ).pack(anchor="w")

        self.dist_label = tk.Label(
            parent,
            text="-- cm",
            bg=config.COLORS["bg"],
            fg="#ffcc00",
            font=("Consolas", 14, "bold"),
            justify="left",
        )
        self.dist_label.pack(anchor="w", pady=(4, 0))

        ttk.Separator(parent, orient="horizontal").pack(fill="x", pady=4)

    def _build_video_panel(self, parent):
        tk.Label(
            parent,
            text="CAMERA GIÁM SÁT (TÙY CHỌN)",
            bg=config.COLORS["bg"],
            fg=config.COLORS["accent"],
            font=("Segoe UI", 12, "bold"),
        ).pack(anchor="w")

        new_w = config.VIDEO_DISPLAY_W
        new_h = config.VIDEO_DISPLAY_H
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

    def on_enter(self):
        self.is_active = True
        self._refresh_status()
        self._start_link_polling()

    def on_leave(self):
        self.is_active = False
        if self.follow_running:
            self.toggle_follow()
        if self._link_after_id is not None:
            self.after_cancel(self._link_after_id)
            self._link_after_id = None
        if self.cam_connected:
            self._stop_camera()

    def toggle_follow(self):
        if self.follow_running:
            self.follow_running = False
            self.follow_btn.config(text="▶ BẮT ĐẦU BÁM")
            self.follow_info.config(text="Trạng thái: đứng yên", fg="#999999")
            communication.link.stop_all()
            self._log("[LỆNH] STOP")
        else:
            if not communication.link.connected:
                self._log("[LỖI] Chưa kết nối xe.")
                return
            self.follow_running = True
            self.follow_btn.config(text="■ DỪNG BÁM")
            self.follow_info.config(
                text="Trạng thái: quét siêu âm...", fg=config.COLORS["ok"]
            )
            communication.link.start_follow()
            self._log("[LỆNH] START FOLLOW - STM32 tự quét servo + HC-SR04.")

    def toggle_car(self):
        if communication.link.connected:
            communication.link.close()
            return
        ip = self.ip_entry.get().strip() or config.CAR_IP
        self.car_status.config(text="● Xe: đang kết nối...", fg=config.COLORS["warn"])
        communication.link.connect_async(ip, config.CAR_PORT)

    def _start_link_polling(self):
        if not self.is_active:
            return
        if self.follow_running and communication.link.connected:
            communication.link.heartbeat()
        try:
            for kind, payload in communication.link.poll_events():
                if kind == "telemetry":
                    self._update_distance(payload)
                elif kind == "state":
                    self._refresh_status()
                    if payload == "disconnected" and self.follow_running:
                        self.follow_running = False
                        self.follow_btn.config(text="▶ BẮT ĐẦU BÁM")
                        self.follow_info.config(
                            text="Trạng thái: mất kết nối", fg=config.COLORS["error"]
                        )
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

    def _refresh_link_stats(self):
        link = communication.link
        sent, recv = link.sent_count, link.recv_count
        if not link.connected:
            self.link_stats.config(text="Đã gửi: -   |   STM32 đáp: -", fg="#999999")
            return
        color = config.COLORS["ok"] if recv > 0 else config.COLORS["error"]
        self.link_stats.config(text=f"Đã gửi: {sent}   |   STM32 đáp: {recv}", fg=color)

    def _update_distance(self, data):
        """Chỉ cập nhật khoảng cách — bỏ qua error, PWM, nhiệt độ, độ ẩm."""
        dist = data.get("dist")
        if dist is not None and 0 < dist < 20:
            self.dist_label.config(
                text=f"{dist:.1f} cm (GẦN!)", fg=config.COLORS["error"]
            )
        elif dist is not None:
            self.dist_label.config(text=f"{dist:.1f} cm", fg="#ffcc00")
        else:
            self.dist_label.config(text="-- cm", fg="#999999")

    def _refresh_status(self):
        car_txt = "đã kết nối" if communication.link.connected else "chưa kết nối"
        color = (
            config.COLORS["ok"]
            if communication.link.connected
            else config.COLORS["error"]
        )
        self.car_status.config(text=f"● Xe: {car_txt}", fg=color)
        self.car_btn.config(
            text="Ngắt kết nối xe" if communication.link.connected else "Kết nối xe"
        )

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
        self._show_placeholder("Camera đã tắt")

    def _start_video_polling(self):
        if not self.cam_connected or self.mjpeg is None:
            return
        frame = self.mjpeg.get_latest_frame()
        if frame is not None:
            self._render_frame(frame)
            self.cam_status.config(text="● Camera: đang stream", fg=config.COLORS["ok"])
        self._video_after_id = self.after(
            int(1000 / config.UI_FPS), self._start_video_polling
        )

    def _render_frame(self, bgr_frame):
        self._video_last_frame = bgr_frame
        display_frame = image_enhance.enhance_frame(bgr_frame)

        target_w, target_h = self._video_display_size
        if target_w <= 0 or target_h <= 0:
            return

        frame_h, frame_w = display_frame.shape[:2]
        scale = min(target_w / frame_w, target_h / frame_h)
        new_w = max(1, int(frame_w * scale))
        new_h = max(1, int(frame_h * scale))

        interpolation = cv2.INTER_AREA if scale < 1 else cv2.INTER_CUBIC
        resized = cv2.resize(display_frame, (new_w, new_h), interpolation=interpolation)
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

    def _on_video_label_resize(self, event):
        self._video_display_size = (event.width, event.height)
        if self._video_last_frame is not None:
            self._render_frame(self._video_last_frame)

    def _on_stream_error(self, msg):
        self.after(0, lambda: self._log(f"[LỖI STREAM] {msg}"))
        self.after(0, self._stop_camera)

    def _show_placeholder(self, text):
        target_w, target_h = self._video_display_size
        if target_w <= 0 or target_h <= 0:
            target_w, target_h = config.VIDEO_DISPLAY_W, config.VIDEO_DISPLAY_H

        img = Image.new("RGB", (target_w, target_h), "#111111")
        draw = ImageDraw.Draw(img)
        text_x = 20
        text_y = target_h // 2 - 10
        draw.text((text_x, text_y), text, fill="#777777")
        tk_img = ImageTk.PhotoImage(img)
        self.video_label.config(image=tk_img)
        self.video_label.image = tk_img

    def _log(self, text):
        self.log_box.insert("end", text)
        self.log_box.see("end")
        if self.log_box.size() > 200:
            self.log_box.delete(0)
