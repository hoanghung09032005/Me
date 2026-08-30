"""
mode4_ai.py
-----------
MODE 4: AI nhận diện line qua video.
Chỉ hiển thị video + log text. Không cần telemetry vì xe chỉ bám line.
"""

import time
import cv2
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageDraw, ImageTk
import numpy as np

import config
import communication
import image_enhance
from ai_model import LineErrorNet, frame_to_tensor, TORCH_AVAILABLE
import os
import time as time_module  # tránh trùng tên với module time đã import

if TORCH_AVAILABLE:
    import torch
from video_stream import MJPEGReader


class AILineFrame(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent, bg=config.COLORS["bg"])

        self.is_active = False
        self.cam_connected = False
        self.ai_running = False
        self.mjpeg = None
        self._video_after_id = None
        self._link_after_id = None

        self.record_enabled = False
        self._record_counter = 0
        self._frame_counter = 0
        os.makedirs(config.AI_DATASET_DIR, exist_ok=True)

        self.use_ai_model = tk.BooleanVar(value=False)
        self.ai_model = None
        self._ai_model_load_attempted = False

        self._build_layout()
        self._show_placeholder("Chưa có tín hiệu video")

    def _build_layout(self):
        left = tk.Frame(self, bg=config.COLORS["bg"], width=340)
        left.pack(side="left", fill="y", padx=5, pady=5)

        right = tk.Frame(self, bg=config.COLORS["bg"])
        right.pack(side="left", fill="both", expand=True, padx=5, pady=5)

        self._build_connection_panel(left)
        self._build_ai_controls(left)
        self._build_video_panel(right)

    def _build_connection_panel(self, parent):
        tk.Label(
            parent,
            text="MODE 4 - AI NHẬN DIỆN LINE",
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

    def _build_ai_controls(self, parent):
        self.ai_btn = ttk.Button(
            parent, text="▶ BẮT ĐẦU AI LINE", command=self.toggle_ai
        )
        self.ai_btn.pack(fill="x", ipady=5)

        tk.Label(
            parent,
            text="Đặt xe trên line đen rồi bấm nút.\n"
                 "PC sẽ nhận diện line và điều khiển xe.",
            bg=config.COLORS["bg"],
            fg="#999999",
            font=("Segoe UI", 8),
            justify="left",
        ).pack(anchor="w", pady=(2, 0))

        self.ai_info = tk.Label(
            parent,
            text="Line: chưa phát hiện",
            bg=config.COLORS["bg"],
            fg="#999999",
            font=("Consolas", 9),
            anchor="w",
        )
        self.ai_info.pack(anchor="w", pady=(4, 0))

        ttk.Separator(parent, orient="horizontal").pack(fill="x", pady=4)

        self.record_btn = ttk.Button(
            parent, text="● Ghi dữ liệu (auto-label)", command=self.toggle_record
        )
        self.record_btn.pack(fill="x", pady=(2, 0))

        self.record_status = tk.Label(
            parent, text="Đã lưu: 0 ảnh", bg=config.COLORS["bg"],
            fg="#999999", font=("Consolas", 9), anchor="w",
        )
        self.record_status.pack(anchor="w")

        tk.Label(
            parent,
            text="Bật khi xe đang bám line TỐT (dùng CV cổ điển)\n"
            "để tự động lưu ảnh + error làm dữ liệu train.",
            bg=config.COLORS["bg"], fg="#999999", font=("Segoe UI", 8), justify="left",
        ).pack(anchor="w")

        ttk.Separator(parent, orient="horizontal").pack(fill="x", pady=4)

        self.ai_model_check = ttk.Checkbutton(
            parent, text="Dùng AI Model thay vì threshold cổ điển",
                variable=self.use_ai_model, command=self._on_toggle_ai_model
        )
        self.ai_model_check.pack(anchor="w", pady=(2, 0))

        ttk.Separator(parent, orient="horizontal").pack(fill="x", pady=4)

    def _build_video_panel(self, parent):
        tk.Label(
            parent,
            text="CAMERA HÀNH TRÌNH",
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
        if self.ai_running:
            self.toggle_ai()
        if self.record_enabled:
            self.toggle_record()
        if self._link_after_id is not None:
            self.after_cancel(self._link_after_id)
            self._link_after_id = None
        if self.cam_connected:
            self._stop_camera()

    def toggle_ai(self):
        if self.ai_running:
            self.ai_running = False
            self.ai_btn.config(text="▶ BẮT ĐẦU AI LINE")
            self.ai_info.config(text="Line: chưa phát hiện", fg="#999999")
            communication.link.stop_all()
            self._log("[LỆNH] STOP")
        else:
            if not communication.link.connected:
                self._log("[LỖI] Chưa kết nối xe.")
                return
            self.ai_running = True
            self.ai_btn.config(text="■ DỪNG AI LINE")

            # SỬA LỖI: trước đây không kiểm tra camera trước khi bật AI
            # Line - nếu quên bật camera, không frame nào được xử lý nên
            # send_ai_error() không bao giờ được gọi. Lệnh "I 0" gửi lúc
            # bấm START chỉ để chuyển car_mode sang MODE_AI_LINE, KHÔNG
            # tương đương với có line - nhưng vì test cũ dùng đúng giá trị
            # 0 làm cả 2 nghĩa ("line giữa tâm" VÀ trong bản gốc từng là
            # "mất line"), xe sẽ đứng im chờ mãi mà GUI vẫn hiển thị như
            # đang chạy AI Line bình thường.
            if not self.cam_connected:
                self.ai_running = False
                self.ai_btn.config(text="▶ BẮT ĐẦU AI LINE")
                self._log("[LỖI] AI Line cần bật camera trước. Bấm 'Bật camera' rồi thử lại.")
                return

            communication.link.start_ai_line()
            self._log("[LỆNH] START AI LINE - đặt xe trên line đen.")
    def toggle_record(self):
        self.record_enabled = not self.record_enabled
        if self.record_enabled:
            self.record_btn.config(text="■ Dừng ghi dữ liệu")
            self._log("[GHI DỮ LIỆU] Bắt đầu - chỉ lưu khi CV cổ điển phát hiện line.")
        else:
            self.record_btn.config(text="● Ghi dữ liệu (auto-label)")
            self._log(f"[GHI DỮ LIỆU] Dừng - tổng {self._record_counter} ảnh.")

    def _save_sample(self, bgr_frame, error_value):
        self._frame_counter += 1
        # Giãn cách để tránh lưu quá nhiều ảnh gần giống hệt nhau
        if self._frame_counter % config.AI_RECORD_EVERY_N_FRAMES != 0:
            return
        filename = f"img_{self._record_counter:06d}.jpg"
        filepath = os.path.join(config.AI_DATASET_DIR, filename)
        cv2.imwrite(filepath, bgr_frame)

        label_path = os.path.join(config.AI_DATASET_DIR, "labels.csv")
        write_header = not os.path.exists(label_path)
        with open(label_path, "a", encoding="utf-8") as f:
            if write_header:
                f.write("file,error\n")
            f.write(f"{filename},{error_value:.4f}\n")

        self._record_counter += 1
        self.after(0, lambda: self.record_status.config(
            text=f"Đã lưu: {self._record_counter} ảnh"
        ))

    def _on_toggle_ai_model(self):
        if not self.use_ai_model.get():
            return
        if not TORCH_AVAILABLE:
            self._log("[LỖI] Chưa cài PyTorch (pip install torch --break-system-packages).")
            self.use_ai_model.set(False)
            return
        if self.ai_model is None:
            try:
                self.ai_model = LineErrorNet()
                self.ai_model.load_state_dict(
                    torch.load(config.AI_MODEL_PATH, map_location="cpu")
                )
                self.ai_model.eval()
                self._log(f"[AI MODEL] Đã load {config.AI_MODEL_PATH}")
            except Exception as e:
                self._log(f"[LỖI] Không load được model: {e}")
                self.use_ai_model.set(False)
                self.ai_model = None
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
        if self.ai_running and communication.link.connected:
            communication.link.heartbeat()
        try:
            for kind, payload in communication.link.poll_events():
                if kind == "state":
                    self._refresh_status()
                    if payload == "disconnected" and self.ai_running:
                        self.ai_running = False
                        self.ai_btn.config(text="▶ BẮT ĐẦU AI LINE")
                        self.ai_info.config(text="Mất kết nối", fg=config.COLORS["error"])
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

    def _refresh_status(self):
        car_txt = "đã kết nối" if communication.link.connected else "chưa kết nối"
        color = config.COLORS["ok"] if communication.link.connected else config.COLORS["error"]
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
        self.cam_status.config(text="● Camera: đang kết nối...", fg=config.COLORS["warn"])
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
            # _process_frame() có TÁC DỤNG PHỤ (gửi lệnh UART xuống STM32,
            # có thể lưu ảnh vào dataset train) - CHỈ được gọi đúng 1 lần
            # cho mỗi frame THẬT từ camera, ở đây. _render_frame() bên dưới
            # (dùng khi resize cửa sổ) KHÔNG được gọi lại hàm này - xem giải
            # thích chi tiết trong _render_frame().
            overlay = self._process_frame(frame)
            self._render_frame(overlay)
            self.cam_status.config(text="● Camera: đang stream", fg=config.COLORS["ok"])
        self._video_after_id = self.after(
            int(1000 / config.UI_FPS), self._start_video_polling
        )

    def _render_frame(self, bgr_frame):
        # SỬA LỖI (quan trọng): trước đây hàm hiển thị (từng tên
        # _display_frame) tự gọi lại _process_frame() bên trong, và
        # _on_video_label_resize() dùng LẠI chính hàm đó để vẽ lại lúc
        # resize cửa sổ. Hệ quả: mỗi lần user kéo giãn/thu nhỏ cửa sổ,
        # _process_frame() chạy lại trên self._video_last_frame - nhưng
        # biến đó thực ra là ẢNH ĐÃ VẼ OVERLAY (contour xanh, chấm đỏ,
        # crosshair) từ lần xử lý TRƯỚC, không phải ảnh gốc từ camera. Kết
        # quả:
        #   1) communication.link.send_ai_error() bị gọi lại một cách
        #      KHÔNG CHỦ Ý, gửi thêm 1 lệnh xuống STM32 chỉ vì user resize
        #      cửa sổ - không liên quan gì tới video thật.
        #   2) Nếu đang bật "Ghi dữ liệu" (record_enabled), có thể LƯU
        #      THÊM 1 ẢNH ĐÃ BỊ NHIỄM OVERLAY vào dataset train - âm thầm
        #      làm bẩn dữ liệu huấn luyện AI mà người dùng không hề biết,
        #      chỉ vì đã resize cửa sổ trong lúc đang ghi.
        # Giải pháp: TÁCH RIÊNG xử lý (có tác dụng phụ, chỉ chạy đúng 1 lần
        # mỗi frame thật - xem _start_video_polling) khỏi hiển thị (hàm
        # này - CHỈ lo resize/letterbox/enhance cho mục đích hiển thị, an
        # toàn để gọi lại bao nhiêu lần cũng được, kể cả khi resize cửa sổ
        # liên tục, không có tác dụng phụ nào).
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

    def _process_frame(self, bgr_frame):
        h, w = bgr_frame.shape[:2]
        crop_y = int(h * config.AI_LINE_CROP_TOP)
        roi = bgr_frame[crop_y:h, 0:w]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (config.AI_LINE_BLUR_KSIZE, config.AI_LINE_BLUR_KSIZE), 0)
        _, thresh = cv2.threshold(blur, config.AI_LINE_THRESHOLD, 255, cv2.THRESH_BINARY_INV)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        overlay = bgr_frame.copy()
        cv2.line(overlay, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)

        error = 0
        detected = False

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area > 100:
                detected = True
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"]) + crop_y

                    cv2.drawContours(overlay, [largest], -1, (0, 255, 0), 2)
                    cv2.circle(overlay, (cx, cy), 5, (0, 0, 255), -1)

                    deviation = (cx - w / 2) / (w / 2)
                    error = deviation * 4.0

        # ---- GHI DỮ LIỆU: dùng error từ CV cổ điển làm nhãn tự động ----
        # Luôn ghi bằng error CỔ ĐIỂN (không phải error AI), kể cả khi đang
        # bật chế độ AI model - vì đây là nhãn "đúng" để làm dữ liệu train.
        if self.record_enabled and detected and self.ai_running:
            self._save_sample(bgr_frame, error)

        # ---- THAY ERROR BẰNG AI MODEL nếu được bật (chỉ khi có line) ----
        # Giữ `detected` từ CV cổ điển làm tín hiệu mất-line: threshold vẫn
        # đáng tin hơn để trả lời "có tồn tại 1 vùng tối giống line không",
        # AI model chỉ đảm nhiệm tính error CHÍNH XÁC hơn khi có line.
        ai_active = self.use_ai_model.get() and self.ai_model is not None
        if ai_active and detected:
            try:
                with torch.no_grad():
                    tensor = frame_to_tensor(bgr_frame, config.AI_LINE_CROP_TOP)
                    error = float(self.ai_model(tensor).item())
            except Exception as e:
                self._log(f"[LỖI AI MODEL] {e} - dùng tạm CV cổ điển.")

        if self.ai_running:
            if detected:
                error_x100 = int(error * 100)
                error_x100 = max(-400, min(400, error_x100))
            else:
                error_x100 = config.AI_LINE_LOST_SENTINEL

            communication.link.send_ai_error(error_x100)

            source_tag = "AI" if ai_active and detected else "CV"
            status = f"[{source_tag}] error={error:+.2f} {'[OK]' if detected else '[MẤT]'}"
            self.after(0, lambda s=status: self.ai_info.config(
                text=s, fg=config.COLORS["ok"] if detected else config.COLORS["warn"]
            ))

        return overlay

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