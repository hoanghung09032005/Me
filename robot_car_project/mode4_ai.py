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

        # SỬA LỖI QUAN TRỌNG (mất dữ liệu): trước đây _record_counter luôn
        # bắt đầu từ 0 mỗi lần mở lại chương trình (mỗi lần khởi tạo
        # AILineFrame). Vì _save_sample() dùng đúng "img_{counter:06d}.jpg"
        # làm tên file và ghi bằng cv2.imwrite() (GHI ĐÈ nếu trùng tên),
        # còn labels.csv lại mở bằng mode "a" (chỉ nối thêm dòng, không xoá
        # dòng cũ) - mỗi phiên ghi dữ liệu MỚI sẽ ĐÈ LÊN ảnh cũ cùng tên
        # trên đĩa, nhưng dòng nhãn CŨ trong labels.csv vẫn còn nguyên, giờ
        # trỏ sai vào nội dung ảnh mới. Hậu quả thực tế: sau vài phiên ghi
        # dữ liệu, hàng chục ảnh bị nhãn sai lệch hoàn toàn, làm nhiễu cả
        # dataset (phát hiện được qua 1 cụm điểm bất thường trong biểu đồ
        # đánh giá model).
        # Giờ quét sẵn các file "img_XXXXXX.jpg" đã có trong thư mục dataset
        # lúc khởi tạo, lấy số lớn nhất + 1 làm điểm bắt đầu - đảm bảo MỌI
        # phiên ghi dữ liệu sau này luôn nối tiếp, không bao giờ ghi đè lên
        # ảnh của phiên trước, dù có tắt/mở lại chương trình bao nhiêu lần.
        self._record_counter = self._find_next_record_index()

        self.use_ai_model = tk.BooleanVar(value=False)
        self.ai_model = None
        self._ai_model_load_attempted = False

        # QUAN TRỌNG: chốt lại giao thức lái ĐÚNG lúc bấm nút START, không
        # đọc lại use_ai_model.get() ở mỗi frame nữa. Lý do: nếu người dùng
        # đổi checkbox giữa lúc đang chạy, xe đang được STM32 lái bằng 1
        # trong 2 cơ chế hoàn toàn khác nhau (MODE_AUTO/PID-IR hay
        # MODE_AI_LINE) - đổi giữa chừng phía PC mà không gửi lại lệnh
        # chuyển mode xuống STM32 sẽ làm 2 bên lệch trạng thái.
        #   'MODE2_PID'  : gửi "A" rồi "N0" - STM32 tự bám line bằng 5 mắt
        #                  IR (Run_Line_PID trong mode2_obstacle.c) nhưng
        #                  ĐÃ TẮT hành vi né vật cản vật lý (dừng-quét
        #                  servo-rẽ) vì thừa và gây nhiễu lúc Mode 4 đang
        #                  ghi nhãn/giám sát bằng camera. Camera CHỈ dùng
        #                  để giám sát + ghi nhãn, KHÔNG gửi bất kỳ lệnh
        #                  "I ..." nào xuống xe.
        #   'AI_LINE'    : gửi "I <error_x100>" mỗi frame - STM32 chạy PID
        #                  riêng trong mode4_ai.c trên error PC tính được.
        self._drive_mode = None
        # Khoảng cách vật cản gần nhất STM32 báo về (cm) - chỉ có giá trị
        # thật khi đang ở 'MODE2_PID' (mode2_obstacle.c mới gửi field
        # "dist" trong gói LOG). Dùng để KHÔNG ghi mẫu train trong lúc xe
        # đang né vật cản (rẽ ngang/lùi) - nếu không, ảnh + error CV lúc đó
        # phản ánh sai "line đang lệch bao nhiêu" (contour dễ bắt trúng vật
        # cản hoặc mép sàn), làm nhiễu nhãn dataset một cách có hệ thống.
        self._last_distance_cm = None

        self._build_layout()
        self._show_placeholder("Chưa có tín hiệu video")

    def _find_next_record_index(self):
        """Quét thư mục dataset, tìm số thứ tự lớn nhất trong các file
        "img_XXXXXX.jpg" đã có, trả về số đó + 1 để làm điểm bắt đầu ghi
        mới - đảm bảo không bao giờ ghi đè lên ảnh của phiên trước. Nếu
        thư mục rỗng hoặc chưa có file nào đúng định dạng, trả về 0."""
        max_index = -1
        prefix, suffix = "img_", ".jpg"
        try:
            for name in os.listdir(config.AI_DATASET_DIR):
                if name.startswith(prefix) and name.endswith(suffix):
                    digits = name[len(prefix):-len(suffix)]
                    if digits.isdigit():
                        max_index = max(max_index, int(digits))
        except OSError:
            pass
        return max_index + 1

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
            parent, text="▶ BẮT ĐẦU", command=self.toggle_ai
        )
        self.ai_btn.pack(fill="x", ipady=5)

        tk.Label(
            parent,
            text="Đặt xe trên line đen rồi bấm nút.\n"
                 "- TẮT checkbox bên dưới: xe lái bằng Mode 2 (5 mắt IR),\n"
                 "  ĐÃ TẮT né vật cản vật lý - camera chỉ giám sát/ghi nhãn,\n"
                 "  không lái xe, không tự rẽ tránh.\n"
                 "- BẬT checkbox: PC dùng AI Model tính error và lái xe qua\n"
                 "  camera, giống hệt cơ chế cũ.",
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
            self._drive_mode = None
            self._last_distance_cm = None
            self.ai_btn.config(text="▶ BẮT ĐẦU")
            self.ai_info.config(text="Line: chưa phát hiện", fg="#999999")
            self.ai_model_check.config(state="normal")  # nhả khoá checkbox
            communication.link.stop_all()
            self._log("[LỆNH] STOP")
        else:
            if not communication.link.connected:
                self._log("[LỖI] Chưa kết nối xe.")
                return

            # SỬA LỖI: trước đây không kiểm tra camera trước khi bật AI
            # Line - nếu quên bật camera, không frame nào được xử lý nên
            # send_ai_error() không bao giờ được gọi. Vẫn giữ điều kiện này
            # cho CẢ 2 nhánh lái bên dưới: kể cả nhánh MODE2_PID (không gửi
            # "I ...") vẫn cần camera bật để CV cổ điển có ảnh mà tính error
            # ghi nhãn - nếu không, bấm "Ghi dữ liệu" sẽ không lưu được gì.
            if not self.cam_connected:
                self._log("[LỖI] Cần bật camera trước. Bấm 'Bật camera' rồi thử lại.")
                return

            # Chốt giao thức lái NGAY LÚC BẤM START, dùng trạng thái checkbox
            # tại thời điểm này - xem giải thích ở _drive_mode trong __init__.
            if self.use_ai_model.get():
                if self.ai_model is None:
                    self._log("[LỖI] Chưa load được AI Model - tick lại checkbox để thử load, "
                              "hoặc bỏ tick để chạy bằng Mode 2 (IR).")
                    return
                self._drive_mode = "AI_LINE"
            else:
                self._drive_mode = "MODE2_PID"

            self.ai_running = True
            self.ai_btn.config(text="■ DỪNG")
            # KHÔNG khoá checkbox nữa (khác bản trước). Lý do an toàn:
            # _drive_mode (biến quyết định GIAO THỨC UART - "I ..." hay
            # "A") đã CHỐT CỨNG ngay dòng trên, không đọc lại checkbox nữa
            # trong suốt quá trình chạy. Checkbox chỉ còn ảnh hưởng tới
            # `ai_active` bên trong _process_frame() (đọc live mỗi frame)
            # - quyết định NGUỒN error (AI Model hay CV cổ điển) khi
            # _drive_mode == "AI_LINE", nhưng dù nguồn nào thì vẫn gửi
            # đúng 1 lệnh "I <error_x100>" y hệt xuống STM32 - không có
            # khái niệm "lệch giao thức" ở đây. Nhờ vậy giờ có thể bật/tắt
            # AI Model NGAY LÚC xe đang chạy để so sánh trực tiếp AI vs CV
            # mà không cần dừng xe - cách chẩn đoán nhanh nhất khi nghi
            # ngờ lỗi nằm ở model chứ không phải pipeline/detect.

            if self._drive_mode == "AI_LINE":
                communication.link.start_ai_line()
                self._log("[LỆNH] START (AI Model) - đặt xe trên line đen.")
            else:
                # TÁI SỬ DỤNG PID bám line 5 mắt IR của Mode 2, nhưng KHÔNG
                # kèm hành vi né vật cản (dừng - quét servo - rẽ) của Mode 2
                # gốc - Mode 4 tự xử lý vật cản/ghi nhãn qua camera ở phía
                # PC (xem avoiding_obstacle trong _process_frame), nên né
                # vật cản vật lý ở đây là THỪA và có thể làm xe tự ý rẽ
                # giữa lúc đang ghi mẫu train, nhiễu cả dataset lẫn hành
                # trình. set_obstacle_avoid(False) PHẢI gọi SAU start_auto()
                # - xem giải thích thứ tự trong communication.py.
                communication.link.start_auto()
                communication.link.set_obstacle_avoid(False)
                self._log("[LỆNH] START (Mode 2 / IR, đã TẮT né vật cản) - "
                          "CV chỉ giám sát và ghi nhãn, không lái, không né.")

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
                        self._drive_mode = None
                        self.ai_btn.config(text="▶ BẮT ĐẦU")
                        self.ai_info.config(text="Mất kết nối", fg=config.COLORS["error"])
                        self.ai_model_check.config(state="normal")
                elif kind == "telemetry":
                    # SỬA LỖI: trước đây gói telemetry (dict) bị rơi vào
                    # nhánh "else" bên dưới và bị log nhầm thành lỗi dạng
                    # f"[LỖI] {payload}" - vì kind=="telemetry" không khớp
                    # "info". Chỉ có field "dist" thực sự cần dùng ở đây:
                    # STM32 CHỈ gửi field này khi đang chạy MODE_AUTO (xem
                    # mode2_obstacle.c ghi log_distance_cm_x10), tức đúng
                    # lúc self._drive_mode == "MODE2_PID" - dùng để tránh
                    # ghi mẫu train lúc xe đang né vật cản (xem _process_frame).
                    dist = payload.get("dist")
                    if dist is not None:
                        self._last_distance_cm = dist
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
        self.mjpeg = MJPEGReader(url, on_error=self._on_stream_error, rotate=config.VIDEO_ROTATE)
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

                    # SỬA LỖI HIỂN THỊ (không ảnh hưởng lái xe): toạ độ các
                    # điểm trong "largest" là toạ độ CỤC BỘ bên trong roi
                    # (gốc 0,0 = mép trên của phần đã cắt bởi crop_y), vì
                    # findContours() chạy trên "thresh" vốn tính từ "roi",
                    # không phải từ bgr_frame gốc. cx/cy dùng để tính error
                    # đã được cộng lại +crop_y ở trên nên ĐÚNG, nhưng contour
                    # dùng để VẼ lại bị vẽ thẳng lên overlay (full-size) mà
                    # KHÔNG cộng lại +crop_y - khiến đường viền xanh trên
                    # màn hình bị đẩy lệch lên trên đúng bằng số pixel đã
                    # cắt, trông như đang bắt trúng vật ở phần trên khung
                    # hình dù thực chất vùng đó chưa từng nằm trong roi.
                    shifted_contour = largest + np.array([[0, crop_y]])
                    cv2.drawContours(overlay, [shifted_contour], -1, (0, 255, 0), 2)
                    cv2.circle(overlay, (cx, cy), 5, (0, 0, 255), -1)

                    deviation = (cx - w / 2) / (w / 2)
                    error = deviation * 4.0

        # ---- GHI DỮ LIỆU: dùng error từ CV cổ điển làm nhãn tự động ----
        # Luôn ghi bằng error CỔ ĐIỂN (không phải error AI), kể cả khi đang
        # ở nhánh AI_LINE - vì đây là nhãn "đúng" để làm dữ liệu train.
        # BỎ QUA lúc đang né vật cản (chỉ có ý nghĩa ở nhánh MODE2_PID, vì
        # chỉ nhánh đó mới nhận được field "dist" từ telemetry) - contour
        # CV lúc xe đang rẽ ngang/lùi để né thường bắt trúng vật cản/mép
        # sàn thay vì vạch line thật, ghi vào sẽ làm nhiễu nhãn dataset.
        avoiding_obstacle = (
            self._drive_mode == "MODE2_PID"
            and self._last_distance_cm is not None
            and self._last_distance_cm < 20.0
        )
        if self.record_enabled and detected and self.ai_running and not avoiding_obstacle:
            self._save_sample(bgr_frame, error)

        if self._drive_mode == "MODE2_PID":
            # KHÔNG gửi bất kỳ lệnh nào xuống STM32 ở đây - xe đang được
            # PID nội bộ trên STM32 (mode2_obstacle.c) lái hoàn toàn độc
            # lập bằng 5 mắt IR + né vật cản, y hệt mode2_auto.py. Camera/
            # CV ở nhánh này CHỈ quan sát + ghi nhãn (khối phía trên), never
            # gọi communication.link.send_ai_error()/start_ai_line() - nếu
            # gọi nhầm, "I ..." có thể bị STM32 hiểu là lệnh chuyển sang
            # MODE_AI_LINE, làm gián đoạn PID-IR đang chạy tốt.
            status = f"[GIÁM SÁT] CV error={error:+.2f} {'[OK]' if detected else '[MẤT]'}"
            if avoiding_obstacle:
                status += f"  (đang né vật cản {self._last_distance_cm:.0f}cm - không ghi mẫu)"
            self.after(0, lambda s=status: self.ai_info.config(
                text=s, fg=config.COLORS["ok"] if detected else "#999999"
            ))
            return overlay

        if self._drive_mode == "AI_LINE":
            # ---- THAY ERROR BẰNG AI MODEL (chỉ khi có line) ----
            # Giữ `detected` từ CV cổ điển làm tín hiệu mất-line: threshold
            # vẫn đáng tin hơn để trả lời "có tồn tại 1 vùng tối giống line
            # không", AI model chỉ đảm nhiệm tính error CHÍNH XÁC hơn khi
            # có line.
            ai_active = self.use_ai_model.get() and self.ai_model is not None
            if ai_active and detected:
                try:
                    with torch.no_grad():
                        tensor = frame_to_tensor(bgr_frame, config.AI_LINE_CROP_TOP)
                        error = float(self.ai_model(tensor).item())
                except Exception as e:
                    self._log(f"[LỖI AI MODEL] {e} - dùng tạm CV cổ điển.")

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
        # FIX BUG 2 (watchdog bypass qua heartbeat): TRƯỚC ĐÂY chỉ gọi
        # _stop_camera() - ai_running vẫn True, nên _start_link_polling() vẫn
        # tiếp tục gửi heartbeat() mỗi 50ms, làm mới last_command_tick bên
        # STM32 -> AUTO_LINK_TIMEOUT_TICKS (~1.5s) không bao giờ kích hoạt dù
        # không còn frame nào được xử lý. Xe chạy vô thời hạn với lệnh lái
        # CŨ (khác "mất line" - sentinel lost-line không bắt được trường hợp
        # mất hẳn NGUỒN dữ liệu này). Giờ dừng hẳn (gửi "S") trước khi tắt
        # hiển thị camera.
        if self.ai_running:
            self.after(0, self.toggle_ai)
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