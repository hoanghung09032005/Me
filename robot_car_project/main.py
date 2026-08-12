"""
main.py
--------
File CHẠY CHÍNH của chương trình. Chỉ làm 4 việc:
  1. Tạo cửa sổ chính
  2. Tạo thanh nút chuyển Mode ở trên cùng
  3. Hiện Frame tương ứng (Mode 1 hoặc Mode 2)
  4. Phanh xe + đóng kết nối khi tắt chương trình

KHÔNG viết logic điều khiển/xử lý ảnh ở đây.
Logic Mode 1 nằm trong mode1_manual.py, Mode 2 nằm trong mode2_auto.py.

Chạy chương trình bằng lệnh:
    python main.py
"""

import tkinter as tk
from tkinter import ttk

import config
import communication
from mode1_manual import ManualControlFrame
from mode2_auto import AutoPatrolFrame


class MainApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(config.WINDOW_TITLE)
        self.geometry(config.WINDOW_SIZE)
        self.configure(bg=config.COLORS["bg"])

        self._build_mode_bar()
        self._build_container()

        self.frames = {}
        self._create_frame("mode1", ManualControlFrame)
        self._create_frame("mode2", AutoPatrolFrame)

        self.show_frame("mode1")

        # Bấm nút X đóng cửa sổ -> phanh xe trước rồi mới thoát
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_mode_bar(self):
        bar = tk.Frame(self, bg=config.COLORS["panel_bg"])
        bar.pack(fill="x", side="top")

        tk.Label(bar, text="  CHỌN CHẾ ĐỘ:", bg=config.COLORS["panel_bg"],
                 fg=config.COLORS["text"], font=("Segoe UI", 10, "bold")).pack(side="left", padx=5, pady=8)

        ttk.Button(bar, text="Mode 1: Lái bằng tay",
                   command=lambda: self.show_frame("mode1")).pack(side="left", padx=5, pady=6)
        ttk.Button(bar, text="Mode 2: Tự động dò line",
                   command=lambda: self.show_frame("mode2")).pack(side="left", padx=5, pady=6)

    def _build_container(self):
        # container này chứa cả 2 frame chồng lên nhau, dùng tkraise() để đổi cái nào hiện lên trên
        self.container = tk.Frame(self, bg=config.COLORS["bg"])
        self.container.pack(fill="both", expand=True)

    def _create_frame(self, key, frame_class):
        frame = frame_class(self.container)
        frame.place(relwidth=1, relheight=1)
        self.frames[key] = frame

    def show_frame(self, key):
        # Gọi on_leave() TRƯỚC để mode cũ kịp phanh xe và nhả tài nguyên,
        # rồi mới on_enter() cho mode mới - tránh 2 mode cùng đọc hàng đợi
        # sự kiện của communication.link một lúc.
        for other_key, frame in self.frames.items():
            if other_key != key and hasattr(frame, "on_leave"):
                frame.on_leave()

        self.frames[key].tkraise()

        if hasattr(self.frames[key], "on_enter"):
            self.frames[key].on_enter()

    def _on_close(self):
        for frame in self.frames.values():
            if hasattr(frame, "on_leave"):
                frame.on_leave()
        communication.link.close()   # tự gửi STOP trước khi ngắt
        self.destroy()


if __name__ == "__main__":
    app = MainApp()
    app.mainloop()
