"""
main.py
--------
File CHẠY CHÍNH. Chỉ làm 4 việc:
  1. Tạo cửa sổ chính
  2. Tạo thanh nút chuyển Mode
  3. Hiện Frame tương ứng (Mode 1..4)
  4. Phanh xe + đóng kết nối khi tắt chương trình
"""

import tkinter as tk
from tkinter import ttk

import config
import communication
from mode1_manual import ManualControlFrame
from mode2_auto import AutoPatrolFrame
from mode3_follow import FollowObjectFrame
from mode4_ai import AILineFrame


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
        self._create_frame("mode3", FollowObjectFrame)
        self._create_frame("mode4", AILineFrame)

        self.show_frame("mode1")

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
        ttk.Button(bar, text="Mode 3: Bám vật thể",
                   command=lambda: self.show_frame("mode3")).pack(side="left", padx=5, pady=6)
        ttk.Button(bar, text="Mode 4: AI dò line",
                   command=lambda: self.show_frame("mode4")).pack(side="left", padx=5, pady=6)

    def _build_container(self):
        self.container = tk.Frame(self, bg=config.COLORS["bg"])
        self.container.pack(fill="both", expand=True)

    def _create_frame(self, key, frame_class):
        frame = frame_class(self.container)
        frame.place(relwidth=1, relheight=1)
        self.frames[key] = frame

    def show_frame(self, key):
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
        communication.link.close()
        self.destroy()


if __name__ == "__main__":
    app = MainApp()
    app.mainloop()
