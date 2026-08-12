#!/usr/bin/env python3
"""
test_stm32_usb.py
-------------------
Nói chuyện THẲNG với STM32 qua cáp USB ST-LINK. Không cần ESP32, không cần
WiFi, không cần đấu một sợi dây nào.

    pip install pyserial
    python test_stm32_usb.py

Làm được điều này là nhờ trên board NUCLEO-F401RE, hai chân PA2/PA3 (USART2)
mặc định đã nối sẵn vào con ST-LINK để làm cổng COM ảo. Tức là đúng sợi cáp
bạn dùng để nạp code cũng là một đường UART thẳng tới STM32.

Đây là phép thử tách bạch được hai thứ mà từ nãy tới giờ cứ lẫn vào nhau:
  - Firmware STM32 có chạy đúng không?
  - Đường ESP32 <-> STM32 có thông không?
Nếu gõ tay qua USB mà xe chạy, thì mọi lỗi còn lại nằm ở đoạn ESP32.
"""

import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Chưa có thư viện pyserial. Cài bằng lệnh:\n    pip install pyserial")
    sys.exit(1)


BAUD = 115200
# Từ khoá nhận dạng cổng COM của ST-LINK trên Windows/Linux/macOS
STLINK_HINTS = ("stlink", "st-link", "stmicroelectronics", "stm32")


def find_port():
    ports = list(list_ports.comports())
    if not ports:
        return None, []

    for p in ports:
        text = f"{p.description} {p.manufacturer} {p.hwid}".lower()
        if any(h in text for h in STLINK_HINTS):
            return p.device, ports
    return None, ports


def choose_port():
    auto, ports = find_port()
    if auto:
        print(f"  Tự tìm thấy ST-LINK ở cổng {auto}")
        return auto

    if not ports:
        print("  [HỎNG] Không thấy cổng COM nào. Đã cắm cáp USB vào Nucleo chưa?")
        return None

    print("  Không tự nhận ra ST-LINK. Các cổng đang có:")
    for i, p in enumerate(ports):
        print(f"    [{i}] {p.device} - {p.description}")
    try:
        idx = int(input("  Chọn số thứ tự cổng: ").strip())
        return ports[idx].device
    except (ValueError, IndexError, EOFError):
        return None


def read_for(ser, seconds):
    """Đọc gom trong ngần này giây."""
    out = b""
    t0 = time.time()
    while time.time() - t0 < seconds:
        n = ser.in_waiting
        if n:
            out += ser.read(n)
        else:
            time.sleep(0.05)
    return out


def clean(raw):
    return [ln.strip() for ln in raw.decode("utf-8", errors="replace").splitlines() if ln.strip()]


def garbled(lines):
    joined = "".join(lines)
    if not joined:
        return False
    weird = sum(1 for c in joined if not (c.isprintable() and ord(c) < 128))
    return weird > len(joined) * 0.3


def main():
    drive = "--drive" in sys.argv

    print("=" * 64)
    print("  KIỂM TRA STM32 QUA CÁP USB (không qua ESP32)")
    print("=" * 64)

    port = choose_port()
    if not port:
        return

    try:
        ser = serial.Serial(port, BAUD, timeout=0.2)
    except serial.SerialException as e:
        print(f"  [HỎNG] Không mở được {port}: {e}")
        print("         Đóng Serial Monitor / PuTTY / STM32CubeIDE đang giữ cổng này rồi thử lại.")
        return

    print(f"  Đã mở {port} @ {BAUD} baud.\n")
    time.sleep(0.3)
    ser.reset_input_buffer()

    # ---- Nhờ người dùng nhấn RESET để bắt câu chào lúc khởi động ----
    print("  >>> NHẤN NÚT RESET (nút đen) TRÊN BOARD NUCLEO NGAY BÂY GIỜ <<<")
    print("      (đang nghe trong 5 giây...)")
    boot = clean(read_for(ser, 5.0))
    if boot:
        print("      Nhận được:")
        for ln in boot:
            print(f"        {ln}")
    else:
        print("      (không nhận được gì)")
    print()

    # ---- Gửi các lệnh thăm dò ----
    results = {}
    for label, cmd in [("PING", b"PING\n"), ("STOP", b"STOP\n"), ("M,F,0", b"M,F,0\n")]:
        ser.reset_input_buffer()
        print(f"  Gửi {label:<7} ...", end=" ", flush=True)
        ser.write(cmd)
        lines = clean(read_for(ser, 1.2))
        results[label] = lines
        print(lines if lines else "(im lặng)")

    if drive:
        print("\n  Gửi M,F,60 trong 2 giây - BÁNH XE PHẢI QUAY BÂY GIỜ...")
        t0 = time.time()
        while time.time() - t0 < 2.0:
            ser.write(b"M,F,60\n")
            time.sleep(0.1)
        ser.write(b"STOP\n")
        print("  Đã phanh.")

    ser.close()

    # ---- Kết luận ----
    everything = boot + sum(results.values(), [])
    joined = " ".join(everything)

    print("\n" + "=" * 64)
    print("  KẾT LUẬN")
    print("=" * 64)

    if not everything:
        print("  STM32 hoàn toàn im lặng qua cả cáp USB.")
        print()
        print("  Nghĩa là vấn đề KHÔNG nằm ở ESP32 hay Python chút nào.")
        print("  Kiểm tra:")
        print("   1. Đã nạp CẢ HAI file main.c và hardware.c mới chưa?")
        print("      Thiếu hardware.c thì USART2/PA2/PA3 không được cấu hình,")
        print("      chương trình vẫn chạy nhưng câm như hến - đúng hiện tượng này.")
        print("   2. Nạp có báo thành công không? Thử nạp lại và xem log của IDE.")
        print("   3. Có mở PuTTY/Serial Monitor nào khác đang giữ cổng COM không?")
        print("   4. Cầu hàn SB13/SB14 trên board còn nguyên chứ? Nếu bạn đã gỡ")
        print("      chúng để đưa PA2/PA3 ra chân ngoài thì đường USB này đứt,")
        print("      và phép thử qua USB sẽ không dùng được nữa.")

    elif garbled(everything):
        print("  Có dữ liệu nhưng toàn ký tự rác.")
        print("   => Sai tốc độ baud. Kiểm tra USART2->BRR trong hardware.c:")
        print("      phải là 16000000 / 115200 (vi điều khiển chạy HSI 16MHz).")

    elif "ACK:PONG" in joined:
        print("  ✓ FIRMWARE MỚI ĐANG CHẠY TỐT, USART2 hoạt động đúng.")
        print()
        if drive:
            print("  Nếu bánh xe vừa rồi VẪN không quay -> lỗi ở phần công suất:")
        else:
            print("  Chạy lại với --drive (nhấc xe lên trước) để thử quay bánh.")
            print("  Nếu bánh vẫn đứng im -> lỗi ở phần công suất:")
        print("   - Nguồn động cơ đã bật chưa (thường tách riêng khỏi nguồn logic)?")
        print("   - Chân enable của mạch cầu H có bị bỏ trống không?")
        print()
        print("  Còn nếu bánh QUAY được qua USB mà không quay qua WiFi, thì lỗi")
        print("  nằm gọn ở đoạn ESP32 <-> STM32: dây GPIO47->PA3, GPIO21->PA2,")
        print("  GND chung, và xung đột ST-LINK trên PA3 (cầu hàn SB13/SB14).")

    elif "[STM32]" in joined:
        print("  STM32 có khởi động (thấy câu chào) nhưng KHÔNG trả lời PING.")
        print("   => Đang chạy firmware CŨ. main.c cũ chỉ hiểu START và STOP,")
        print("      gặp PING hay M,F,60 thì bỏ qua lặng lẽ.")
        print("   => Nạp lại main.c mới.")

    else:
        print("  Có tín hiệu nhưng không khớp mẫu nào đã biết:")
        for ln in everything[:8]:
            print(f"      {ln!r}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass