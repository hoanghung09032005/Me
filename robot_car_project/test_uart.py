#!/usr/bin/env python3
"""
test_uart.py
--------------
Kiểm tra riêng đoạn ESP32-S3 <-> STM32, sau khi cổng 8080 đã nối được.

    python test_uart.py              # kiểm tra bình thường, xe KHÔNG chạy
    python test_uart.py --drive      # có quay bánh 1 giây (NHẤC XE LÊN TRƯỚC!)
    python test_uart.py --loopback   # kiểm tra riêng phần ESP32 (xem hướng dẫn dưới)

Script gửi lần lượt PING, STOP, M,F,0 rồi căn cứ vào việc STM32 trả lời
cái gì để kết luận. Nguyên tắc phân biệt:

    Firmware MỚI hiểu : PING, MANUAL, M,..., START, STOP
    Firmware CŨ hiểu  : chỉ START và STOP, mọi lệnh khác bị bỏ qua lặng lẽ

Nên nếu STOP có trả lời mà PING thì không, đích thị là firmware cũ.
"""

import socket
import sys
import time

import config


def collect(sock, seconds):
    """Gom mọi thứ nhận được trong ngần này giây."""
    out = b""
    t0 = time.time()
    sock.settimeout(0.2)
    while time.time() - t0 < seconds:
        try:
            chunk = sock.recv(512)
            if not chunk:
                break
            out += chunk
        except socket.timeout:
            pass
    return out


def clean_lines(raw):
    """Bỏ dòng chào của ESP32, chỉ giữ cái do STM32 gửi."""
    text = raw.decode("utf-8", errors="replace")
    return [ln.strip() for ln in text.splitlines()
            if ln.strip() and not ln.startswith("[ESP32-S3]")]


def looks_garbled(lines):
    """Nhiều ký tự lạ = sai baud hoặc thiếu GND chung."""
    joined = "".join(lines)
    if not joined:
        return False
    weird = sum(1 for c in joined if not (c.isprintable() and ord(c) < 128))
    return weird > len(joined) * 0.3


def main():
    drive = "--drive" in sys.argv
    loopback = "--loopback" in sys.argv

    if loopback:
        print("=" * 64)
        print("  CHẾ ĐỘ LOOPBACK - kiểm tra riêng phía ESP32")
        print("=" * 64)
        print("  Rút 2 dây đi STM32 ra, rồi nối TẮT GPIO47 với GPIO21 bằng 1 sợi")
        print("  dây nhỏ (chập TX vào chính RX của ESP32).")
        print("  Làm xong bấm Enter...")
        input()

    print(f"\n  Đang nối tới {config.ESP32_IP}:{config.CAR_PORT} ...")
    try:
        sock = socket.create_connection((config.ESP32_IP, config.CAR_PORT), timeout=4)
    except OSError as e:
        print(f"  [HỎNG] Không nối được cổng 8080: {e}")
        print("         Chạy chan_doan.py trước đã.")
        return
    print("  [ OK ] Đã nối.\n")

    boot = clean_lines(collect(sock, 1.5))
    if boot:
        print("  Nhận được ngay khi vừa nối:")
        for ln in boot:
            print(f"      {ln}")
        print()

    # ---------------- LOOPBACK ----------------
    if loopback:
        sock.sendall(b"XINCHAO\n")
        got = collect(sock, 1.5).decode("utf-8", errors="replace")
        sock.close()
        print("=" * 64)
        if "XINCHAO" in got:
            print("  [ OK ] Chữ gửi đi đã quay về nguyên vẹn.")
            print("         => Phần ESP32, Python và cả 2 chân GPIO47/21 đều TỐT.")
            print("         => Lỗi nằm ở STM32 hoặc ở 2 sợi dây nối sang STM32.")
        else:
            print("  [HỎNG] Không nhận lại được gì.")
            print(f"         (nhận được: {got!r})")
            print("         => Vấn đề ở chính ESP32: sai số chân trong car_bridge.cpp,")
            print("            dây chập chưa tiếp xúc, hoặc GPIO47/21 đã bị dùng cho việc khác.")
        print("=" * 64)
        return

    # ---------------- CÁC PHÉP THỬ ----------------
    results = {}
    for label, cmd, wait in [("PING", b"PING\n", 1.0),
                             ("STOP", b"STOP\n", 1.0),
                             ("M,F,0", b"M,F,0\n", 1.0)]:
        print(f"  Gửi {label:<7} ...", end=" ", flush=True)
        sock.sendall(cmd)
        lines = clean_lines(collect(sock, wait))
        results[label] = lines
        print(f"nhận về: {lines if lines else '(im lặng)'}")

    if drive:
        print("\n  Gửi M,F,60 trong 1 giây - bánh xe phải quay bây giờ...")
        t0 = time.time()
        while time.time() - t0 < 1.0:
            sock.sendall(b"M,F,60\n")
            time.sleep(0.1)
        sock.sendall(b"M,S,0\n")
        sock.sendall(b"STOP\n")
        print("  Đã phanh.")

    sock.close()

    # ---------------- KẾT LUẬN ----------------
    everything = boot + sum(results.values(), [])
    joined = " ".join(everything)

    print("\n" + "=" * 64)
    print("  KẾT LUẬN")
    print("=" * 64)

    if not everything:
        print("  STM32 KHÔNG trả lời gì cả. Chiều STM32 -> máy tính đang đứt.")
        print()
        print("  Kiểm tra theo thứ tự:")
        print("   1. Dây PA9 (TX của STM32) -> GPIO21 (RX của ESP32) đã nối chưa?")
        print("      Đây là sợi HAY QUÊN NHẤT vì lệnh gửi xuống vẫn chạy bình thường")
        print("      mà không cần nó - đúng như hiện tượng bạn đang gặp.")
        print("   2. GND của hai board đã nối với nhau chưa?")
        print("   3. STM32 đã được cấp nguồn chưa?")
        print("   4. Chạy: python test_uart.py --loopback")
        print("      để loại trừ khả năng lỗi nằm ở phía ESP32.")

    elif looks_garbled(everything):
        print("  Có dữ liệu về nhưng toàn ký tự rác.")
        print("   => Sai tốc độ baud, hoặc THIẾU DÂY GND CHUNG giữa 2 board.")
        print("      Cả hai đầu phải là 115200.")

    elif "ACK:PONG" in joined:
        print("  ✓ STM32 đang chạy firmware MỚI và cả 2 chiều đều thông.")
        print()
        if drive:
            print("  Nếu vừa rồi bánh xe VẪN không quay thì lỗi ở phần công suất:")
        else:
            print("  Nếu bấm nút vẫn không chạy, chạy lại với --drive (nhấc xe lên trước)")
            print("  và nếu bánh vẫn đứng im thì lỗi ở phần công suất:")
        print("   - Nguồn động cơ đã bật chưa? (thường tách riêng khỏi nguồn logic)")
        print("   - Chân enable của mạch cầu H có bị bỏ trống không?")
        print("   - Thử tăng thanh tốc độ lên 80-100%.")

    elif "ACK:STOP" in joined or "ACK:START" in joined:
        print("  ✗ STM32 CHƯA ĐƯỢC NẠP main.c MỚI.")
        print()
        print("  Bằng chứng: nó trả lời ACK cho STOP nhưng LỜ ĐI lệnh PING.")
        print("  Firmware cũ chỉ biết đúng START và STOP; gặp PING hay M,F,60")
        print("  thì không khớp nhánh nào và cũng không có else để báo lỗi,")
        print("  nên im lặng - đúng hiện tượng 'STOP thì nhận, lái thì không'.")
        print()
        print("  => Nạp lại STM32 bằng main.c mới. hardware.c giữ nguyên, không đổi.")

    else:
        print("  Có tín hiệu về nhưng không khớp mẫu nào đã biết:")
        for ln in everything[:8]:
            print(f"      {ln!r}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass