#ifndef CAR_BRIDGE_H
#define CAR_BRIDGE_H

// =====================================================================
// car_bridge.h
// ---------------------------------------------------------------------
// Biến ESP32-S3 CAM thành cầu nối 2 chiều giữa máy tính và STM32,
// thay thế hoàn toàn vai trò của ESP8266 cũ:
//
//     Python (TCP :8080)  <--WiFi-->  ESP32-S3  <--UART 115200-->  STM32
//
// Chạy SONG SONG với camera web server (port 80 = điều khiển camera,
// port 81 = luồng MJPEG), không đụng chạm gì tới code camera có sẵn.
// =====================================================================

// Gọi 1 lần trong setup(), SAU khi WiFi đã kết nối xong.
void carBridgeBegin();

// Gọi liên tục trong loop(). Hàm này không chặn (non-blocking).
void carBridgeLoop();

// Có máy tính nào đang cắm vào cầu nối không?
bool carBridgeHasClient();

#endif  // CAR_BRIDGE_H
