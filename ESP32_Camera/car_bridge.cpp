#include <Arduino.h>
#include <WiFi.h>
#include "car_bridge.h"

WiFiServer bridgeServer(8080);
WiFiClient bridgeClient;

void carBridgeBegin() {
    // Khởi tạo Serial1, sử dụng RX = 21, TX = 47, tốc độ 115200
    Serial1.begin(115200, SERIAL_8N1, 21, 47);
    
    bridgeServer.begin();
    bridgeServer.setNoDelay(true); // Tắt thuật toán Nagle để giảm độ trễ TCP
}

void carBridgeLoop() {
    // 1. Quản lý Client kết nối vào cổng TCP 8080
    if (bridgeServer.hasClient()) {
        if (!bridgeClient || !bridgeClient.connected()) {
            if (bridgeClient) bridgeClient.stop();
            bridgeClient = bridgeServer.available();
        } else {
            // Từ chối client mới nếu đã có người kết nối
            WiFiClient temp = bridgeServer.available();
            temp.stop();
        }
    }
    
    // 2. Chuyển tiếp TCP -> UART (Nhận lệnh từ Python đẩy xuống STM32)
    if (bridgeClient && bridgeClient.connected()) {
        while (bridgeClient.available()) {
            Serial1.write(bridgeClient.read());
        }
    }
    
    // 3. Chuyển tiếp UART -> TCP (Nhận Telemetry từ STM32 đẩy lên Python)
    while (Serial1.available()) {
        char c = Serial1.read();
        if (bridgeClient && bridgeClient.connected()) {
            bridgeClient.write(c);
        }
    }
}