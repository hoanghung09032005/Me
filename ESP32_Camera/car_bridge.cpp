#include <Arduino.h>
#include <WiFi.h>
#include "esp_system.h"

#include "car_bridge.h"

#define STM32_UART_BAUD   115200
#define STM32_UART_RX     21
#define STM32_UART_TX     47

/* ============================================================================
 * BRIDGE_DEBUG_MIRROR:
 * ----------------------------------------------------------------------------
 * TRƯỚC ĐÂY: carBridgeLoop() gọi Serial.write(c) cho TỪNG BYTE ở CẢ 2 CHIỀU
 * (lệnh GUI -> STM32 và LOG telemetry STM32 -> GUI) để tiện xem debug qua
 * Serial Monitor. VẤN ĐỀ: trên ESP32-S3, "Serial" mặc định là cổng USB-CDC
 * GỐC (native USB), không phải UART vật lý rời. Khi xe chạy THẬT (chạy
 * pin, không cắm PC / không mở Serial Monitor), không có ai đọc phía USB -
 * bộ đệm TX của USB-CDC đầy dần, và tuỳ phiên bản Arduino-ESP32 core,
 * Serial.write() có thể BỊ BLOCK khi bộ đệm đầy. Nếu carBridgeLoop() bị
 * kẹt đủ lâu (vài giây) vì lý do này, Task Watchdog của ESP-IDF (mặc định
 * ~5s) sẽ tự PANIC VÀ REBOOT CHIP - toàn bộ kết nối WiFi/TCP bị ngắt đột
 * ngột, phía PC nhận đúng lỗi "connection was forcibly closed"
 * (WinError 10054) dù code gửi TCP đã gom byte đúng cách (xem txBuf bên
 * dưới) - vì gốc vấn đề không nằm ở TCP mà ở chính debug mirror này.
 *
 * BÂY GIỜ: TẮT HẲN mirror này theo mặc định (macro = 0). Đặt = 1 CHỈ khi
 * đang cắm dây USB + mở Serial Monitor để debug tại bàn - KHÔNG bật khi xe
 * chạy thật/chạy pin không giám sát qua USB. */
#define BRIDGE_DEBUG_MIRROR   0

/* Bộ đệm gom dữ liệu STM32 -> PC.
 * TRƯỚC ĐÓ NỮA: mỗi byte đọc được từ Serial1 bị gửi bằng MỘT lệnh
 * bridgeClient.write() riêng -> mỗi byte tạo ra một gói TCP nhỏ. Khi luồng
 * video MJPEG đang chạy song song, hàng loạt gói tin nhỏ này cạnh tranh
 * băng thông/CPU với luồng video, làm telemetry đến trễ/rớt.
 * Gom byte cho tới khi gặp '\n' (mọi dòng STM32 gửi đều kết thúc bằng '\n',
 * xem UART_SendString ở main.c) rồi gửi MỘT LẦN bằng một lệnh write() ->
 * giảm số gói TCP đi rất nhiều. (Fix này ĐÃ CÓ TỪ TRƯỚC và vẫn đúng - lỗi
 * WinError 10054 vẫn xảy ra sau khi có fix này là dấu hiệu cho thấy gốc
 * vấn đề thực ra nằm ở BRIDGE_DEBUG_MIRROR phía trên, không phải ở đây.) */
#define BRIDGE_TX_BUF_SIZE 160

WiFiServer bridgeServer(8080);
WiFiClient bridgeClient;
static bool clientWasConnected = false;

static uint8_t txBuf[BRIDGE_TX_BUF_SIZE];
static size_t txLen = 0;

static void flushTxBuf()
{
    if (txLen == 0) {
        return;
    }
    if (bridgeClient && bridgeClient.connected()) {
        bridgeClient.write(txBuf, txLen);
    }
    txLen = 0;
}

static void sendBridgeInfo(const char *message)
{
    if (bridgeClient && bridgeClient.connected()) {
        flushTxBuf();   // đẩy nốt dữ liệu telemetry đang gom dở để giữ đúng thứ tự
        bridgeClient.print("[ESP32-S3] ");
        bridgeClient.println(message);
    }
}

/* Báo cho GUI biết NGUYÊN NHÂN LẦN RESET GẦN NHẤT của chính ESP32-S3 -
 * y hệt cơ chế "STM32,RESET_CAUSE,..." đã có bên firmware STM32
 * (Report_Reset_Cause() trong main.c), nhưng cho con chip ESP32-S3 này.
 * Gửi ngay khi GUI vừa kết nối (không phải lúc setup(), vì lúc setup() PC
 * chưa kết nối TCP nên message sẽ rơi vào hư vô).
 *
 * CÁCH DÙNG khi nghi ngờ lại gặp lỗi rớt kết nối "forcibly closed": mở lại
 * GUI, kết nối xe, xem dòng "[ESP32-S3] ESP32_RESET_REASON,..." xuất hiện
 * ngay đầu log:
 *   - BROWNOUT       -> nguồn bị sụt áp thật (motor kéo dòng lớn) - cần
 *                        tách nguồn động cơ khỏi nguồn logic ESP32/camera,
 *                        thêm tụ lọc lớn (vd 1000-2200uF) gần chân 5V ESP32.
 *   - TASK_WDT/INT_WDT -> một tác vụ nào đó (rất có thể là chính
 *                        carBridgeLoop() khi BRIDGE_DEBUG_MIRROR còn bật)
 *                        bị treo quá lâu, watchdog buộc phải reset.
 *   - POWERON         -> chỉ là lần cấp nguồn bình thường, không phải lỗi. */
static void sendResetReason()
{
    esp_reset_reason_t reason = esp_reset_reason();
    const char *text;

    switch (reason) {
        case ESP_RST_POWERON:   text = "POWERON";   break;
        case ESP_RST_BROWNOUT:  text = "BROWNOUT";  break;
        case ESP_RST_TASK_WDT:  text = "TASK_WDT";  break;
        case ESP_RST_INT_WDT:   text = "INT_WDT";   break;
        case ESP_RST_WDT:       text = "WDT";       break;
        case ESP_RST_PANIC:     text = "PANIC";     break;
        case ESP_RST_SW:        text = "SOFTWARE";  break;
        case ESP_RST_DEEPSLEEP: text = "DEEPSLEEP";  break;
        case ESP_RST_EXT:       text = "EXT_PIN";   break;
        default:                text = "OTHER";     break;
    }

    char msg[48];
    snprintf(msg, sizeof(msg), "ESP32_RESET_REASON,%s", text);
    sendBridgeInfo(msg);
}

static void stopStm32Car()
{
    /* STM32 treats S as a global emergency-stop command. */
    Serial1.print("S\n");
}

void carBridgeBegin()
{
    /* ESP32 RX receives STM32 PA9/TX; ESP32 TX drives STM32 PA10/RX. */
    Serial1.begin(STM32_UART_BAUD, SERIAL_8N1, STM32_UART_RX, STM32_UART_TX);

    bridgeServer.begin();
    clientWasConnected = false;
    txLen = 0;
}

void carBridgeLoop()
{
    bool clientConnected = bridgeClient && bridgeClient.connected();

    /* Keep a single controlling PC. Replacing a disconnected client first
     * stops the car so a lost TCP link cannot leave AUTO mode running. */
    if (bridgeServer.hasClient()) {
        if (!clientConnected) {
            if (clientWasConnected) {
                stopStm32Car();
            }
            if (bridgeClient) {
                bridgeClient.stop();
            }
            txLen = 0;
            bridgeClient = bridgeServer.available();
            clientConnected = bridgeClient && bridgeClient.connected();
            if (clientConnected) {
                /* QUAN TRỌNG: setNoDelay() phải gọi TRÊN CLIENT vừa accept.
                 * Bản cũ gọi bridgeServer.setNoDelay(true) - không có tác
                 * dụng gì tới socket dữ liệu thật sự, Nagle vẫn bật khiến
                 * lệnh lái bị delay thêm vài chục ms không cần thiết. */
                bridgeClient.setNoDelay(true);
                sendBridgeInfo("TCP connected; UART bridge ready (RX=21, TX=47, 115200).");
                sendResetReason();
            }
        } else {
            WiFiClient rejectedClient = bridgeServer.available();
            rejectedClient.stop();
        }
    }

    if (clientWasConnected && !clientConnected) {
        stopStm32Car();
        txLen = 0;
    }
    clientWasConnected = clientConnected;

    /* TCP -> UART: GUI commands to STM32. */
    if (clientConnected) {
        while (bridgeClient.available()) {
            char c = (char)bridgeClient.read();
            Serial1.write((uint8_t)c);
#if BRIDGE_DEBUG_MIRROR
            Serial.write(c);
#endif
        }
    }

    /* UART -> TCP: STM32 telemetry to GUI, gom theo dòng thay vì gửi từng byte. */
    while (Serial1.available()) {
        char c = (char)Serial1.read();
#if BRIDGE_DEBUG_MIRROR
        Serial.write(c);
#endif
        if (clientConnected) {
            if (txLen < BRIDGE_TX_BUF_SIZE) {
                txBuf[txLen++] = (uint8_t)c;
            }
            if (c == '\n' || txLen >= BRIDGE_TX_BUF_SIZE) {
                flushTxBuf();
            }
        }
    }
}

bool carBridgeHasClient()
{
    return bridgeClient && bridgeClient.connected();
}