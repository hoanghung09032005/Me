#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include "car_bridge.h"

#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    15
#define SIOD_GPIO_NUM    4
#define SIOC_GPIO_NUM    5
#define Y9_GPIO_NUM      16
#define Y8_GPIO_NUM      17
#define Y7_GPIO_NUM      18
#define Y6_GPIO_NUM      12
#define Y5_GPIO_NUM      10
#define Y4_GPIO_NUM      8
#define Y3_GPIO_NUM      9
#define Y2_GPIO_NUM      11
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM    13

const char *ssid = "Ngo5C6";
const char *password = "12345678a@";

void startCameraServer();
void setupLedFlash();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  // GIỮ NGUYÊN 20MHz: từng thử tăng lên 24MHz nhưng bị nhiễu hình (artifact),
  // đã revert. Muốn tăng chất lượng thì nên chỉnh jpeg_quality/sharpness
  // (bên dưới) thay vì đụng vào xclk.
  config.xclk_freq_hz = 20000000;

  config.frame_size = FRAMESIZE_VGA;   // 640x480 - tăng thêm 1 bước nữa từ CIF
                                        // (400x296), vẫn đi từng bước một thay vì
                                        // nhảy thẳng lên mức cao nhất (SVGA/UXGA),
                                        // đúng lý do đã ghi ở bước CIF trước: ảnh
                                        // to hơn = ESP32 nén nặng hơn mỗi khung +
                                        // tốn băng thông WiFi hơn + khả năng nóng
                                        // máy hơn. BẮT BUỘC chạy thử liên tục ít
                                        // nhất 10-15 phút rồi kiểm tra nhiệt độ vỏ
                                        // chip camera (sờ tay hoặc súng đo nhiệt)
                                        // trước khi coi bước này là an toàn - nếu
                                        // >60°C hoặc mất kết nối lặp lại, hạ về
                                        // lại FRAMESIZE_CIF ngay (xem lịch sử
                                        // overheating đã từng gặp ở phần
                                        // jpeg_quality/sharpness bên dưới).
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_DRAM;
  // jpeg_quality: 0-63, SỐ CÀNG THẤP ẢNH CÀNG NÉT nhưng ESP32 phải nén
  // NẶNG HƠN mỗi frame (tốn CPU + dòng điện hơn). ĐÃ RÚT LẠI về 20 (mặc
  // định gốc) sau khi ghi nhận camera OV3660 nóng >60°C kèm mất kết nối -
  // giảm quality (12) là nghi phạm hàng đầu vì đúng hướng "bắt ESP32 làm
  // việc nặng hơn liên tục", CHƯA kiểm chứng được trên phần cứng thật
  // trước khi đổi. GIỮ NGUYÊN 20 ở lần tăng độ phân giải này - KHÔNG đổi
  // 2 biến (resolution và quality) cùng lúc, để nếu có vấn đề về nhiệt/mất
  // kết nối thì biết chắc nguyên nhân là do đổi resolution, không lẫn với
  // quality. Muốn nét hơn nữa, đợi xác nhận VGA ổn định rồi mới thử hạ
  // quality dần một mình (vd 20 -> 18), không đổi chung với resolution.
  config.jpeg_quality = 10;
  config.fb_count = 1;

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.fb_count = 2;
        config.jpeg_quality = 20;
        config.grab_mode = CAMERA_GRAB_LATEST;
    }
  } else {
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, -2);
    s->set_contrast(s, 1);
    s->set_saturation(s, -2);
  }
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_VGA);
  }

  // ĐÃ RÚT LẠI set_sharpness()/set_denoise() đã thêm lượt trước: đây là
  // 2 thay đổi CHƯA kiểm chứng được trên phần cứng thật (không có thiết
  // bị để test), và trùng thời điểm ghi nhận camera OV3660 nóng >60°C +
  // mất kết nối timeout. Không có bằng chứng chắc chắn 2 dòng này là thủ
  // phạm, nhưng khi nghi ngờ ảnh hưởng tới an toàn phần cứng thì rút lại
  // trước, xác nhận hết nóng máy với cấu hình an toàn này đã rồi mới thử
  // bật lại TỪNG DÒNG MỘT (không bật cả 2 cùng lúc) để cô lập nguyên nhân
  // nếu vẫn muốn ảnh nét hơn sau này.

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  WiFi.begin(ssid, password);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  startCameraServer();
  Serial.println("Camera Server Started!");

  // Mở cầu nối STM32 (Bắt buộc dùng carBridgeBegin để cấu hình chân 21, 47)
  carBridgeBegin();
}

void loop() {
  carBridgeLoop();
  yield(); // <-- Đã sửa: Không block CPU, chống tràn bộ đệm UART
}
