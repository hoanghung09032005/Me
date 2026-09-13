#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_wifi.h"
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

// FIX TIẾP THEO (sau khi hạ quality 12->20 vẫn còn hơi lag lúc nóng):
// hạ độ phân giải VGA -> CIF, đúng bước lùi lại 1 nấc từng làm với OV3660
// trước khi tăng lên VGA. Ảnh nhỏ hơn = ít pixel phải nén mỗi khung = nhẹ
// hơn cho CPU/nhiệt, đổi lấy mượt/mát - đúng ưu tiên "không cần nét, chỉ
// cần đủ nhìn". Đây là biến DUY NHẤT đổi trong lần sửa này - giữ quality
// (20), xclk (20MHz), vflip/hmirror như đã chốt, không đổi gì khác.
static constexpr framesize_t CAMERA_FRAME_SIZE = FRAMESIZE_CIF;  // 400x296 - hạ 1 bước từ VGA
// FIX NÓNG MÁY: quality=12 (số càng thấp càng nén nặng = càng tốn CPU/nhiệt)
// từng bị nghi là thủ phạm gây nóng máy trên OV3660 (xem lịch sử project).
// Giờ xác nhận cam OV5640 cũng nóng ở quality=12 - nâng lên 20 (mức "an
// toàn" đã dùng ổn định với OV3660 trước đây) để ưu tiên mượt/mát hơn nét,
// đúng yêu cầu "video không cần quá nét, chỉ cần đủ nhìn".
static constexpr int CAMERA_JPEG_QUALITY = 20;                   // lower = sharper/larger/slower/hotter
static constexpr uint32_t CAMERA_XCLK_HZ = 18000000;             // keep stable on ESP32-S3 + OV5640

void startCameraServer();
void setupLedFlash();

static void applySensorTuning(sensor_t *s) {
  if (s == nullptr) {
    return;
  }

  if (s->id.PID == OV5640_PID) {
    s->set_framesize(s, CAMERA_FRAME_SIZE);
    s->set_quality(s, CAMERA_JPEG_QUALITY);
    s->set_brightness(s, 0);
    s->set_contrast(s, 1);
    s->set_saturation(s, 0);
    // Tắt sharpness/denoise trên chip (0 thay vì 1) - đây cũng là xử lý
    // thêm tốn CPU/nhiệt trên ISP của cảm biến, không cần thiết khi mục
    // tiêu là mượt/mát hơn là nét. Muốn nét hơn sau này, bật lại TỪNG DÒNG
    // MỘT sau khi đã xác nhận nhiệt độ ổn định lâu dài với cấu hình này.
    s->set_sharpness(s, 0);
    s->set_denoise(s, 0);
    // CHẨN ĐOÁN QUAN TRỌNG: đã thử GAINCEILING 8X/16X/32X và ae_level 1/2,
    // KHÔNG có bất kỳ khác biệt độ sáng nào - nghĩa là thuật toán AEC/AGC
    // tự động đang tự dừng ở một mức thấp hơn NHIỀU so với trần cho phép,
    // không hề dùng hết gain/exposure được cấp. Tắt hẳn tự động, ép tay
    // lên mức tối đa để biết chắc: nếu ép tay vẫn không sáng hơn -> giới
    // hạn vật lý thật (cảm biến+ống kính không nhận đủ photon, không có
    // tham số phần mềm nào cứu được). Nếu ép tay SÁNG HƠN HẲN -> lỗi nằm ở
    // thuật toán AEC/AGC của driver, không phải giới hạn vật lý - lúc đó
    // quay lại dùng exposure/gain THỦ CÔNG thay vì tự động.
    s->set_exposure_ctrl(s, 0);   // tắt tự động phơi sáng
    s->set_gain_ctrl(s, 0);       // tắt tự động gain - THIẾU DÒNG NÀY thì agc_gain ép tay bên dưới sẽ bị auto ghi đè lại
    // ĐÃ XÁC NHẬN qua ảnh cháy sáng: phòng đủ sáng thật, lỗi nằm ở AEC/AGC
    // tự động (không tận dụng hết trần gain đã cấp trước đó) - không phải
    // giới hạn vật lý. Chuyển hẳn sang ĐIỀU KHIỂN THỦ CÔNG làm cấu hình
    // chính thức thay vì phụ thuộc tự động (driver OV5640 trong thư viện
    // esp32-camera hỗ trợ AEC/AGC tự động không đầy đủ bằng OV2640/OV3660).
    //
    // Mức tối đa (1200/30) bị cháy sáng nặng - hạ xuống còn khoảng 1/4 làm
    // điểm dò đầu tiên. ĐỔI CẢ 2 SỐ CÙNG LÚC Ở BƯỚC NÀY vì đang dò khoảng
    // giá trị rộng (chưa cần tách riêng từng biến) - một khi đã gần đúng
    // mức sáng mong muốn, mới tách riêng aec_value (ảnh hưởng blur khi xe
    // di chuyển) và agc_gain (ảnh hưởng nhiễu/sọc) để tinh chỉnh từng cái.
    s->set_aec_value(s, 300);     // 1/4 mức tối đa 1200
    s->set_agc_gain(s, 14);       // tăng từ 8 lên 14 để sáng hơn - CHỈ đổi số này, giữ nguyên aec_value=300
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    // ĐÃ XÁC NHẬN QUA TEST CHỮ VIẾT TAY (không phải đoán qua ảnh phòng):
    // ảnh raw (vflip=0,hmirror=0) đúng chiều trái-phải và đúng góc xoay,
    // chỉ bị lật trên-dưới - đúng chức năng của vflip. KHÔNG cần xoay 90°
    // (rotate=None ở config.py phía PC) và KHÔNG cần hmirror.
    s->set_hmirror(s, 0);
    s->set_vflip(s, 1);
    Serial.println("OV5640 detected: VGA MJPEG low-heat profile active (quality=20, sharpness/denoise off).");
    return;
  }

  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, -2);
    s->set_contrast(s, 1);
    s->set_saturation(s, -2);
  }

  s->set_framesize(s, CAMERA_FRAME_SIZE);
  s->set_quality(s, CAMERA_JPEG_QUALITY);
}

static void connectWiFi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  WiFi.begin(ssid, password);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected.");
  Serial.print("Camera page: http://");
  Serial.println(WiFi.localIP());
  Serial.print("MJPEG stream: http://");
  Serial.print(WiFi.localIP());
  Serial.println(":81/stream");
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
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
  config.xclk_freq_hz = CAMERA_XCLK_HZ;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = CAMERA_FRAME_SIZE;
  config.jpeg_quality = CAMERA_JPEG_QUALITY;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.fb_count = 2;  // continuous capture, latest frame wins
  } else {
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.fb_count = 1;
    config.frame_size = FRAMESIZE_CIF;  // safer fallback without PSRAM
    config.jpeg_quality = 16;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  applySensorTuning(esp_camera_sensor_get());
  connectWiFi();
  startCameraServer();
  carBridgeBegin();
}

void loop() {
  carBridgeLoop();
  delay(1);
}
