#include "mode2.h"
#include "hardware.h"

float Kp = 42.0f;
float Ki = 0.0f;
float Kd = 120.0f;
int   Base_Speed = 350;

#define MIN_SPEED        250
// Số chu kỳ quét không thấy đường trước khi quyết định dừng (chống nhiễu góc vuông)
#define MAX_LOST_CYCLES  15

extern volatile int error, last_error, log_pwm_l, log_pwm_r, telemetry_ready;

volatile float   I_term = 0.0f;
volatile int     lost_count = 0;
volatile int     line_detected = 0;

void Mode2_Init(void) {
    line_detected = 0;
    I_term = 0.0f;
    error = last_error = 0;
    lost_count = 0;
    Set_Motor_Outputs(0, 0);
}

void Mode2_Update(uint8_t raw_state) {
    int leds_off = 0;
    for (int i = 0; i < 5; i++) {
        if ((raw_state >> i) & 1u) leds_off++;
    }

    // 1. CHỈ CHẠY KHI PHÁT HIỆN ĐƯỜNG (Gặp line mới chạy)
    if (!line_detected) {
        if (leds_off >= 1 && leds_off <= 3) {
            line_detected = 1;
        } else {
            Set_Motor_Outputs(0, 0);
            return;
        }
    }

    // 2. XỬ LÝ TÍN HIỆU ĐƯỜNG LIỀN MẠCH, GÓC VUÔNG VÀ NHẤC XE
    if (leds_off == 0 || leds_off == 5) {
        // Trạng thái 0 (toàn trắng) hoặc 5 (toàn đen)
        lost_count++;

        // Nếu tình trạng này kéo dài -> Xe bị nhấc lên hoặc văng hẳn khỏi sa hình -> DỪNG
        if (lost_count > MAX_LOST_CYCLES) {
            Set_Motor_Outputs(0, 0);
            error = last_error = I_term = line_detected = 0;
            log_pwm_l = log_pwm_r = 0;
            telemetry_ready = 1;
            return;
        }

        // Nếu mới mất vạch (toàn trắng) do chạy nhanh văng lố ở góc vuông 90 độ
        // Dựa vào lịch sử lỗi cũ để ép xe bẻ lái gắt tìm lại vạch
        if (leds_off == 0) {
            if (last_error > 0) error = 5;       // Ép cua phải gắt
            else if (last_error < 0) error = -5; // Ép cua trái gắt
        }
    }
    else {
        // Đã thấy lại đường, reset bộ đếm mất vạch
        lost_count = 0;

        // Tính trọng số lỗi từ thanh cảm biến
        switch (raw_state) {
            case 0x04: error =  0; break;
            case 0x0C: error =  1; break;   case 0x08: error =  2; break;
            case 0x18: error =  3; break;   case 0x10: error =  4; break;
            case 0x06: error = -1; break;   case 0x02: error = -2; break;
            case 0x03: error = -3; break;   case 0x01: error = -4; break;
            default:   error = last_error;  break;
        }
    }

    // 3. TÍNH TOÁN PID CƠ BẢN
    float P = Kp * error;

    I_term += Ki * error;
    // Chống tích lũy I (Windup)
    if (I_term > 200.0f) I_term = 200.0f; else if (I_term < -200.0f) I_term = -200.0f;

    float D = Kd * (error - last_error);
    last_error = error;

    int PID_val = (int)(P + I_term + D);

    // 4. BÙ GA TỰ ĐỘNG (Phanh hãm tốc khi bẻ cua)
    int abs_error = (error > 0) ? error : -error;
    int base_pwm = Base_Speed - (abs_error * 25);
    base_pwm = (base_pwm < MIN_SPEED) ? MIN_SPEED : base_pwm;

    // 5. TRỘN XUNG VÀ XUẤT RA ĐỘNG CƠ
    int pwm_l = base_pwm + PID_val;
    int pwm_r = base_pwm - PID_val;

    // Giới hạn giá trị PWM an toàn để motor không bị bão hòa
    if (pwm_l > 999) pwm_l = 999; else if (pwm_l < -999) pwm_l = -999;
    if (pwm_r > 999) pwm_r = 999; else if (pwm_r < -999) pwm_r = -999;

    Set_Motor_Outputs(pwm_l, pwm_r);

    // Bắn log Telemetry lên giao diện PC
    log_pwm_l = pwm_l; log_pwm_r = pwm_r;
    telemetry_ready = 1;
}
