/**
 * @file mode2_obstacle.c
 * @brief File ĐIỀU PHỐI của Mode 2 - API công khai (mode2_obstacle.h)
 * KHÔNG đổi gì so với trước, nên main.c không cần sửa.
 *
 * Toàn bộ logic thật đã tách ra 2 module con:
 *   - mode2_line_pid.c/h  : bám line PID + xử lý góc vuông 90 độ
 *   - mode2_avoid.c/h     : né vật cản (state machine)
 * File này chỉ còn 3 việc: khởi tạo cả 2 module, xuất PWM (bù trừ +
 * telemetry, dùng CHUNG cho cả 2 module qua mode2_internal.h), và mỗi
 * tick quyết định gọi module nào.
 */

#include "mode2_obstacle.h"
#include "mode2_internal.h"
#include "mode2_line_pid.h"
#include "mode2_avoid.h"
#include "hardware.h"

#define AUTO_TELE_DIV             5
static volatile int auto_tele_count = 0;
extern volatile int log_pwm_l;
extern volatile int log_pwm_r;
extern volatile int telemetry_ready;

/* Hàm xuất PWM DÙNG CHUNG cho cả mode2_line_pid.c lẫn mode2_avoid.c (khai
 * báo trong mode2_internal.h) - đặt ở đây vì AUTO_TELE_DIV/auto_tele_count
 * là quy ước throttle telemetry CHUNG cho toàn Mode 2, không thuộc riêng
 * PID hay riêng né vật cản - tránh mỗi module tự định nghĩa 1 bản khác
 * nhau (đúng kiểu lỗi "protocol mismatch giữa các file" đã từng gặp). */
void Set_Motors_Compensated(int pwm_l, int pwm_r)
{
    int compensated_pwm_r = (pwm_r * MOTOR_RIGHT_COMPENSATION_PCT) / 100;

    Set_Motor_Outputs(pwm_l, compensated_pwm_r);

    log_pwm_l = pwm_l;
    log_pwm_r = compensated_pwm_r;
    if (++auto_tele_count >= AUTO_TELE_DIV) {
        auto_tele_count = 0;
        telemetry_ready = 1;
    }
}

void Mode2_Obstacle_SetSpeedPercent(uint8_t speed_pct)
{
    LinePID_SetBaseSpeedPercent(speed_pct);
}

void Mode2_Obstacle_Init(void)
{
    auto_tele_count = 0;
    LinePID_Init();
    Avoid_Init();
}

void Mode2_Obstacle_Update(uint8_t raw_state, uint8_t side_left, uint8_t side_right)
{
    if (LinePID_GetBaseSpeed() <= 0) {
        Set_Motors_Compensated(0, 0);
        return;
    }

    /* Avoid_Update() tự quyết định có chiếm quyền điều khiển tick này hay
     * không (đang né, hoặc vừa phát hiện vật cản và bắt đầu né) - trả về
     * 1 nghĩa là nó đã tự gọi Set_Motors_Compensated() rồi, không được
     * chạy PID chồng lên trong CÙNG 1 tick. */
    if (Avoid_Update(raw_state, side_left, side_right)) {
        return;
    }

    LinePID_Run(raw_state, side_left, side_right);
}
