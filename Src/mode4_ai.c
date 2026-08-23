/**
 * @file mode4_ai.c
 * @brief Mode 4: AI nhận diện line qua video.
 *        Nhận error từ PC, chạy PID giống hệt mode2_line_pid.c
 *        nhưng error đến từ camera thay vì 5 mắt IR.
 */

#include "mode4_ai.h"
#include "mode2_internal.h"
#include "hardware.h"

/* ==================================================================
 * THÔNG SỐ PID (giống mode2_line_pid.c, có thể tune riêng)
 * ================================================================== */
static float Kp = 40.0f;
static float Ki = 0.0f;
static float Kd = 120.0f;

#define AI_BASE_SPEED             400
#define AI_MIN_BASE_SPEED         400
#define AI_MAX_BASE_SPEED         600

#define AI_MAX_LOST_CYCLES        30      /* Dài hơn Mode 2 vì PC chậm hơn IR */
#define AI_PIVOT_SPEED            350

/* ==================================================================
 * TRẠNG THÁI NỘI BỘ
 * ================================================================== */
static volatile float I_term = 0.0f;
static volatile int ai_error = 0;
static volatile int last_ai_error = 0;
static volatile int lost_count = 0;
static volatile int line_detected = 0;
static volatile int last_turn_direction = 0;
static volatile int base_speed = AI_BASE_SPEED;

extern volatile int error;
extern volatile int last_error;
extern volatile int log_pwm_l;
extern volatile int log_pwm_r;
extern volatile int telemetry_ready;

#define AI_TELE_DIV               5
static volatile int ai_tele_count = 0;

void Mode4_AI_Init(void)
{
    I_term = 0.0f;
    ai_error = 0;
    last_ai_error = 0;
    lost_count = 0;
    line_detected = 0;
    last_turn_direction = 0;
    base_speed = AI_BASE_SPEED;
    ai_tele_count = 0;
    Set_Motors_Compensated(0, 0);
}

void Mode4_AI_SetError(int error_val)
{
    /* FIX BUG 2: Phân biệt sentinel "không thấy line" với error hợp lệ.
     * Chỉ set line_detected khi nhận giá trị thực sự (khác sentinel).
     * Sentinel 9999 vẫn được lưu vào ai_error để Update() xử lý riêng. */
    ai_error = error_val;
    if (ai_error != AI_NO_LINE_SENTINEL) {
        line_detected = 1;
    }
}

void Mode4_AI_Update(void)
{
    if (!line_detected) {
        /* Chờ lệnh đầu tiên hợp lệ từ PC */
        Set_Motors_Compensated(0, 0);
        return;
    }

    /* FIX BUG 2: Kiểm tra sentinel TRƯỚC khi chia /100.
     * Sentinel 9999 nằm ngoài dải hợp lệ - dùng để báo "PC không thấy line"
     * thay vì nhầm lẫn với error=0 (đi đúng giữa). */
    if (ai_error == AI_NO_LINE_SENTINEL) {
        lost_count++;
        if (lost_count > AI_MAX_LOST_CYCLES) {
            Set_Motors_Compensated(0, 0);
            I_term = 0.0f;
            line_detected = 0;
            lost_count = 0;
            last_turn_direction = 0;
            return;
        }
        /* Quay tại chỗ theo hướng cuối cùng */
        int pwm_l = (last_turn_direction >= 0) ? AI_PIVOT_SPEED : -AI_PIVOT_SPEED;
        int pwm_r = -pwm_l;
        Set_Motors_Compensated(pwm_l, pwm_r);
        return;
    }

    /* error từ PC: -400..400 tương ứng -4.00..4.00 */
    int e = ai_error / 100;

    if (e == 0 && ai_error != 0) {
        /* Trường hợp ai_error nhỏ hơn 100 nhưng khác 0 */
        e = (ai_error > 0) ? 1 : -1;
    }

    /* Cập nhật biến toàn cục để telemetry hiển thị đúng */
    error = e;
    last_error = last_ai_error;

    lost_count = 0;

    /* ---- PID CHẠY THẲNG ---- */
    if (e > 0) last_turn_direction = 1;
    else if (e < 0) last_turn_direction = -1;

    float p_term = Kp * e;
    I_term += Ki * e;
    if (I_term > 200.0f) I_term = 200.0f;
    else if (I_term < -200.0f) I_term = -200.0f;

    float d_term = Kd * (e - last_ai_error);
    last_ai_error = e;

    int pid_value = (int)(p_term + I_term + d_term);
    int abs_e = (e >= 0) ? e : -e;

    /* Phanh tự động khi vào cua */
    int base_pwm = base_speed - (abs_e * 25);
    if (base_pwm < AI_MIN_BASE_SPEED) base_pwm = AI_MIN_BASE_SPEED;

    int pwm_l = base_pwm + pid_value;
    int pwm_r = base_pwm - pid_value;

    if (pwm_l > 999)  pwm_l = 999;
    if (pwm_l < -999) pwm_l = -999;
    if (pwm_r > 999)  pwm_r = 999;
    if (pwm_r < -999) pwm_r = -999;

    Set_Motors_Compensated(pwm_l, pwm_r);

    if (++ai_tele_count >= AI_TELE_DIV) {
        ai_tele_count = 0;
        telemetry_ready = 1;
    }
}
