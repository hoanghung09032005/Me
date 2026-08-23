/**
 * @file mode3_follow.c
 * @brief Mode 3: Bám vật thể — 2 cách:
 *   • Siêu âm: STM32 tự quét servo + HC-SR04 (state machine)
 *   • AI: PC gửi error, STM32 chạy P-controller đơn giản
 */

#include "mode3_follow.h"
#include "mode2_internal.h"
#include "hardware.h"

/* ==================================================================
 * THÔNG SỐ CHUNG
 * ================================================================== */
#define FOLLOW_TARGET_DIST_CM       20
#define FOLLOW_DIST_TOLERANCE       5
#define FOLLOW_MAX_DIST_CM          100
#define FOLLOW_MIN_DIST_CM          5

#define FOLLOW_BASE_SPEED           350
#define FOLLOW_TURN_SPEED           300
#define FOLLOW_MAX_PWM              700

#define SCAN_ANGLE_LEFT             60
#define SCAN_ANGLE_CENTER           90
#define SCAN_ANGLE_RIGHT            120

#define SCAN_WAIT_TICKS             4
#define TRACK_WAIT_TICKS            4
#define TRACK_CYCLES_BEFORE_RESCAN  8

/* ==================================================================
 * AI MODE (PC gửi error)
 * ================================================================== */
#define AI_FOLLOW_DEADZONE_X        10
#define AI_FOLLOW_DEADZONE_Y        10
#define AI_FOLLOW_KP_X              8
#define AI_FOLLOW_KP_Y              4

/* ==================================================================
 * STATE MACHINE (Siêu âm)
 * ================================================================== */
typedef enum {
    FOLLOW_SCAN_SET_ANGLE,
    FOLLOW_SCAN_WAIT,
    FOLLOW_SCAN_READ,
    FOLLOW_TRACK_SET_ANGLE,
    FOLLOW_TRACK_WAIT,
    FOLLOW_TRACK_READ
} FollowState;

static const int scan_angles[] = {SCAN_ANGLE_LEFT, SCAN_ANGLE_CENTER, SCAN_ANGLE_RIGHT};
#define SCAN_COUNT  (sizeof(scan_angles) / sizeof(scan_angles[0]))

/* ==================================================================
 * BIẾN NỘI BỘ
 * ================================================================== */
static volatile uint8_t follow_use_ai = 0;   /* 0 = siêu âm, 1 = AI */
static volatile int ai_x_err = 0;
static volatile int ai_y_err = 0;

/* Siêu âm state */
static volatile FollowState follow_state = FOLLOW_SCAN_SET_ANGLE;
static volatile int scan_idx = 0;
static volatile int best_angle = 90;
static volatile int best_distance = 999;
static volatile int wait_tick_counter = 0;
static volatile int track_cycle_count = 0;

extern volatile int error;
extern volatile int log_pwm_l;
extern volatile int log_pwm_r;
extern volatile int telemetry_ready;

#define FOLLOW_TELE_DIV  5
static volatile int follow_tele_count = 0;

/* ================================================================== */

void Mode3_Follow_Init(void)
{
    follow_use_ai = 0;
    ai_x_err = 0;
    ai_y_err = 0;

    follow_state = FOLLOW_SCAN_SET_ANGLE;
    scan_idx = 0;
    best_angle = 90;
    best_distance = 999;
    wait_tick_counter = 0;
    track_cycle_count = 0;
    follow_tele_count = 0;
    error = 0;
    Servo_SetAngle(90);
    Set_Motors_Compensated(0, 0);
}

void Mode3_Follow_SetError(int x_err, int y_err)
{
    follow_use_ai = 1;
    if (x_err < -100) x_err = -100;
    if (x_err > 100)  x_err = 100;
    if (y_err < -100) y_err = -100;
    if (y_err > 100)  y_err = 100;
    ai_x_err = x_err;
    ai_y_err = y_err;
}

/* ------------------------------------------------------------------ */

static void Update_Ultrasonic(int *pwm_l, int *pwm_r)
{
    switch (follow_state) {

        case FOLLOW_SCAN_SET_ANGLE:
            Servo_SetAngle(scan_angles[scan_idx]);
            HCSR04_RequestMeasurement();
            wait_tick_counter = 0;
            follow_state = FOLLOW_SCAN_WAIT;
            break;

        case FOLLOW_SCAN_WAIT:
            wait_tick_counter++;
            if (wait_tick_counter >= SCAN_WAIT_TICKS) {
                follow_state = FOLLOW_SCAN_READ;
            }
            break;

        case FOLLOW_SCAN_READ: {
            int dist = HCSR04_GetDistance_cm_x10() / 10;

            if (dist > 0 && dist < best_distance) {
                best_distance = dist;
                best_angle = scan_angles[scan_idx];
            }

            scan_idx++;
            if (scan_idx >= (int)SCAN_COUNT) {
                /* Đã quét xong 3 góc */
                scan_idx = 0;
                if (best_distance < FOLLOW_MAX_DIST_CM && best_distance > 0) {
                    /* Tìm thấy vật -> chuyển sang track */
                    track_cycle_count = 0;
                    follow_state = FOLLOW_TRACK_SET_ANGLE;
                } else {
                    /* Không thấy vật -> quét lại từ đầu */
                    best_distance = 999;
                    follow_state = FOLLOW_SCAN_SET_ANGLE;
                    *pwm_l = 0;
                    *pwm_r = 0;
                }
            } else {
                /* Quét góc tiếp theo */
                follow_state = FOLLOW_SCAN_SET_ANGLE;
            }
            break;
        }

        case FOLLOW_TRACK_SET_ANGLE:
            Servo_SetAngle(best_angle);
            HCSR04_RequestMeasurement();
            wait_tick_counter = 0;
            follow_state = FOLLOW_TRACK_WAIT;
            break;

        case FOLLOW_TRACK_WAIT:
            wait_tick_counter++;
            if (wait_tick_counter >= TRACK_WAIT_TICKS) {
                follow_state = FOLLOW_TRACK_READ;
            }
            break;

        case FOLLOW_TRACK_READ: {
            int dist = HCSR04_GetDistance_cm_x10() / 10;

            /* Kiểm tra mất vật */
            if (dist <= 0 || dist > FOLLOW_MAX_DIST_CM) {
                best_distance = 999;
                follow_state = FOLLOW_SCAN_SET_ANGLE;
                *pwm_l = 0;
                *pwm_r = 0;
                break;
            }

            /* ---- TÍNH TOÁN ĐIỀU KHIỂN ---- */
            int angle_error = best_angle - 90;
            error = angle_error;

            int turn = 0;
            int speed = 0;

            if (angle_error < -10) {
                turn = -FOLLOW_TURN_SPEED;
            } else if (angle_error > 10) {
                turn = FOLLOW_TURN_SPEED;
            }

            if (dist < FOLLOW_MIN_DIST_CM) {
                speed = -FOLLOW_BASE_SPEED;
            } else if (dist < (FOLLOW_TARGET_DIST_CM - FOLLOW_DIST_TOLERANCE)) {
                speed = -(FOLLOW_BASE_SPEED / 2);
            } else if (dist > (FOLLOW_TARGET_DIST_CM + FOLLOW_DIST_TOLERANCE)) {
                speed = FOLLOW_BASE_SPEED;
            } else {
                speed = 0;
            }

            *pwm_l = speed + turn;
            *pwm_r = speed - turn;

            /* Sau N chu kỳ track, quét lại */
            track_cycle_count++;
            if (track_cycle_count >= TRACK_CYCLES_BEFORE_RESCAN) {
                best_distance = 999;
                follow_state = FOLLOW_SCAN_SET_ANGLE;
            } else {
                follow_state = FOLLOW_TRACK_SET_ANGLE;
            }
            break;
        }

        default:
            follow_state = FOLLOW_SCAN_SET_ANGLE;
            break;
    }
}

static void Update_AI(int *pwm_l, int *pwm_r)
{
    int abs_x = (ai_x_err < 0) ? -ai_x_err : ai_x_err;
    int abs_y = (ai_y_err < 0) ? -ai_y_err : ai_y_err;

    /* Lưu error để telemetry hiển thị */
    error = ai_x_err;

    if (abs_x > AI_FOLLOW_DEADZONE_X) {
        /* Vật lệch ngang -> xoay tại chỗ */
        int turn = (ai_x_err * FOLLOW_TURN_SPEED) / 100;
        if (turn > FOLLOW_TURN_SPEED)  turn = FOLLOW_TURN_SPEED;
        if (turn < -FOLLOW_TURN_SPEED) turn = -FOLLOW_TURN_SPEED;

        *pwm_l = turn;
        *pwm_r = -turn;
    } else {
        /* Xử lý lệch dọc (tiến/lùi) */
        if (abs_y > AI_FOLLOW_DEADZONE_Y) {
            int speed = FOLLOW_BASE_SPEED - (ai_y_err * AI_FOLLOW_KP_Y);
            if (speed > FOLLOW_MAX_PWM) speed = FOLLOW_MAX_PWM;
            if (speed < -FOLLOW_MAX_PWM) speed = -FOLLOW_MAX_PWM;
            if (speed > 0 && speed < 200) speed = 200;
            if (speed < 0 && speed > -200) speed = -200;

            *pwm_l = speed;
            *pwm_r = speed;
        } else {
            *pwm_l = 0;
            *pwm_r = 0;
        }
    }
}

/* ================================================================== */

void Mode3_Follow_Update(void)
{
    int pwm_l = 0;
    int pwm_r = 0;

    if (follow_use_ai) {
        Update_AI(&pwm_l, &pwm_r);
    } else {
        Update_Ultrasonic(&pwm_l, &pwm_r);
    }

    /* Giới hạn PWM */
    if (pwm_l > FOLLOW_MAX_PWM)  pwm_l = FOLLOW_MAX_PWM;
    if (pwm_l < -FOLLOW_MAX_PWM) pwm_l = -FOLLOW_MAX_PWM;
    if (pwm_r > FOLLOW_MAX_PWM)  pwm_r = FOLLOW_MAX_PWM;
    if (pwm_r < -FOLLOW_MAX_PWM) pwm_r = -FOLLOW_MAX_PWM;

    Set_Motors_Compensated(pwm_l, pwm_r);

    /* Telemetry */
    if (++follow_tele_count >= FOLLOW_TELE_DIV) {
        follow_tele_count = 0;
        log_pwm_l = pwm_l;
        log_pwm_r = pwm_r;
        telemetry_ready = 1;
    }
}
