/**
 * @file mode2_avoid.c
 * @brief Né vật cản (State Machine): quét siêu âm, lách qua, quay lại bám line.
 *
 * Tách ra từ mode2_obstacle.c. File này KHÔNG đụng trực tiếp vào biến nội
 * bộ của mode2_line_pid.c - khi vừa bám lại line xong, gọi
 * LinePID_NotifyLineReacquired() thay vì tự ý set line_detected/error/...
 */

#include "mode2_avoid.h"
#include "mode2_internal.h"
#include "mode2_line_pid.h"
#include "hardware.h"

/* ==================================================================
 * THÔNG SỐ NÉ VẬT CẢN
 * ================================================================== */
#define OBSTACLE_DISTANCE_X10    180     /* Ngưỡng né: 18cm */
#define AVOID_SAFETY_TIMEOUT     400     /* Timeout 4s: Tránh xe chạy lố khỏi làn */

#define AVOID_PIVOT_SPEED         500    /* PWM xoay lách ra/quặt vào */
#define AVOID_ALIGN_SPEED         550    /* PWM bơm mạnh để nắn thẳng, chống đì */
#define AVOID_SPEED               350    /* PWM tiến thẳng khi né */

#define AVOID_PIVOT_OUT_CYCLES    15     /* Xoay lách ra (150ms) */
#define AVOID_FORWARD_SIDE_CYCLES 150    /* Tiến song song vật cản (1.5s) */
#define AVOID_PIVOT_BACK_CYCLES   30     /* Quặt hướng về line (300ms) */
#define AVOID_REACQUIRE_MIN_TICKS 15     /* Bắt buộc đi thẳng tối thiểu 150ms trước khi nắn */
#define AVOID_WAIT_CYCLES         100    /* Dừng tĩnh dập quán tính (1s) */

/* ==================================================================
 * THÔNG SỐ MÙ SIÊU ÂM (COOLDOWN)
 * ================================================================== */
#define AVOID_COOLDOWN_TICKS      150    /* Điếc siêu âm 1.5s sau khi né xong để tránh đuôi xe */
static volatile int avoid_cooldown_timer = 0;

#define SERVO_CENTER_DEG          90
#define SERVO_LEFT_DEG            170
#define SERVO_RIGHT_DEG           10

#define SCAN_SETTLE_TICKS         100
#define SCAN_HCSR04_WAIT_TICKS    15
#define SCAN_LEFT_TRIGGER_TICK   (SCAN_SETTLE_TICKS)
#define SCAN_LEFT_READ_TICK      (SCAN_LEFT_TRIGGER_TICK + SCAN_HCSR04_WAIT_TICKS)
#define SCAN_RIGHT_TRIGGER_TICK  (SCAN_LEFT_READ_TICK + SCAN_SETTLE_TICKS)
#define SCAN_RIGHT_READ_TICK     (SCAN_RIGHT_TRIGGER_TICK + SCAN_HCSR04_WAIT_TICKS)
#define SCAN_CENTER_SETTLE_TICK  (SCAN_RIGHT_READ_TICK + SCAN_SETTLE_TICKS)

#define FRONT_PING_DIV            4

/* Ghi telemetry khoảng cách - biến toàn cục dùng chung với gói LOG trong
 * main.c (giữ nguyên như bản gốc, KHÔNG chuyển thành hàm getter vì đây là
 * kiểu telemetry throttle theo tick, không phải trạng thái điều khiển). */
extern volatile int log_distance_cm_x10;

typedef enum {
    AVOID_NONE = 0,
    AVOID_SCAN,
    AVOID_PIVOT_OUT,
    AVOID_WAIT1,
    AVOID_FORWARD_SIDE,
    AVOID_WAIT2,
    AVOID_PIVOT_BACK,
    AVOID_WAIT3,
    AVOID_FORWARD_REACQUIRE,
    AVOID_ALIGN_LINE,
    AVOID_WAIT4
} AvoidState_t;

static volatile AvoidState_t avoid_state = AVOID_NONE;
static volatile int avoid_timer = 0;
static volatile int avoid_turn_dir = 1;
static volatile int align_dir = 0; /* Lưu hướng xoay nắn thẳng */
static int ping_counter = 0;
static int16_t last_distance_cm_x10 = -1;
static int16_t scan_distance_left_cm_x10 = -1;
static int16_t scan_distance_right_cm_x10 = -1;
static volatile int reacquire_confirm_count = 0;

void Avoid_Init(void)
{
    avoid_state = AVOID_NONE; avoid_timer = 0;
    avoid_turn_dir = 1; align_dir = 0; avoid_cooldown_timer = 0; reacquire_confirm_count = 0;
    ping_counter = 0;
    last_distance_cm_x10 = -1; scan_distance_left_cm_x10 = -1; scan_distance_right_cm_x10 = -1;
    log_distance_cm_x10 = HCSR04_GetDistance_cm_x10();
    Servo_SetAngle(SERVO_CENTER_DEG);
    Set_Motors_Compensated(0, 0);
}

int Avoid_Update(uint8_t raw_state, uint8_t side_left, uint8_t side_right)
{
    int16_t newest_distance_cm_x10 = HCSR04_GetDistance_cm_x10();
    if (newest_distance_cm_x10 > 0) last_distance_cm_x10 = newest_distance_cm_x10;
    else if (newest_distance_cm_x10 < 0) last_distance_cm_x10 = -1;
    log_distance_cm_x10 = newest_distance_cm_x10;

    /* Chưa đang né: kiểm tra điều kiện kích hoạt (ping định kỳ + ngưỡng
     * khoảng cách + hết thời gian mù). Nếu không kích hoạt, trả về 0 để
     * người gọi tự chạy PID bám line như bình thường. */
    if (avoid_state == AVOID_NONE) {
        if (avoid_cooldown_timer > 0) avoid_cooldown_timer--;

        if (++ping_counter >= FRONT_PING_DIV) {
            ping_counter = 0;
            HCSR04_RequestMeasurement();
        }

        if (avoid_cooldown_timer == 0 &&
            last_distance_cm_x10 > 0 &&
            last_distance_cm_x10 < OBSTACLE_DISTANCE_X10) {
            Set_Motors_Compensated(0, 0);
            avoid_state = AVOID_SCAN;
            avoid_timer = 0;
            return 1;
        }

        return 0;
    }

    /* STATE MACHINE NÉ VẬT CẢN (OBSTACLE AVOIDANCE) */
    switch (avoid_state) {
        case AVOID_SCAN:
            if (avoid_timer == 0) Servo_SetAngle(SERVO_LEFT_DEG);
            else if (avoid_timer == SCAN_LEFT_TRIGGER_TICK) HCSR04_RequestMeasurement();
            else if (avoid_timer == SCAN_LEFT_READ_TICK) {
                scan_distance_left_cm_x10 = HCSR04_GetDistance_cm_x10();
                Servo_SetAngle(SERVO_RIGHT_DEG);
            } else if (avoid_timer == SCAN_RIGHT_TRIGGER_TICK) HCSR04_RequestMeasurement();
            else if (avoid_timer == SCAN_RIGHT_READ_TICK) {
                scan_distance_right_cm_x10 = HCSR04_GetDistance_cm_x10();
                Servo_SetAngle(SERVO_CENTER_DEG);
            } else if (avoid_timer == SCAN_CENTER_SETTLE_TICK) {
                if (scan_distance_left_cm_x10 <= 0) avoid_turn_dir = 1;
                else if (scan_distance_right_cm_x10 <= 0) avoid_turn_dir = -1;
                else avoid_turn_dir = (scan_distance_right_cm_x10 > scan_distance_left_cm_x10) ? 1 : -1;

                avoid_state = AVOID_PIVOT_OUT;
                avoid_timer = 0;
                break;
            }
            avoid_timer++;
            break;

        case AVOID_PIVOT_OUT:
            /* Lách ra ngoài */
            {
                int pwm_l = (avoid_turn_dir > 0) ?  AVOID_PIVOT_SPEED : -AVOID_PIVOT_SPEED;
                Set_Motors_Compensated(pwm_l, -pwm_l);
                if (++avoid_timer >= AVOID_PIVOT_OUT_CYCLES) { avoid_state = AVOID_WAIT1; avoid_timer = 0; }
            }
            break;

        case AVOID_WAIT1:
            Set_Motors_Compensated(0, 0);
            if (++avoid_timer >= AVOID_WAIT_CYCLES) { avoid_state = AVOID_FORWARD_SIDE; avoid_timer = 0; }
            break;

        case AVOID_FORWARD_SIDE:
            /* Đi thẳng song song vật cản */
            Set_Motors_Compensated(AVOID_SPEED, AVOID_SPEED);
            if (++avoid_timer >= AVOID_FORWARD_SIDE_CYCLES) { avoid_state = AVOID_WAIT2; avoid_timer = 0; }
            break;

        case AVOID_WAIT2:
            Set_Motors_Compensated(0, 0);
            if (++avoid_timer >= AVOID_WAIT_CYCLES) { avoid_state = AVOID_PIVOT_BACK; avoid_timer = 0; }
            break;

        case AVOID_PIVOT_BACK:
            /* Quặt chéo đầu xe về lại phía line */
            {
                int pwm_l = (avoid_turn_dir > 0) ? -AVOID_PIVOT_SPEED : AVOID_PIVOT_SPEED;
                Set_Motors_Compensated(pwm_l, -pwm_l);
                if (++avoid_timer >= AVOID_PIVOT_BACK_CYCLES) {
                    avoid_state = AVOID_WAIT3;
                    avoid_timer = 0;
                }
            }
            break;

        case AVOID_WAIT3:
            Set_Motors_Compensated(0, 0);
            if (++avoid_timer >= AVOID_WAIT_CYCLES) {
                avoid_state = AVOID_FORWARD_REACQUIRE;
                avoid_timer = 0;
                reacquire_confirm_count = 0;
            }
            break;

        case AVOID_FORWARD_REACQUIRE:
            /* Chạy thẳng về phía line. Dừng và chốt hướng nắn (align_dir) khi 5 mắt giữa chạm vạch */
            {
                int sensor_high_count = 0;
                Set_Motors_Compensated(AVOID_SPEED, AVOID_SPEED);

                for (int i = 0; i < 5; i++) {
                    if ((raw_state >> i) & 1U) sensor_high_count++;
                }

                if (sensor_high_count >= 1) {
                    if (++reacquire_confirm_count >= 2 && avoid_timer >= AVOID_REACQUIRE_MIN_TICKS) {
                        /* Chốt hướng nắn thẳng: Xoay ngược ra xa mắt biên chạm trước để triệt góc chéo */
                        if (side_right && !side_left) align_dir = -1;
                        else if (side_left && !side_right) align_dir = 1;
                        else align_dir = (avoid_turn_dir > 0) ? -1 : 1;

                        avoid_state = AVOID_ALIGN_LINE;
                        avoid_timer = 0;
                        reacquire_confirm_count = 0;
                    }
                } else {
                    reacquire_confirm_count = 0;
                }

                /* Hết thời gian an toàn mà không thấy vạch -> Thoát */
                if (++avoid_timer >= AVOID_SAFETY_TIMEOUT) {
                    avoid_state = AVOID_NONE; avoid_timer = 0;
                    avoid_cooldown_timer = AVOID_COOLDOWN_TICKS; last_distance_cm_x10 = -1;
                }
            }
            break;

        case AVOID_ALIGN_LINE:
            /* Nắn thẳng xe tại chỗ (Snap). Dùng AVOID_ALIGN_SPEED bơm mạnh chống ma sát tĩnh */
            {
                int pwm_l = (align_dir > 0) ? AVOID_ALIGN_SPEED : -AVOID_ALIGN_SPEED;
                Set_Motors_Compensated(pwm_l, -pwm_l);

                /* Xong khi mắt giữa ngậm line */
                if ((raw_state & 0x04) || ++avoid_timer >= AVOID_SAFETY_TIMEOUT) {
                    avoid_state = AVOID_WAIT4;
                    avoid_timer = 0;
                    LinePID_NotifyLineReacquired();
                    avoid_cooldown_timer = AVOID_COOLDOWN_TICKS;
                    last_distance_cm_x10 = -1;
                }
            }
            break;

        case AVOID_WAIT4:
            Set_Motors_Compensated(0, 0);
            if (++avoid_timer >= AVOID_WAIT_CYCLES) {
                avoid_state = AVOID_NONE;
                avoid_timer = 0;
            }
            break;

        default:
            avoid_state = AVOID_NONE; avoid_timer = 0;
            Set_Motors_Compensated(0, 0);
            break;
    }

    return 1;
}
