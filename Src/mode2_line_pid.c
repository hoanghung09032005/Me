/**
 * @file mode2_line_pid.c
 * @brief Bám line (PID) + xử lý góc vuông 90 độ tại giao lộ.
 *
 * Tách ra từ mode2_obstacle.c (bản gốc gộp chung cả PID + né vật cản trong
 * 1 file dài) để dễ sửa/quản lý hơn. File này KHÔNG biết gì về né vật cản
 * (mode2_avoid.c/h) - chỉ có 1 điểm giao tiếp NGƯỢC LẠI duy nhất:
 * mode2_avoid.c gọi LinePID_NotifyLineReacquired() khi vừa bám lại line
 * xong sau khi né. Không có chiều ngược lại (file này không gọi gì từ
 * mode2_avoid.c cả).
 */

#include "mode2_line_pid.h"
#include "mode2_internal.h"
#include "hardware.h"

/* ==================================================================
 * THÔNG SỐ PID BÁM LINE
 * ================================================================== */
static float Kp = 40.0f;
static float Ki = 0.0f;
static float Kd = 120.0f;

#define AUTO_MIN_BASE_SPEED      400
#define AUTO_MAX_BASE_SPEED      600
#define DEFAULT_BASE_SPEED       400

#define MAX_LOST_CYCLES          15      /* Ngưỡng timeout báo mất line (150ms) */
#define PIVOT_SPEED              350     /* Tốc độ xoay tìm line khi bị mất */

/* ==================================================================
 * THÔNG SỐ BẮT GÓC VUÔNG 90 ĐỘ
 * ================================================================== */
#define CORNER_CONFIRM_TICKS       3     /* Lọc nhiễu mắt biên (cần 3 nhịp) */
#define CORNER_MIN_ADVANCE_TICKS   5     /* Chờ tiến lên tối thiểu 50ms rồi mới bẻ */
#define CORNER_MAX_ADVANCE_TICKS   20    /* Quá 200ms ép bẻ góc luôn */

#define CORNER_TURN_TICKS         60
#define CORNER_PIVOT_SPEED        750

/* Số tick LIÊN TỤC tối thiểu mà sensor_high_count >= 3 phải giữ nguyên để
 * được coi là "vạch ngang rộng thật" (ngã tư), thay vì chỉ là 1 tick quét
 * chéo thoáng qua của khúc cua chữ S - xem giải thích đầy đủ ở TẦNG 3 bên
 * dưới. 2 tick = 20ms, cùng bậc với các ngưỡng debounce khác trong file
 * này (CORNER_CONFIRM_TICKS, AIRBORNE_CONFIRM_TICKS). */
#define CORNER_WIDE_PERSIST_TICKS 2
static volatile int wide_persist_count = 0;

#define AIRBORNE_CONFIRM_TICKS    3
static volatile int airborne_count = 0;

typedef enum {
    CORNER_NONE = 0,
    CORNER_ARMED,
    CORNER_TURNING
} CornerState_t;

static volatile CornerState_t corner_state = CORNER_NONE;
static volatile int corner_dir = 0;
static volatile int corner_timer = 0;
static volatile int corner_confirm_count = 0;
static volatile int corner_pending_dir = 0;

extern volatile int error;
extern volatile int last_error;
extern uint8_t Side_EdgeEvent_TakeAndClear(int8_t *out_dir);

static volatile float I_term = 0.0f;
static volatile int lost_count = 0;
static volatile int line_detected = 0;
static volatile int last_turn_direction = 0;
static volatile int Base_Speed = DEFAULT_BASE_SPEED;

void LinePID_SetBaseSpeedPercent(uint8_t speed_pct)
{
    if (speed_pct > 100U) speed_pct = 100U;
    if (speed_pct == 0U) { Base_Speed = 0; return; }
    Base_Speed = AUTO_MIN_BASE_SPEED + ((int)speed_pct * (AUTO_MAX_BASE_SPEED - AUTO_MIN_BASE_SPEED)) / 100;
}

int LinePID_GetBaseSpeed(void)
{
    return Base_Speed;
}

void LinePID_NotifyLineReacquired(void)
{
    line_detected = 1;
    error = 0;
    last_error = 0;
    I_term = 0.0f;
}

void LinePID_Init(void)
{
    line_detected = 0; I_term = 0.0f; error = 0; last_error = 0; lost_count = 0;
    last_turn_direction = 0; airborne_count = 0;
    corner_state = CORNER_NONE; corner_dir = 0; corner_timer = 0;
    corner_confirm_count = 0; corner_pending_dir = 0;
    wide_persist_count = 0;
    (void)Side_EdgeEvent_TakeAndClear(NULL); /* Xả rác EXTI còn sót từ trước */
}

void LinePID_Run(uint8_t raw_state, uint8_t side_left, uint8_t side_right)
{
    int sensor_high_count = 0;

    /* Lọc nhiễu: Xe bị nhấc lên khỏi bàn */
    if (raw_state == 0U && side_left == 0U && side_right == 0U) {
        if (++airborne_count >= AIRBORNE_CONFIRM_TICKS) {
            Set_Motors_Compensated(0, 0);
            line_detected = 0; error = 0; last_error = 0; I_term = 0.0f;
            lost_count = 0; last_turn_direction = 0;
            return;
        }
    } else {
        airborne_count = 0;
    }

    for (int i = 0; i < 5; i++) {
        if ((raw_state >> i) & 1U) sensor_high_count++;
    }

    /* ---- TẦNG 1: THỰC THI BẺ GÓC VUÔNG ---- */
    if (corner_state == CORNER_TURNING) {
        int pwm_l = (corner_dir > 0) ? CORNER_PIVOT_SPEED : -CORNER_PIVOT_SPEED;
        int pwm_r = -pwm_l;

        Set_Motors_Compensated(pwm_l, pwm_r);

        if (sensor_high_count >= 1 && sensor_high_count <= 3) {
            corner_state = CORNER_NONE; corner_timer = 0; error = 0;
            last_error = 0; I_term = 0.0f; lost_count = 0;
            (void)Side_EdgeEvent_TakeAndClear(NULL); /* Xả rác EXTI */
        } else if (++corner_timer >= CORNER_TURN_TICKS) {
            corner_state = CORNER_NONE; corner_timer = 0;
            (void)Side_EdgeEvent_TakeAndClear(NULL);
        }
        return;
    }

    /* ---- TẦNG 2: GÓC VUÔNG CHỜ RẼ (ARMED) ---- */
    if (corner_state == CORNER_ARMED) {
        corner_timer++;
        int wide_line = (sensor_high_count >= 3);

        if (corner_timer >= CORNER_MIN_ADVANCE_TICKS &&
            (wide_line || corner_timer >= CORNER_MAX_ADVANCE_TICKS)) {
            corner_state = CORNER_TURNING;
            corner_timer = 0;
            return;
        }

        /* Tước quyền PID: Ép chạy thẳng qua vạch ngang ngã tư cho tới khi
         * đủ điều kiện bẻ ở trên. */
        Set_Motors_Compensated(Base_Speed, Base_Speed);
        return;
    }

    /* ---- TẦNG 3: BẮT TÍN HIỆU GÓC VUÔNG (EXTI) ---- */
    {
        int8_t edge_dir = 0;
        uint8_t side_edge_happened = Side_EdgeEvent_TakeAndClear(&edge_dir);

        /* SỬA LỖI (quan trọng): bản trước dùng "abs_error <= 5" để cố phân
         * biệt ngã tư thật (vạch ngang rộng) với khúc cua chữ S (line vẫn
         * mảnh, chỉ cắt chéo qua vài mắt). NHƯNG error chỉ có thể nhận giá
         * trị trong khoảng [-4, 4] (xem switch-case TẦNG 5 bên dưới) - điều
         * kiện "<= 5" do đó LUÔN LUÔN ĐÚNG với mọi error hợp lệ, không lọc
         * được gì cả (tương đương hệt như không có điều kiện này). Thêm
         * nữa, error đọc ở đây là giá trị CÒN SÓT LẠI TỪ TICK TRƯỚC - Tầng
         * 5 (tính error từ raw_state) chạy SAU khối này trong cùng 1 lần
         * gọi hàm, nên không phản ánh đúng hình dạng vạch NGAY LÚC NÀY.
         *
         * THAY THẾ bằng tiêu chí bám đúng đặc điểm VẬT LÝ thực sự khác
         * nhau giữa 2 tình huống, dùng wide_persist_count khai báo ở trên:
         *   - Ngã tư thật: vạch ngang RỘNG theo hướng xe đi (bằng bề ngang
         *     đường giao, thường vài cm) -> ở tốc độ hiện tại,
         *     sensor_high_count >= 3 GIỮ NGUYÊN qua NHIỀU tick liên tục
         *     khi xe tiến vào, không chỉ 1 tick rồi biến mất.
         *   - Khúc cua chữ S: line vẫn MẢNH (bề rộng ~1 mắt cảm biến), chỉ
         *     cắt CHÉO qua 3 mắt đúng lúc mảng cảm biến quét ngang đoạn
         *     cong gấp - hiện tượng này chỉ tồn tại ĐÚNG 1 TICK (~10ms)
         *     rồi sensor_high_count rớt ngay về 1-2 khi xe tiếp tục tiến,
         *     KHÔNG duy trì được nhiều tick liên tiếp như vạch ngang thật.
         * Yêu cầu sensor_high_count >= 3 giữ liên tục ít nhất
         * CORNER_WIDE_PERSIST_TICKS tick mới coi là "vạch rộng thật" - lọc
         * đúng hiện tượng "quét chéo thoáng qua 1 tick" của khúc cua, mà
         * không cần thêm cảm biến nào.
         *
         * LƯU Ý: đây vẫn là suy luận dựa trên hình học, CHƯA kiểm chứng
         * trên xe thật. Nếu khúc cua chữ S có bán kính rất hẹp và xe chạy
         * đủ chậm, về lý thuyết sensor_high_count>=3 vẫn có thể giữ đủ lâu
         * để lọt qua ngưỡng này - cần test thực tế đúng khúc cua gấp nhất
         * trên đường thi công, tăng CORNER_WIDE_PERSIST_TICKS nếu vẫn còn
         * báo ngã tư giả ở khúc cua. Ngược lại, tăng ngưỡng này quá cao sẽ
         * làm trễ phản ứng ở ngã tư thật - đánh đổi cần tune trên xe thật,
         * không suy ra được từ code. */
        uint8_t wide_now = (sensor_high_count >= 3);
        if (wide_now) {
            if (wide_persist_count < 250) wide_persist_count++;
        } else {
            wide_persist_count = 0;
        }
        uint8_t likely_cross = (wide_persist_count >= CORNER_WIDE_PERSIST_TICKS);

        if (corner_state == CORNER_NONE && line_detected && likely_cross &&
            (side_left != side_right || side_edge_happened)) {

            /* FIX: Bước +=2 nên chỉ cần 2 tick liên tiếp là đạt ngưỡng 3
             * (0→2→4 ≥ 3), không phải 3 nhịp như comment cũ. */
            if (corner_confirm_count < CORNER_CONFIRM_TICKS) corner_confirm_count += 2;

            if (side_edge_happened && edge_dir != 0) corner_pending_dir = edge_dir;
            else if (side_left != side_right) corner_pending_dir = side_right ? 1 : -1;

            if (corner_confirm_count >= CORNER_CONFIRM_TICKS) {
                corner_state = CORNER_ARMED;
                corner_dir = corner_pending_dir;
                corner_timer = 0;
                corner_confirm_count = 0;
            }
        } else if (corner_confirm_count > 0) {
            corner_confirm_count--;
        }
    }

    if (!line_detected) {
        if (sensor_high_count >= 1 && sensor_high_count <= 3) line_detected = 1;
        else { Set_Motors_Compensated(0, 0); return; }
    }

    /* ---- TẦNG 4: XỬ LÝ MẤT VẠCH (LOST LINE) ---- */
    if (sensor_high_count == 0 || sensor_high_count == 5) {
        int pwm_l, pwm_r;
        lost_count++;

        if (lost_count > MAX_LOST_CYCLES) {
            Set_Motors_Compensated(0, 0);
            error = 0; last_error = 0; I_term = 0.0f; line_detected = 0;
            lost_count = 0; last_turn_direction = 0;
            return;
        }

        /* Quay tại chỗ dò tìm line theo tín hiệu cuối cùng */
        if (sensor_high_count == 0) {
            if (side_right && !side_left) { pwm_l = PIVOT_SPEED; pwm_r = -PIVOT_SPEED; }
            else if (side_left && !side_right) { pwm_l = -PIVOT_SPEED; pwm_r = PIVOT_SPEED; }
            else if (last_turn_direction >= 0) { pwm_l = PIVOT_SPEED; pwm_r = -PIVOT_SPEED; }
            else { pwm_l = -PIVOT_SPEED; pwm_r = PIVOT_SPEED; }
        } else {
            pwm_l = Base_Speed; pwm_r = Base_Speed;
        }

        Set_Motors_Compensated(pwm_l, pwm_r);
        return;
    }

    lost_count = 0;

    /* ---- TẦNG 5: TÍNH TOÁN PID CHẠY THẲNG ---- */
    switch (raw_state) {
        case 0x04: error =  0; break;
        case 0x0C: error =  1; break;
        case 0x08: error =  2; break;
        case 0x18: error =  3; break;
        case 0x10: error =  4; break;
        case 0x06: error = -1; break;
        case 0x02: error = -2; break;
        case 0x03: error = -3; break;
        case 0x01: error = -4; break;
        default:   error = last_error; break;
    }

    if (error > 0) last_turn_direction = 1;
    else if (error < 0) last_turn_direction = -1;

    float p_term = Kp * error;
    I_term += Ki * error;
    if (I_term > 200.0f) I_term = 200.0f;
    else if (I_term < -200.0f) I_term = -200.0f;

    float d_term = Kd * (error - last_error);
    last_error = error;

    int pid_value = (int)(p_term + I_term + d_term);
    int abs_error = (error >= 0) ? error : -error;

    /* Phanh tự động khi vào cua, đảm bảo không rớt dưới AUTO_MIN_BASE_SPEED */
    int base_pwm = Base_Speed - (abs_error * 25);
    if (base_pwm < AUTO_MIN_BASE_SPEED) base_pwm = AUTO_MIN_BASE_SPEED;

    int pwm_l = base_pwm + pid_value;
    int pwm_r = base_pwm - pid_value;

    if (pwm_l > 999)  { pwm_l = 999; }
    if (pwm_l < -999) { pwm_l = -999; }
    if (pwm_r > 999)  { pwm_r = 999; }
    if (pwm_r < -999) { pwm_r = -999; }

    Set_Motors_Compensated(pwm_l, pwm_r);
}
