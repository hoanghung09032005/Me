#include "mode1.h"
#include "hardware.h"

#define MANUAL_TIMEOUT   40
#define MANUAL_MIN_PWM   300
#define TURN_SCALE_PCT   80
#define MANUAL_TELE_DIV  5

extern volatile int log_pwm_l, log_pwm_r, telemetry_ready;

volatile int manual_l = 0, manual_r = 0;
volatile int manual_watchdog = 0;
volatile int manual_tele_count = 0;

void Mode1_Init() {
    manual_l = manual_r = 0;
    manual_watchdog = 0;
    manual_tele_count = 0;
    Set_Motor_Outputs(0, 0);
}

void Mode1_Apply_Command(char dir, int speed_pct) {
    if (speed_pct < 0)   speed_pct = 0;
    if (speed_pct > 100) speed_pct = 100;

    int pwm = (speed_pct * 999) / 100;
    if (pwm > 0 && pwm < MANUAL_MIN_PWM) pwm = MANUAL_MIN_PWM; //Tự động đẩy lên tốc độ min nếu tốc độ hiện tại quá yếu

    int turn = (pwm * TURN_SCALE_PCT) / 100; //xoay nhẹ khoảng 0.8 lực, tránh xoay quá mạnh

    switch (dir) {
        case 'F': manual_l =  pwm;  manual_r =  pwm;  break;
        case 'B': manual_l = -pwm;  manual_r = -pwm;  break;
        case 'L': manual_l = -turn; manual_r =  turn; break;
        case 'R': manual_l =  turn; manual_r = -turn; break;
        default:  manual_l = 0;     manual_r = 0;     break;
    }
    manual_watchdog = 0;
}

void Mode1_Update() {
    if (++manual_watchdog > MANUAL_TIMEOUT) {
        manual_l = 0;
        manual_r = 0;
    }

    /* BÙ TRỪ LỆCH MOTOR: áp dụng ĐÚNG hệ số MOTOR_RIGHT_COMPENSATION_PCT
     * dùng chung với Mode 2 (định nghĩa duy nhất ở hardware.h) - bánh
     * phải thực tế quay nhanh/mạnh hơn bánh trái là đặc tính PHẦN CỨNG,
     * không phụ thuộc đang chạy Mode 1 hay Mode 2, nên phải bù giống hệt
     * nhau ở cả 2 nơi. Trước đây Mode 1 gọi thẳng Set_Motor_Outputs()
     * không qua bù, khiến xe lái tay bị xỉa lệch trong khi Mode 2 (sau
     * khi thêm Set_Motors_Compensated() ở mode2_obstacle.c) đã đi thẳng
     * đúng - 2 mode có "tính cách" lái khác nhau dù cùng 1 xe. */
    int compensated_r = (manual_r * MOTOR_RIGHT_COMPENSATION_PCT) / 100;
    Set_Motor_Outputs(manual_l, compensated_r);

    /* Log lại giá trị PWM THẬT SỰ đã xuất ra động cơ (đã bù), không phải
     * giá trị lệnh gốc manual_r - khớp đúng quy ước telemetry mà Mode 2
     * đang dùng (mode2_obstacle.c cũng log log_pwm_r = compensated_pwm_r,
     * xem Set_Motors_Compensated()), để GUI hiển thị PWM có cùng ý nghĩa
     * ở cả 2 mode, không lẫn lộn "lệnh" với "thực tế xuất ra". */
    log_pwm_l = manual_l;
    log_pwm_r = compensated_r;
    if (++manual_tele_count >= MANUAL_TELE_DIV) {
        manual_tele_count = 0;
        telemetry_ready = 1;
    }
}
