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
    Set_Motor_Outputs(manual_l, manual_r);

    log_pwm_l = manual_l;
    log_pwm_r = manual_r;
    if (++manual_tele_count >= MANUAL_TELE_DIV) {
        manual_tele_count = 0;
        telemetry_ready = 1;
    }
}

