#include "mode2_obstacle.h"
#include "hardware.h"

/* =========================================================================
 * MODE2_OBSTACLE.C - PID Line Follower + OBSTACLE AVOIDANCE
 * ========================================================================= */

static float Kp = 42.0f;
static float Ki = 0.0f;
static float Kd = 120.0f;
static int   Base_Speed = 350;

#define MIN_SPEED        250
#define MAX_LOST_CYCLES  15
#define PIVOT_SPEED      300

/* ---- Obstacle Avoidance Parameters ---- */
#define OBSTACLE_DISTANCE_CM   15
#define AVOID_TURN_CYCLES      40
#define AVOID_FORWARD_CYCLES   60
#define AVOID_SAFETY_TIMEOUT   300

#define SERVO_CENTER_DEG   90
#define SERVO_LEFT_DEG     150
#define SERVO_RIGHT_DEG    30

extern volatile int error, last_error, log_pwm_l, log_pwm_r, telemetry_ready;
extern float HCSR04_ReadDistance_cm(void);
extern void  Servo_SetAngle(uint16_t deg);

static volatile float I_term = 0.0f;
static volatile int   lost_count = 0;
static volatile int   line_detected = 0;
static volatile int   last_turn_direction = 0;

typedef enum {
    AVOID_NONE = 0,
    AVOID_SCAN,
    AVOID_TURN1,
    AVOID_FORWARD1,
    AVOID_TURN2,
    AVOID_FORWARD2
} AvoidState_t;

static volatile AvoidState_t avoid_state  = AVOID_NONE;
static volatile int          avoid_timer  = 0;
static volatile int          avoid_turn_dir = 1;

void Mode2_Obstacle_Init(void) {
    line_detected = 0;
    I_term = 0.0f;
    error = last_error = 0;
    lost_count = 0;
    last_turn_direction = 0;
    avoid_state = AVOID_NONE;
    avoid_timer = 0;
    Servo_SetAngle(SERVO_CENTER_DEG);
    Set_Motor_Outputs(0, 0);
}

static void Run_Line_PID(uint8_t raw_state, uint8_t side_left, uint8_t side_right) {
    int leds_off = 0;
    for (int i = 0; i < 5; i++) {
        if ((raw_state >> i) & 1u) leds_off++;
    }

    if (!line_detected) {
        if (leds_off >= 1 && leds_off <= 3) {
            line_detected = 1;
        } else {
            Set_Motor_Outputs(0, 0);
            return;
        }
    }

    if (leds_off == 0 || leds_off == 5) {
        lost_count++;

        if (lost_count > MAX_LOST_CYCLES) {
            Set_Motor_Outputs(0, 0);
            error = last_error = 0;
            I_term = 0.0f;
            line_detected = 0;
            lost_count = 0;
            last_turn_direction = 0;
            log_pwm_l = log_pwm_r = 0;
            telemetry_ready = 1;
            return;
        }

        if (leds_off == 0) {
            int pwm_l, pwm_r;
            if (side_right && !side_left) {
                pwm_l =  PIVOT_SPEED; pwm_r = -PIVOT_SPEED;
            } else if (side_left && !side_right) {
                pwm_l = -PIVOT_SPEED; pwm_r =  PIVOT_SPEED;
            } else if (last_turn_direction >= 0) {
                pwm_l =  PIVOT_SPEED; pwm_r = -PIVOT_SPEED;
            } else {
                pwm_l = -PIVOT_SPEED; pwm_r =  PIVOT_SPEED;
            }
            Set_Motor_Outputs(pwm_l, pwm_r);
            log_pwm_l = pwm_l; log_pwm_r = pwm_r;
            telemetry_ready = 1;
            return;
        } else {
            Set_Motor_Outputs(Base_Speed, Base_Speed);
            log_pwm_l = log_pwm_r = Base_Speed;
            telemetry_ready = 1;
            return;
        }
    }

    lost_count = 0;

    switch (raw_state) {
        case 0x04: error =  0; break;
        case 0x0C: error =  1; break;   case 0x08: error =  2; break;
        case 0x18: error =  3; break;   case 0x10: error =  4; break;
        case 0x06: error = -1; break;   case 0x02: error = -2; break;
        case 0x03: error = -3; break;   case 0x01: error = -4; break;
        default:   error = last_error;  break;
    }

    if (error > 0) last_turn_direction = 1;
    else if (error < 0) last_turn_direction = -1;

    float P = Kp * error;
    I_term += Ki * error;
    if (I_term > 200.0f) I_term = 200.0f; else if (I_term < -200.0f) I_term = -200.0f;
    float D = Kd * (error - last_error);
    last_error = error;

    int PID_val = (int)(P + I_term + D);

    int abs_error = (error > 0) ? error : -error;
    int base_pwm = Base_Speed - (abs_error * 25);
    base_pwm = (base_pwm < MIN_SPEED) ? MIN_SPEED : base_pwm;

    int pwm_l = base_pwm + PID_val;
    int pwm_r = base_pwm - PID_val;

    if (pwm_l > 999) pwm_l = 999; else if (pwm_l < -999) pwm_l = -999;
    if (pwm_r > 999) pwm_r = 999; else if (pwm_r < -999) pwm_r = -999;

    Set_Motor_Outputs(pwm_l, pwm_r);
    log_pwm_l = pwm_l; log_pwm_r = pwm_r;
    telemetry_ready = 1;
}

void Mode2_Obstacle_Update(uint8_t raw_state, uint8_t side_left, uint8_t side_right) {

    /* ================================================================
     * NORMAL LINE FOLLOWING -> Measure distance every ~60ms
     * ================================================================ */
    if (avoid_state == AVOID_NONE) {
        static int ping_counter = 0;
        static float last_dist = 100.0f;

        if (++ping_counter >= 6) {
            ping_counter = 0;
            float d = HCSR04_ReadDistance_cm();
            if (d > 0.0f) last_dist = d;
        }

        if (last_dist > 0.0f && last_dist < OBSTACLE_DISTANCE_CM) {
            Set_Motor_Outputs(0, 0);
            avoid_state = AVOID_SCAN;
            avoid_timer = 0;
            return;
        }

        Run_Line_PID(raw_state, side_left, side_right);
        return;
    }

    /* ================================================================
     * OBSTACLE AVOIDANCE STATE MACHINE
     * ================================================================ */
    switch (avoid_state) {

        case AVOID_SCAN: {
            static float dist_left = 0.0f;
            static float dist_right = 0.0f;

            if (avoid_timer == 0) {
                Servo_SetAngle(SERVO_LEFT_DEG);
            }
            else if (avoid_timer == 30) {
                dist_left = HCSR04_ReadDistance_cm();
                Servo_SetAngle(SERVO_RIGHT_DEG);
            }
            else if (avoid_timer == 60) {
                dist_right = HCSR04_ReadDistance_cm();
                Servo_SetAngle(SERVO_CENTER_DEG);
            }
            else if (avoid_timer == 80) {
                if (dist_left <= 0.0f) avoid_turn_dir = 1;
                else if (dist_right <= 0.0f) avoid_turn_dir = -1;
                else avoid_turn_dir = (dist_right > dist_left) ? 1 : -1;

                avoid_state = AVOID_TURN1;
                avoid_timer = -1;
            }
            avoid_timer++;
            break;
        }

        case AVOID_TURN1: {
            int pwm_l, pwm_r;
            if (avoid_turn_dir > 0) { pwm_l =  PIVOT_SPEED; pwm_r = -PIVOT_SPEED; }
            else                    { pwm_l = -PIVOT_SPEED; pwm_r =  PIVOT_SPEED; }
            Set_Motor_Outputs(pwm_l, pwm_r);
            log_pwm_l = pwm_l; log_pwm_r = pwm_r; telemetry_ready = 1;

            avoid_timer++;
            if (avoid_timer >= AVOID_TURN_CYCLES) {
                avoid_state = AVOID_FORWARD1;
                avoid_timer = 0;
            }
            break;
        }

        case AVOID_FORWARD1: {
            Set_Motor_Outputs(Base_Speed, Base_Speed);
            log_pwm_l = log_pwm_r = Base_Speed; telemetry_ready = 1;

            avoid_timer++;
            if (avoid_timer >= AVOID_FORWARD_CYCLES) {
                avoid_state = AVOID_TURN2;
                avoid_timer = 0;
            }
            break;
        }

        case AVOID_TURN2: {
            int pwm_l, pwm_r;
            if (avoid_turn_dir > 0) { pwm_l = -PIVOT_SPEED; pwm_r =  PIVOT_SPEED; }
            else                    { pwm_l =  PIVOT_SPEED; pwm_r = -PIVOT_SPEED; }
            Set_Motor_Outputs(pwm_l, pwm_r);
            log_pwm_l = pwm_l; log_pwm_r = pwm_r; telemetry_ready = 1;

            avoid_timer++;
            if (avoid_timer >= AVOID_TURN_CYCLES) {
                avoid_state = AVOID_FORWARD2;
                avoid_timer = 0;
            }
            break;
        }

        case AVOID_FORWARD2: {
            Set_Motor_Outputs(Base_Speed, Base_Speed);
            log_pwm_l = log_pwm_r = Base_Speed; telemetry_ready = 1;

            int leds_off = 0;
            for (int i = 0; i < 5; i++) {
                if ((raw_state >> i) & 1u) leds_off++;
            }

            avoid_timer++;
            if (leds_off >= 1 && leds_off <= 3) {
                avoid_state = AVOID_NONE;
                avoid_timer = 0;
                line_detected = 1;
                error = last_error = 0;
                I_term = 0.0f;
            } else if (avoid_timer >= AVOID_SAFETY_TIMEOUT) {
                avoid_state = AVOID_NONE;
                avoid_timer = 0;
                line_detected = 0;
            }
            break;
        }

        default:
            avoid_state = AVOID_NONE;
            break;
    }
}
