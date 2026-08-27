/**
 * @file mode3_follow.c
 * @brief Mode 3: Bam vat the - LOCK (full scan) + TRACK (cua so hep, re-center)
 *
 * Kien truc:
 *
 *  LOCK  : quet toan dai 30 -> 150 do, step 15 do (9 diem), moi diem do 3 lan
 *          + median. Dung de TIM vat lan dau hoac sau khi mat dau.
 *
 *  TRACK : chi quet mot cua so hep 5 diem (step van 15 do, khong doi do
 *          phan giai) quanh track_center_angle. Moi vong xong se RE-CENTER
 *          cua so ngay tai goc gan nhat vua tim duoc - day la diem khac biet
 *          voi ban ALIGN cu (ban cu giu window co dinh quanh goc CU nen bi
 *          "dinh" khi vat troi tu tu).
 *
 *          Neu TRACK khong thay vat hop le lien tiep TRACK_LOST_LIMIT vong
 *          thi tu dong quay ve LOCK de quet lai toan dai.
 *
 * MOTOR: da noi vao Update_Follow_Motor() - xe chi chay khi dang o
 *        MODE_TRACK va vua co best_distance hop le. 3 UU TIEN THEO THU TU
 *        (khong bao gio trong 2 hanh dong cung 1 tick):
 *          1. Neu khoang cach nam trong vung dung [STOP_NEAR,STOP_FAR] ->
 *             DUNG HAN, BAT KE vat dang lech trai/phai the nao (vat chi
 *             can nam tren cung tron ban kinh do quanh cam bien).
 *          2. Neu chua dung va con lech goc ro ret -> XOAY TAI CHO (2
 *             banh cung do lon PWM, nguoc chieu), KHONG tien/lui.
 *          3. Neu da thang huong va chua dung -> TIEN/LUI THANG (2 banh
 *             cung 1 gia tri PWM, khong xoay).
 *        Thiet ke nay dam bao 2 banh LUON nhan PWM cung do lon (chi khac
 *        dau khi xoay) - tranh truong hop cong don turn+forward ra 1 gia
 *        tri qua nho o 1 ben, duoi nguong dong co thuc su quay noi, gay
 *        "1 banh quay 1 banh khong" lam xe tu xoay ngoai y muon.
 */

#include "mode3_follow.h"
#include "mode2_internal.h"
#include "hardware.h"


/* ================================================================
 * SERVO
 * ================================================================ */

#define SERVO_TRACK_MIN_DEG       30
#define SERVO_TRACK_MAX_DEG       150
#define SERVO_TRACK_CENTER_DEG    90


/* ================================================================
 * LOCK - quet toan dai
 *
 * 30,45,60,75,90,105,120,135,150 => 9 diem
 * ================================================================ */

#define SCAN_STEP_DEG             15

#define SCAN_POINT_COUNT \
    (((SERVO_TRACK_MAX_DEG - SERVO_TRACK_MIN_DEG) / SCAN_STEP_DEG) + 1)

#define SERVO_SETTLE_TICKS         25   /* LOCK: servo nhay xa, can settle lau */
#define MEASURE_COUNT              3    /* LOCK: 3 lan do + median */


/* ================================================================
 * TRACK - cua so hep quanh track_center_angle
 *
 * QUAN TRONG: giu nguyen do phan giai 15 do (khong tang step len 30 do),
 * chi tang SO DIEM (5 diem) de mo rong be rong cua so. Neu tang step,
 * khe ho giua 2 diem do se lon hon goc mo chum tia HC-SR04 tai khoang
 * cach xa, de bo sot vat hep (chai nuoc, coc).
 * ================================================================ */

#define TRACK_WINDOW_POINTS        5    /* step 15 do => cua so rong 60 do */
#define TRACK_WINDOW_STEP_DEG      15
#define TRACK_LOST_LIMIT           2    /* so vong lien tiep khong thay vat truoc khi ve LOCK */
#define TRACK_SETTLE_TICKS         10   /* TRACK: servo nhich it, settle nhanh hon */
#define TRACK_MEASURE_COUNT        1    /* TRACK: 1 lan do/diem, khong median */


/* ================================================================
 * HC-SR04
 * ================================================================ */

/* [SUA] Tang tam phat hien tu 30 -> 40cm: xe chay nhanh co the "vut qua"
 * vat truoc khi vong quet servo (~750ms/vong) kip nhan ra, dan den mat
 * dau vat mac du vat van con trong tam mat. Tang tam giup TRACK giu duoc
 * vat lau hon o toc do cao. */
#define FOLLOW_MAX_DIST_CM        40

#define PING_WAIT_TICKS            5    /* dung chung cho ca LOCK va TRACK */


/* ================================================================
 * MOTOR
 * ================================================================ */

#define FOLLOW_BASE_SPEED          300
#define FOLLOW_TURN_SPEED          400
#define FOLLOW_MAX_PWM             700

/* [SUA] Doi tu cong thuc ty le (dist_err * KP) sang 3 VUNG KHOANG CACH
 * CO DINH. Ly do: voi FOLLOW_MAX_DIST_CM gioi han dist_err toi da, PWM
 * "forward" truoc day gan nhu luon bi clamp ve dung 1 gia tri san
 * (FOLLOW_MIN_MOVE_PWM) trong toan bo dai khoang cach thuc te co the
 * xay ra - nghia la "ty le" chi la hinh thuc, khong tao ra khac biet
 * thuc chat.
 *
 * 3 vung, moi vung 1 toc do PWM co dinh:
 *   dist <  FOLLOW_STOP_NEAR_CM              -> LUI  (FOLLOW_RETREAT_SPEED)
 *   FOLLOW_STOP_NEAR_CM..FOLLOW_STOP_FAR_CM  -> DUNG (pwm = 0), BAT KE goc
 *   dist >  FOLLOW_STOP_FAR_CM                -> TIEN (FOLLOW_APPROACH_SPEED)
 *
 * Khong co truong hop dist am/loi lot vao day: HCSR04_GetDistance tra ve
 * gia tri khong hop le (<=0) da bi loc bo o ROUND_MEASURE_WAIT (xem
 * "dist > 0 && dist <= FOLLOW_MAX_DIST_CM"), nen best_distance khi vao
 * toi day luon la mot php do THAT trong khoang hop le. Truong hop mat
 * hoan toan tin hieu (khong do duoc) da duoc panic_reverse_active xu ly
 * RIENG o dau ham nay, khong lien quan gi den 3 vung o duoi. */
#define FOLLOW_STOP_NEAR_CM        7
#define FOLLOW_STOP_FAR_CM         12

#define FOLLOW_APPROACH_SPEED      300
/* Lui hoi manh hon tien: nguoi dieu khien vat co the day vat toi gan xe
 * nhanh hon toc do xe tu tien toi, nen can phan ung nhanh hon khi lui de
 * tranh bi dam/ep sat. Neu thuc te lui qua giat, ha xuong gan bang
 * FOLLOW_APPROACH_SPEED. */
#define FOLLOW_RETREAT_SPEED       320

/* ================================================================
 * PANIC REVERSE
 * ================================================================ */
#define FOLLOW_PANIC_DIST_CM       8
#define FOLLOW_PANIC_REVERSE_PWM   300
#define FOLLOW_PANIC_TICKS_LIMIT   30

/*
 * FOLLOW_ANGLE_DEADZONE_DEG: vung chet cho error (do lech goc servo).
 * Goc servo chi co do phan giai SCAN_STEP_DEG/TRACK_WINDOW_STEP_DEG
 * (15 do) - moi vong TRACK re-scan, cua so do "hunting" qua lai quanh
 * huong that cua vat (vd 75/90/105) DU VAT DUNG YEN, chi vi sai so
 * luong tu hoa cua servo/HC-SR04. Neu khong loc, moi lan hunting nhu
 * vay se kich hoat xoay (xem UU TIEN 2 trong Update_Follow_Motor) du
 * vat dung yen. Coi |error| <= 1 buoc quet la "dang thang", khong xoay -
 * chi xoay that su khi vat lech ro rang (>= 2 buoc quet).
 *
 * SUA LOI (phat hien khi review lai - dieu kien bien cu la NO-OP): ban
 * truoc dung so sanh CHAT (>  / <), vi error CHI CO THE la boi so cua
 * SCAN_STEP_DEG (15) - AngleAtIndex() luon cong/tru boi so 15 vao mot
 * moc cung la boi so 15 (track_center_angle khoi tao = 90). Voi
 * FOLLOW_ANGLE_DEADZONE_DEG=15, dieu kien "> -15 && < 15" CHI dung voi
 * error=0 (von di da bang 0) - moi gia tri thuc te khac (+-15, +-30,...)
 * deu KHONG lot qua dieu kien nay, nen eff_error khong bao gio thuc su bi
 * ep ve 0 ngoai truong hop no da la 0 san. Doi sang so sanh BAO GOM BIEN
 * (>= / <=) de +-15 (dung 1 buoc quet) cung duoc coi la "dang thang" -
 * dung y do that su cua deadzone nay. */
#define FOLLOW_ANGLE_DEADZONE_DEG  SCAN_STEP_DEG


/* ================================================================
 * AI MODE
 * ================================================================ */

#define AI_FOLLOW_DEADZONE_X       10
#define AI_FOLLOW_DEADZONE_Y       10
#define AI_FOLLOW_KP_X             8
#define AI_FOLLOW_KP_Y             4


/* ================================================================
 * STATE MACHINE
 * ================================================================ */

typedef enum
{
    MODE_LOCK = 0,
    MODE_TRACK = 1

} FollowMode;

typedef enum
{
    ROUND_SET_ANGLE,
    ROUND_WAIT_SERVO,
    ROUND_MEASURE_REQUEST,
    ROUND_MEASURE_WAIT,
    ROUND_NEXT_MEASURE,
    ROUND_FINISH

} FollowState;


/* ================================================================
 * BIEN
 * ================================================================ */

static volatile uint8_t follow_use_ai = 0;

static volatile int ai_x_err = 0;
static volatile int ai_y_err = 0;

static volatile FollowState follow_state = ROUND_SET_ANGLE;
static volatile FollowMode  follow_mode  = MODE_LOCK;

static volatile int debug_follow_mode = 0;
static volatile int track_center_angle = SERVO_TRACK_CENTER_DEG;
static volatile int track_miss_streak = 0;

static volatile int scan_idx = 0;
static volatile int measure_idx = 0;
static volatile int wait_tick_counter = 0;

static volatile int measure_values[MEASURE_COUNT];
static volatile int scan_dist[SCAN_POINT_COUNT];

static volatile int best_angle = SERVO_TRACK_CENTER_DEG;
static volatile int best_distance = 999;
static volatile int last_valid_distance = 999;

static volatile uint8_t panic_reverse_active = 0;
static volatile int panic_reverse_ticks = 0;

static volatile int prev_out_l = 0;
static volatile int prev_out_r = 0;


extern volatile int error;
extern volatile int log_pwm_l;
extern volatile int log_pwm_r;
extern volatile int telemetry_ready;

#define FOLLOW_TELE_DIV 5

static volatile int follow_tele_count = 0;


static int Median3(int a, int b, int c)
{
    int temp;

    if (a > b) { temp = a; a = b; b = temp; }
    if (b > c) { temp = b; b = c; c = temp; }
    if (a > b) { temp = a; a = b; b = temp; }

    return b;
}


static int PointCountForMode(void)
{
    return (follow_mode == MODE_LOCK) ? SCAN_POINT_COUNT : TRACK_WINDOW_POINTS;
}

static int SettleTicksForMode(void)
{
    return (follow_mode == MODE_LOCK) ? SERVO_SETTLE_TICKS : TRACK_SETTLE_TICKS;
}

static int MeasureCountForMode(void)
{
    return (follow_mode == MODE_LOCK) ? MEASURE_COUNT : TRACK_MEASURE_COUNT;
}

static int AngleAtIndex(int idx)
{
    int angle;

    if (follow_mode == MODE_LOCK)
    {
        angle = SERVO_TRACK_MIN_DEG + idx * SCAN_STEP_DEG;
    }
    else
    {
        int half_span = (TRACK_WINDOW_POINTS / 2) * TRACK_WINDOW_STEP_DEG;

        angle = (track_center_angle - half_span) + idx * TRACK_WINDOW_STEP_DEG;
    }

    if (angle < SERVO_TRACK_MIN_DEG) angle = SERVO_TRACK_MIN_DEG;
    if (angle > SERVO_TRACK_MAX_DEG) angle = SERVO_TRACK_MAX_DEG;

    return angle;
}


void Mode3_Follow_Init(void)
{
    int i;

    follow_use_ai = 0;

    ai_x_err = 0;
    ai_y_err = 0;

    follow_state = ROUND_SET_ANGLE;
    follow_mode = MODE_LOCK;
    debug_follow_mode = 0;

    track_center_angle = SERVO_TRACK_CENTER_DEG;
    track_miss_streak = 0;

    scan_idx = 0;
    measure_idx = 0;
    wait_tick_counter = 0;

    best_angle = SERVO_TRACK_CENTER_DEG;
    best_distance = 999;
    last_valid_distance = 999;

    panic_reverse_active = 0;
    panic_reverse_ticks = 0;

    prev_out_l = 0;
    prev_out_r = 0;

    error = 0;

    for (i = 0; i < SCAN_POINT_COUNT; i++)
    {
        scan_dist[i] = -1;
    }

    for (i = 0; i < MEASURE_COUNT; i++)
    {
        measure_values[i] = -1;
    }

    Servo_SetAngle(SERVO_TRACK_CENTER_DEG);
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


static void FinishRound(void)
{
    if (follow_mode == MODE_LOCK)
    {
        int i;
        int found_distance = 999;
        int found_angle = SERVO_TRACK_CENTER_DEG;

        for (i = 0; i < SCAN_POINT_COUNT; i++)
        {
            if (scan_dist[i] > 0 && scan_dist[i] < found_distance)
            {
                found_distance = scan_dist[i];
                found_angle = SERVO_TRACK_MIN_DEG + i * SCAN_STEP_DEG;
            }
        }

        if (found_distance < 999)
        {
            best_angle = found_angle;
            best_distance = found_distance;

            track_center_angle = found_angle;
            track_miss_streak = 0;

            follow_mode = MODE_TRACK;
        }
        else
        {
            best_angle = SERVO_TRACK_CENTER_DEG;
            best_distance = 999;
        }
    }
    else
    {
        int i;
        int found_distance = 999;
        int found_idx = -1;

        for (i = 0; i < TRACK_WINDOW_POINTS; i++)
        {
            if (scan_dist[i] > 0 && scan_dist[i] < found_distance)
            {
                found_distance = scan_dist[i];
                found_idx = i;
            }
        }

        if (found_idx >= 0)
        {
            int found_angle = AngleAtIndex(found_idx);

            track_center_angle = found_angle;
            best_angle = found_angle;
            best_distance = found_distance;
            last_valid_distance = found_distance;

            track_miss_streak = 0;

            panic_reverse_active = 0;
            panic_reverse_ticks = 0;
        }
        else
        {
            if (last_valid_distance < FOLLOW_PANIC_DIST_CM && !panic_reverse_active)
            {
                panic_reverse_active = 1;
                panic_reverse_ticks = 0;
            }
            else if (!panic_reverse_active)
            {
                track_miss_streak++;

                if (track_miss_streak >= TRACK_LOST_LIMIT)
                {
                    follow_mode = MODE_LOCK;
                    track_miss_streak = 0;
                    best_distance = 999;
                    last_valid_distance = 999;
                }
            }
        }
    }

    debug_follow_mode = (int)follow_mode;

    error = SERVO_TRACK_CENTER_DEG - best_angle;
}


static void Update_Follow_Motor(int *pwm_l, int *pwm_r)
{
    int turn;
    int forward;
    int eff_error;

    /* [SUA] Panic reverse: PWM co dinh, khong RateLimit - ly do giong
     * 2 nhanh TURN/DRIVE ben duoi (xem giai thich chi tiet o do). */
    if (panic_reverse_active)
    {
        *pwm_l = -FOLLOW_PANIC_REVERSE_PWM;
        *pwm_r = -FOLLOW_PANIC_REVERSE_PWM;

        prev_out_l = *pwm_l;
        prev_out_r = *pwm_r;

        if (++panic_reverse_ticks >= FOLLOW_PANIC_TICKS_LIMIT)
        {
            panic_reverse_active = 0;
            panic_reverse_ticks = 0;
            follow_mode = MODE_LOCK;
            track_miss_streak = 0;
            best_distance = 999;
            last_valid_distance = 999;
        }

        return;
    }

    if (follow_mode != MODE_TRACK || best_distance >= 999)
    {
        *pwm_l = 0;
        *pwm_r = 0;
        prev_out_l = 0;
        prev_out_r = 0;
        return;
    }

    /* [SUA] UU TIEN 1 - VUNG DUNG BAT KE GOC LECH:
     * Vat duoc coi la "da bam duoc" mien khoang cach nam trong
     * [FOLLOW_STOP_NEAR_CM, FOLLOW_STOP_FAR_CM], BAT KE vat dang lech
     * trai/phai bao nhieu - noi cach khac, vat chi can nam tren CUNG
     * TRON co ban kinh trong khoang do, tam la cam bien sieu am, la du
     * de dung han, khong xoay them de "can chinh" nua. Kiem tra dieu
     * kien nay TRUOC va return NGAY, khong de eff_error/turn ben duoi
     * co co hoi chen vao. */
    if (best_distance >= FOLLOW_STOP_NEAR_CM && best_distance <= FOLLOW_STOP_FAR_CM)
    {
        *pwm_l = 0;
        *pwm_r = 0;
        prev_out_l = 0;
        prev_out_r = 0;
        return;
    }

    eff_error = error;

    /* SUA LOI (xem giai thich day du o dinh file, dinh nghia
     * FOLLOW_ANGLE_DEADZONE_DEG): doi sang so sanh BAO GOM BIEN (>= / <=)
     * thay vi so sanh CHAT (> / <) - voi so sanh chat cu, dieu kien nay
     * la NO-OP vi error luon la boi so cua 15, khong bao gio roi vao
     * khoang mo (-15, 15) tru dung 0. */
    if (eff_error >= -FOLLOW_ANGLE_DEADZONE_DEG && eff_error <= FOLLOW_ANGLE_DEADZONE_DEG)
    {
        eff_error = 0;
    }

    /* [SUA] UU TIEN 2 - XOAY TAI CHO (khong tien/lui dong thoi):
     * Truoc day out_l = forward + turn / out_r = forward - turn cong 2
     * thanh phan lai voi nhau - khi forward va turn trai dau (vd forward
     * am luc lui, turn duong), 1 ben bi TRIET TIEU BOT con ben kia bi
     * CONG DON, tao ra 1 gia tri PWM nho/lech han so voi ben kia. Neu
     * gia tri nho do roi xuong duoi nguong dong co that su quay noi
     * (so sanh MANUAL_MIN_PWM=300 dang dung ben mode1.c cho lai tay) -
     * banh do se i, banh kia van chay binh thuong => xe tu xoay vong
     * ngoai y muon, dung la trieu chung ban gap phai.
     *
     * Sua: KHONG BAO GIO cong turn va forward trong cung 1 tick nua. Neu
     * con lech goc ro ret (eff_error != 0), CHI xoay tai cho (2 banh
     * dung do lon FOLLOW_TURN_SPEED, nguoc chieu nhau) - khong tien/lui.
     * Chi khi da thang huong (eff_error == 0) moi roi xuong UU TIEN 3
     * de tien/lui THANG (2 banh cung 1 gia tri, khong lech). Ca 2 nhanh
     * deu dam bao 2 banh LUON cung do lon PWM (chi khac dau khi xoay),
     * khong con truong hop 1 banh PWM nho hon ban kia. */
    if (eff_error != 0)
    {
        turn = (eff_error > 0) ? FOLLOW_TURN_SPEED : -FOLLOW_TURN_SPEED;

        *pwm_l = turn;
        *pwm_r = -turn;

        prev_out_l = *pwm_l;
        prev_out_r = *pwm_r;
        return;
    }

    /* [SUA] UU TIEN 3 - TIEN/LUI THANG (da het lech goc, khong con trong
     * vung dung): 2 banh nhan CUNG 1 gia tri PWM co dinh, khong co thanh
     * phan xoay nao cong vao. */
    forward = (best_distance > FOLLOW_STOP_FAR_CM) ? FOLLOW_APPROACH_SPEED : -FOLLOW_RETREAT_SPEED;

    *pwm_l = forward;
    *pwm_r = forward;

    prev_out_l = forward;
    prev_out_r = forward;
}


static void Update_Ultrasonic(int *pwm_l, int *pwm_r)
{
    switch (follow_state)
    {
        case ROUND_SET_ANGLE:
        {
            int angle = AngleAtIndex(scan_idx);

            Servo_SetAngle(angle);

            measure_idx = 0;
            wait_tick_counter = 0;

            follow_state = ROUND_WAIT_SERVO;

            break;
        }

        case ROUND_WAIT_SERVO:
        {
            wait_tick_counter++;

            if (wait_tick_counter >= SettleTicksForMode())
            {
                wait_tick_counter = 0;
                measure_idx = 0;

                follow_state = ROUND_MEASURE_REQUEST;
            }

            break;
        }

        case ROUND_MEASURE_REQUEST:
        {
            HCSR04_RequestMeasurement();

            wait_tick_counter = 0;

            follow_state = ROUND_MEASURE_WAIT;

            break;
        }

        case ROUND_MEASURE_WAIT:
        {
            wait_tick_counter++;

            if (wait_tick_counter >= PING_WAIT_TICKS)
            {
                int dist = HCSR04_GetDistance_cm_x10() / 10;

                if (dist > 0 && dist <= FOLLOW_MAX_DIST_CM)
                {
                    measure_values[measure_idx] = dist;
                }
                else
                {
                    measure_values[measure_idx] = -1;
                }

                follow_state = ROUND_NEXT_MEASURE;
            }

            break;
        }

        case ROUND_NEXT_MEASURE:
        {
            int need = MeasureCountForMode();

            measure_idx++;

            if (measure_idx < need)
            {
                wait_tick_counter = 0;
                follow_state = ROUND_MEASURE_REQUEST;
            }
            else
            {
                int result;

                if (need == 1)
                {
                    result = measure_values[0];
                }
                else
                {
                    int valid_count = 0;

                    int a = measure_values[0];
                    int b = measure_values[1];
                    int c = measure_values[2];

                    if (a > 0) valid_count++;
                    if (b > 0) valid_count++;
                    if (c > 0) valid_count++;

                    if (valid_count >= 2)
                    {
                        if (a <= 0) a = (b > 0) ? b : c;
                        if (b <= 0) b = (a > 0) ? a : c;
                        if (c <= 0) c = (a > 0) ? a : b;

                        result = Median3(a, b, c);
                    }
                    else
                    {
                        result = -1;
                    }
                }

                scan_dist[scan_idx] = result;

                scan_idx++;

                if (scan_idx >= PointCountForMode())
                {
                    scan_idx = 0;
                    follow_state = ROUND_FINISH;
                }
                else
                {
                    follow_state = ROUND_SET_ANGLE;
                }
            }

            break;
        }

        case ROUND_FINISH:
        {
            FinishRound();

            scan_idx = 0;
            measure_idx = 0;

            follow_state = ROUND_SET_ANGLE;

            break;
        }

        default:
        {
            follow_state = ROUND_SET_ANGLE;
            break;
        }
    }

    Update_Follow_Motor(pwm_l, pwm_r);
}


static void Update_AI(int *pwm_l, int *pwm_r)
{
    int abs_x = (ai_x_err < 0) ? -ai_x_err : ai_x_err;
    int abs_y = (ai_y_err < 0) ? -ai_y_err : ai_y_err;

    error = ai_x_err;

    if (abs_x > AI_FOLLOW_DEADZONE_X)
    {
        int turn = (ai_x_err * FOLLOW_TURN_SPEED) / 100;

        if (turn > FOLLOW_TURN_SPEED) turn = FOLLOW_TURN_SPEED;
        if (turn < -FOLLOW_TURN_SPEED) turn = -FOLLOW_TURN_SPEED;

        *pwm_l = turn;
        *pwm_r = -turn;
    }
    else
    {
        if (abs_y > AI_FOLLOW_DEADZONE_Y)
        {
            int speed = FOLLOW_BASE_SPEED - (ai_y_err * AI_FOLLOW_KP_Y);

            if (speed > FOLLOW_MAX_PWM) speed = FOLLOW_MAX_PWM;
            if (speed < -FOLLOW_MAX_PWM) speed = -FOLLOW_MAX_PWM;

            if (speed > 0 && speed < 200) speed = 200;
            if (speed < 0 && speed > -200) speed = -200;

            *pwm_l = speed;
            *pwm_r = speed;
        }
        else
        {
            *pwm_l = 0;
            *pwm_r = 0;
        }
    }
}


void Mode3_Follow_Update(void)
{
    int pwm_l = 0;
    int pwm_r = 0;

    if (follow_use_ai)
    {
        Update_AI(&pwm_l, &pwm_r);
    }
    else
    {
        Update_Ultrasonic(&pwm_l, &pwm_r);
    }

    if (pwm_l > FOLLOW_MAX_PWM) pwm_l = FOLLOW_MAX_PWM;
    if (pwm_l < -FOLLOW_MAX_PWM) pwm_l = -FOLLOW_MAX_PWM;
    if (pwm_r > FOLLOW_MAX_PWM) pwm_r = FOLLOW_MAX_PWM;
    if (pwm_r < -FOLLOW_MAX_PWM) pwm_r = -FOLLOW_MAX_PWM;

    Set_Motors_Compensated(pwm_l, pwm_r);

    if (++follow_tele_count >= FOLLOW_TELE_DIV)
    {
        follow_tele_count = 0;

        log_pwm_l = pwm_l;
        log_pwm_r = pwm_r;

        telemetry_ready = 1;
    }
}
