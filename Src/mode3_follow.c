/**
 * @file mode3_follow.c
 * @brief Mode 3: Bam vat the bang HC-SR04 quet servo - LOCK (tim tho toan
 *        dai) + TRACK (bam min, cua so hep, tu re-center).
 *
 * LOCK  : quet toan dai 30->150 do, buoc LOCK_SCAN_STEP_DEG (15 do, 9
 *         diem - giu day vi vat ~10cm o xa 40cm chi chiem ~14 do goc
 *         nhin, hep hon khe ho neu quet thua hon). Tu dong quet lai tu
 *         dau neu chua thay gi, khong bao gio "bo cuoc".
 *
 * TRACK : quet 1 cua so (TRACK_WINDOW_POINTS diem, buoc
 *         TRACK_WINDOW_STEP_DEG) quanh track_center_angle, RE-CENTER
 *         ngay tai goc vua tim duoc moi vong. Mat vat TRACK_LOST_LIMIT
 *         vong lien tiep -> ve LOCK.
 *
 * MOI DIEM DO 2 LAN (TRACK_MEASURE_COUNT=2), khong con 1 lan: 1 lan do
 * duy nhat rat de bi nhieu/timeout ngau nhien lam mat vat oan du vat
 * van o do. Neu ca 2 lan hop le, lay gia tri GAN HON (an toan hon cho
 * cac buoc dung/panic phia sau).
 *
 * DIEU KHIEN DONG CO (Update_Follow_Motor) - 4 uu tien theo thu tu, khong
 * bao gio 2 hanh dong (xoay + tien/lui) cung 1 tick:
 *   1. Khoang cach trong [STOP_NEAR, STOP_FAR] -> DUNG HAN, bat ke goc lech.
 *   2. Con lech goc, CHUA dung luot nhich cua vong nay -> xoay tai cho 1 CU
 *      NGAN (CORRECT_PIVOT -> CORRECT_WAIT), do dai TI LE VOI SO BUOC LECH,
 *      co RAMP len/xuong o dau-cuoi de giam giat co khi (xem
 *      TRACK_CORRECT_RAMP_TICKS).
 *   3. Da dung het luot nhich ma VAN CON lech -> DUNG IM CHO (khong
 *      tien/lui, tranh di CHEO sai huong) - cho vong quet moi mang ve error
 *      tuoi roi moi nhich tiep.
 *   4. Da THANG HUONG that su (eff_error == 0) -> TIEN/LUI THANG.
 *
 * KHOA DO TRONG LUC XOAY/PANIC: switch quet trong Update_Ultrasonic() chi
 * chay khi correct_state == CORRECT_NONE VA khong panic_reverse_active -
 * tranh doc sai do rung than xe/servo tro giua luc chassis dang chuyen
 * dong manh. Phep do tiep tuc dung cho da bo khi mo khoa lai, khong quet
 * lai tu dau.
 *
 * 3 VUNG KHOANG CACH (dong nhat, khong con chong lan):
 *   dist < 5           : panic - MAT vat dot ngot ngay sau khi vua do rat
 *                         gan (FOLLOW_PANIC_DIST_CM) -> lui full luc
 *   5 <= dist < 10      : lui thuong (UU TIEN 4, khong panic)
 *   10 <= dist <= 15     : DUNG (khoang cach an toan, doc lap voi logic xoay)
 *   dist > 15           : tien toi
 *
 * DANH DOI: moi vong quet chi sinh DUNG 1 lan nhich (khong xoay lien tuc
 * dua tren error cu) -> loi goc lon can vai vong quet de hoi tu, cham hon
 * ly thuyet nhung doi lai khong bao gio xoay qua da/dao dong khong dung.
 * Cua so TRACK rong (5 diem) + do 2 lan/diem lam vong quet cham hon truoc,
 * nhung uu tien dung huong/nhan vat truoc, toc do la thu yeu.
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
 * LOCK - quet tho toan dai, chi de tim so bo huong vat (TRACK bam chinh
 * xac lai sau)
 * ================================================================ */

#define LOCK_SCAN_STEP_DEG        15
#define SCAN_POINT_COUNT \
    (((SERVO_TRACK_MAX_DEG - SERVO_TRACK_MIN_DEG) / LOCK_SCAN_STEP_DEG) + 1)

#define SERVO_SETTLE_TICKS         25   /* servo nhay xa, can settle lau */
#define MEASURE_COUNT              3    /* 3 lan do + median */


/* ================================================================
 * TRACK - cua so quanh track_center_angle, quet min de bam chinh xac
 *
 * [SUA] TRACK_WINDOW_POINTS 3 -> 5 (cua so 30 do -> 50 do): cua so hep
 * de vat "troi" ra ngoai giua 2 vong quet neu vat dich chuyen, gay cam
 * giac "phat hien kem". Doi lai vong quet TRACK cham hon truoc - chap
 * nhan duoc vi uu tien nhan vat truoc toc do.
 * [SUA] TRACK_MEASURE_COUNT 1 -> 2: xem giai thich o dau file.
 * ================================================================ */

#define TRACK_WINDOW_POINTS        5    /* cua so rong 50 do (5 x 15) */
#define TRACK_WINDOW_STEP_DEG      15
#define TRACK_LOST_LIMIT           2    /* so vong lien tiep khong thay vat truoc khi ve LOCK */
#define TRACK_SETTLE_TICKS         6
#define TRACK_MEASURE_COUNT        2


/* ================================================================
 * HC-SR04
 * ================================================================ */

/* [SUA] Cac gia tri nay TRUOC LA HANG SO CO DINH (40cm/5-7cm/300 pwm) -
 * qua chat, khien xe chi bam duoc vat rat gan va dung o vung sat va cham.
 * Gio la BIEN co the chinh RUNTIME qua lenh UART "C3 <target> <tol> <max>
 * <move> <turn>" (xem Mode3_Follow_SetTuning ben duoi) - khong can build
 * lai firmware moi lan hieu chinh tren xe that. Gia tri duoi day chi la
 * MAC DINH luc chua nhan lenh C3 nao. */
#define FOLLOW_TARGET_DEFAULT_CM     15
#define FOLLOW_TOLERANCE_DEFAULT_CM  5
#define FOLLOW_MAX_DIST_DEFAULT_CM   800
#define FOLLOW_MOVE_PWM_DEFAULT      400
#define FOLLOW_TURN_PWM_DEFAULT      430

/* Gioi han an toan khi nhan lenh C3 - tranh GUI/nguoi dung gui nham gia
 * tri lam xe mat kiem soat (vd target am, PWM vuot muc dong co chiu duoc). */
#define FOLLOW_TARGET_MIN_CM      25
#define FOLLOW_TARGET_MAX_CM      100
#define FOLLOW_TOLERANCE_MIN_CM    3
#define FOLLOW_TOLERANCE_MAX_CM   25
#define FOLLOW_MAX_DIST_MIN_CM    60
#define FOLLOW_MAX_DIST_MAX_CM   220
#define FOLLOW_PWM_FLOOR         380    /* duoi muc nay dong co khong du luc thang tai (vung chet) */
#define FOLLOW_PWM_CEIL          700    /* PHAI khop FOLLOW_MAX_PWM ben duoi - da la gioi han clamp
                                          * cuoi cung truoc khi goi Set_Motors_Compensated() */

/* Lui manh hon tien mot chut (vat co the bi day toi nhanh hon xe tu tien) -
 * giu nguyen ty le nhu ban cu (300 -> 320, tuc +20). */
#define FOLLOW_RETREAT_EXTRA_PWM   20

#define MEASURE_TIMEOUT_TICKS     12    /* ~120ms @ 10ms/tick - qua nguong nay coi la mat echo,
                                          * KHONG dung mau cu du sequence chua doi. Thay the hoan
                                          * toan cho PING_WAIT_TICKS co dinh cu (xem
                                          * Update_Ultrasonic() -> ROUND_MEASURE_WAIT). */


/* ================================================================
 * MOTOR
 * ================================================================ */

#define FOLLOW_MAX_PWM             700

/* [SUA] FOLLOW_STOP_NEAR_CM/FAR_CM, FOLLOW_APPROACH_SPEED/RETREAT_SPEED,
 * FOLLOW_TURN_SPEED khong con la hang so - da chuyen thanh bien runtime
 * (follow_stop_near_cm...) o duoi, dat gia tri qua Mode3_Follow_SetTuning().
 * PANIC giu NGUYEN dang hang so tuyet doi, KHONG phu thuoc follow_stop_near_cm -
 * tranh nham lan khi sau nay doi vung dung ma quen doi PANIC theo. */
#define FOLLOW_PANIC_DIST_CM       2     /* mat vat dot ngot ngay sau khi vua do duoi muc nay moi coi la panic that su */
#define FOLLOW_PANIC_REVERSE_PWM   300
#define FOLLOW_PANIC_TICKS_LIMIT   30

/* Vung chet cho error: goc servo chi co do phan giai TRACK_WINDOW_STEP_DEG,
 * moi vong TRACK re-scan de "hunting" +-1 buoc du vat dung yen (sai so
 * luong tu hoa). So sanh BAO GOM BIEN (>=/<=) de +-1 buoc cung duoc coi
 * la "dang thang". */
#define FOLLOW_ANGLE_DEADZONE_DEG  TRACK_WINDOW_STEP_DEG

/* Xoay tai cho DUNG 1 cu/vong quet, do dai TI LE SO BUOC LECH
 * (abs(eff_error)/TRACK_WINDOW_STEP_DEG, toi da TRACK_CORRECT_MAX_STEPS
 * buoc) - loi lon nhich lau hon, hoi tu nhanh hon ma khong can nhieu
 * vong quet lien tiep.
 * [THEM] TRACK_CORRECT_RAMP_TICKS: lam mem PWM tang/giam dan trong
 * RAMP_TICKS dau va RAMP_TICKS cuoi cua cu nhich (thay vi bat/tat dot
 * ngot o FOLLOW_TURN_SPEED) - giam giat co khi that su cam nhan duoc,
 * cung giam rung than xe lam sieu am doc nhieu hon luc vua nhich xong. */
#define TRACK_CORRECT_PIVOT_TICKS  6    /* burst co so: ~60ms / 1 buoc lech */
#define TRACK_CORRECT_WAIT_TICKS   3    /* dung han dap quan tinh, TRUOC KHI cho quet lai */
#define TRACK_CORRECT_MAX_STEPS    4    /* eff_error toi da 60 do = 4 buoc - gioi han tren */
#define TRACK_CORRECT_RAMP_TICKS   2    /* so tick lam mem o dau va o cuoi cu nhich */


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

typedef enum
{
    CORRECT_NONE = 0,
    CORRECT_PIVOT,
    CORRECT_WAIT

} CorrectState_t;


/* ================================================================
 * BIEN
 * ================================================================ */

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

static volatile CorrectState_t correct_state = CORRECT_NONE;
static volatile int correct_timer = 0;
static volatile int correct_dir = 0;
static volatile int pivot_ticks_target = TRACK_CORRECT_PIVOT_TICKS;

/* Da dung luot nhich (1 cu PIVOT->WAIT) cua VONG QUET HIEN TAI hay chua.
 * Reset ve 0 DUY NHAT trong FinishRound() (moi khi co error tuoi tu 1 vong
 * quet MOI). Khi =1, khong nhich them trong cung vong - neu van con lech,
 * roi vao UU TIEN 3 (dung im cho). */
static volatile uint8_t round_pivot_used = 0;

/* [THEM] Tham so hieu chinh runtime - xem giai thich o dinh nghia
 * FOLLOW_*_DEFAULT_CM/PWM phia tren. Khoi tao bang gia tri mac dinh, CHI
 * doi khi nhan lenh "C3 ..." qua Mode3_Follow_SetTuning() - Mode3_Follow_Init()
 * KHONG duoc dong vao day, vi GUI gui "C3 ..." NGAY TRUOC "O" moi lan bat
 * dau bam; neu Init() reset ve default sau do se de mat gia tri C3 vua nhan. */
static volatile int follow_stop_near_cm = FOLLOW_TARGET_DEFAULT_CM - FOLLOW_TOLERANCE_DEFAULT_CM;
static volatile int follow_stop_far_cm  = FOLLOW_TARGET_DEFAULT_CM + FOLLOW_TOLERANCE_DEFAULT_CM;
static volatile int follow_max_dist_cm  = FOLLOW_MAX_DIST_DEFAULT_CM;
static volatile int follow_move_pwm     = FOLLOW_MOVE_PWM_DEFAULT;
static volatile int follow_turn_pwm     = FOLLOW_TURN_PWM_DEFAULT;

/* Sequence cua phep do dang cho ket qua - luu luc goi HCSR04_RequestMeasurement()
 * de sau do biet chac ket qua doc duoc la cua phep do NAY, khong phai echo cu. */
static volatile uint32_t pending_measure_sequence = 0;


extern volatile int error;
extern volatile int log_pwm_l;
extern volatile int log_pwm_r;
extern volatile int log_distance_cm_x10;
extern volatile int telemetry_ready;

#define FOLLOW_TELE_DIV 5

static volatile int follow_tele_count = 0;


void Mode3_Follow_SetTuning(int target_cm, int tolerance_cm, int max_cm,
                             int move_pwm, int turn_pwm)
{
    int near_cm;
    int far_cm;

    if (target_cm < FOLLOW_TARGET_MIN_CM) target_cm = FOLLOW_TARGET_MIN_CM;
    if (target_cm > FOLLOW_TARGET_MAX_CM) target_cm = FOLLOW_TARGET_MAX_CM;

    if (tolerance_cm < FOLLOW_TOLERANCE_MIN_CM) tolerance_cm = FOLLOW_TOLERANCE_MIN_CM;
    if (tolerance_cm > FOLLOW_TOLERANCE_MAX_CM) tolerance_cm = FOLLOW_TOLERANCE_MAX_CM;

    if (max_cm < FOLLOW_MAX_DIST_MIN_CM) max_cm = FOLLOW_MAX_DIST_MIN_CM;
    if (max_cm > FOLLOW_MAX_DIST_MAX_CM) max_cm = FOLLOW_MAX_DIST_MAX_CM;

    if (move_pwm < FOLLOW_PWM_FLOOR) move_pwm = FOLLOW_PWM_FLOOR;
    if (move_pwm > FOLLOW_PWM_CEIL)  move_pwm = FOLLOW_PWM_CEIL;

    if (turn_pwm < FOLLOW_PWM_FLOOR) turn_pwm = FOLLOW_PWM_FLOOR;
    if (turn_pwm > FOLLOW_PWM_CEIL)  turn_pwm = FOLLOW_PWM_CEIL;

    near_cm = target_cm - tolerance_cm;
    far_cm  = target_cm + tolerance_cm;
    if (near_cm < 1) near_cm = 1;  /* khong de vung dung lan vao/duoi vung panic (FOLLOW_PANIC_DIST_CM) */

    follow_stop_near_cm = near_cm;
    follow_stop_far_cm  = far_cm;
    follow_max_dist_cm  = max_cm;
    follow_move_pwm     = move_pwm;
    follow_turn_pwm     = turn_pwm;
}


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
        angle = SERVO_TRACK_MIN_DEG + idx * LOCK_SCAN_STEP_DEG;
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

    correct_state = CORRECT_NONE;
    correct_timer = 0;
    correct_dir = 0;
    pivot_ticks_target = TRACK_CORRECT_PIVOT_TICKS;
    round_pivot_used = 0;

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


/* Stub rong - giu lai CHI de khong pha vo API cong khai da khai bao trong
 * mode3_follow.h (Mode 3 hien chi con duong bam vat bang sieu am, khong
 * con nhanh AI Camera). Neu header/GUI khong con goi toi, co the xoa han
 * ca khai bao lan dinh nghia nay. */
void Mode3_Follow_SetError(int x_err, int y_err)
{
    (void)x_err;
    (void)y_err;
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
                found_angle = SERVO_TRACK_MIN_DEG + i * LOCK_SCAN_STEP_DEG;
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
            /* Khong doi follow_mode: van con MODE_LOCK, vong sau tu dong
             * quet lai tu dau - khong "bo cuoc" du chua thay vat. */
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

    /* Error vua duoc lam moi that su -> cho phep sinh DUNG 1 lan nhich moi
     * (neu can) cho vong nay. */
    round_pivot_used = 0;
}


static void Update_Follow_Motor(int *pwm_l, int *pwm_r)
{
    int turn;
    int forward;
    int eff_error;

    /* Panic reverse: PWM co dinh, gianh quyen dieu khien hoan toan, huy
     * moi cu nhich dang do dang. */
    if (panic_reverse_active)
    {
        *pwm_l = -FOLLOW_PANIC_REVERSE_PWM;
        *pwm_r = -FOLLOW_PANIC_REVERSE_PWM;

        prev_out_l = *pwm_l;
        prev_out_r = *pwm_r;

        correct_state = CORRECT_NONE;
        correct_timer = 0;
        round_pivot_used = 0;

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

        correct_state = CORRECT_NONE;
        correct_timer = 0;
        round_pivot_used = 0;
        return;
    }

    /* UU TIEN 1 - vung dung an toan, bat ke goc lech. */
    if (best_distance >= follow_stop_near_cm && best_distance <= follow_stop_far_cm)
    {
        *pwm_l = 0;
        *pwm_r = 0;
        prev_out_l = 0;
        prev_out_r = 0;

        correct_state = CORRECT_NONE;
        correct_timer = 0;
        round_pivot_used = 0;
        return;
    }

    eff_error = error;

    if (eff_error >= -FOLLOW_ANGLE_DEADZONE_DEG && eff_error <= FOLLOW_ANGLE_DEADZONE_DEG)
    {
        eff_error = 0;
    }

    /* UU TIEN 2 - xoay tai cho DUNG 1 cu/vong quet, do dai ti le so buoc
     * lech (xem giai thich o dinh nghia TRACK_CORRECT_*). */
    if (correct_state == CORRECT_NONE && eff_error != 0 && !round_pivot_used)
    {
        int steps = (eff_error > 0) ? (eff_error / TRACK_WINDOW_STEP_DEG)
                                     : (-eff_error / TRACK_WINDOW_STEP_DEG);

        if (steps < 1) steps = 1;
        if (steps > TRACK_CORRECT_MAX_STEPS) steps = TRACK_CORRECT_MAX_STEPS;

        correct_state = CORRECT_PIVOT;
        correct_timer = 0;
        correct_dir = (eff_error > 0) ? 1 : -1;
        pivot_ticks_target = TRACK_CORRECT_PIVOT_TICKS * steps;
    }

    if (correct_state == CORRECT_PIVOT)
    {
        /* [THEM] Ramp: trong RAMP_TICKS dau va RAMP_TICKS cuoi cua cu
         * nhich, tang/giam PWM tuyen tinh thay vi bat/tat dot ngot o
         * FOLLOW_TURN_SPEED - giam giat co khi va rung than xe. Doan
         * giua (neu pivot du dai) van chay full toc nhu cu. */
        int ramp_in = correct_timer + 1;
        int ramp_out = pivot_ticks_target - correct_timer;
        int scale = TRACK_CORRECT_RAMP_TICKS;

        if (ramp_in < scale)  scale = ramp_in;
        if (ramp_out < scale) scale = ramp_out;
        if (scale < 1) scale = 1;
        if (scale > TRACK_CORRECT_RAMP_TICKS) scale = TRACK_CORRECT_RAMP_TICKS;

        turn = (correct_dir * follow_turn_pwm * scale) / TRACK_CORRECT_RAMP_TICKS;

        *pwm_l = turn;
        *pwm_r = -turn;

        prev_out_l = *pwm_l;
        prev_out_r = *pwm_r;

        if (++correct_timer >= pivot_ticks_target)
        {
            correct_state = CORRECT_WAIT;
            correct_timer = 0;
        }

        return;
    }

    if (correct_state == CORRECT_WAIT)
    {
        *pwm_l = 0;
        *pwm_r = 0;

        prev_out_l = 0;
        prev_out_r = 0;

        if (++correct_timer >= TRACK_CORRECT_WAIT_TICKS)
        {
            /* Xong 1 nhich duy nhat cua vong nay. Tro ve CORRECT_NONE de
             * Update_Ultrasonic() mo khoa lai qua trinh do. */
            correct_state = CORRECT_NONE;
            correct_timer = 0;
            round_pivot_used = 1;
        }

        return;
    }

    /* UU TIEN 3 - da dung het luot nhich ma VAN CON lech -> dung im cho,
     * TUYET DOI khong tien/lui (tranh di cheo sai huong). Do da mo khoa
     * lai (correct_state == CORRECT_NONE), se som mang ve error moi. */
    if (eff_error != 0)
    {
        *pwm_l = 0;
        *pwm_r = 0;

        prev_out_l = 0;
        prev_out_r = 0;

        return;
    }

    /* UU TIEN 4 - da thang huong that su -> tien/lui thang. */
    if (best_distance > follow_stop_far_cm)
    {
        forward = follow_move_pwm;
    }
    else
    {
        int retreat_pwm = follow_move_pwm + FOLLOW_RETREAT_EXTRA_PWM;
        if (retreat_pwm > FOLLOW_PWM_CEIL) retreat_pwm = FOLLOW_PWM_CEIL;
        forward = -retreat_pwm;
    }

    *pwm_l = forward;
    *pwm_r = forward;

    prev_out_l = forward;
    prev_out_r = forward;
}


static void Update_Ultrasonic(int *pwm_l, int *pwm_r)
{
    /* Goi dong co TRUOC, roi moi xet co cho do tiep tuc khong - khoa do
     * NGAY LAP TUC trong chinh tick chuyen sang CORRECT_PIVOT/panic,
     * khong lot 1 tick nao. Do CHI chay khi than xe dang dung yen/di
     * thang that su (khong dang xoay, khong dang panic-reverse) - trong
     * ca 2 truong hop kia than xe rung/di chuyen manh, do luc nay de sai,
     * de mat dau vat hon. */
    Update_Follow_Motor(pwm_l, pwm_r);

    if (correct_state != CORRECT_NONE || panic_reverse_active)
    {
        return;
    }

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
            /* [SUA] Luu sequence NGAY LUC yeu cau - ROUND_MEASURE_WAIT sau
             * do so sanh voi sequence hien tai de biet CHAC ket qua la cua
             * phep do nay, khong con doan mo bang so tick co dinh. */
            pending_measure_sequence = HCSR04_GetMeasurementSequence();
            HCSR04_RequestMeasurement();

            wait_tick_counter = 0;

            follow_state = ROUND_MEASURE_WAIT;

            break;
        }

        case ROUND_MEASURE_WAIT:
        {
            wait_tick_counter++;

            if (HCSR04_GetMeasurementSequence() != pending_measure_sequence)
            {
                /* Phep do MOI da hoan tat that su (khong phai doan mo) -
                 * gia tri doc duoc chac chan thuoc ve lan yeu cau nay. */
                int dist = HCSR04_GetDistance_cm_x10() / 10;

                if (dist > 0 && dist <= follow_max_dist_cm)
                {
                    measure_values[measure_idx] = dist;
                }
                else
                {
                    measure_values[measure_idx] = -1;
                }

                follow_state = ROUND_NEXT_MEASURE;
            }
            else if (wait_tick_counter >= MEASURE_TIMEOUT_TICKS)
            {
                /* Qua timeout ma sequence van chua doi -> that su mat echo,
                 * KHONG dung mau cu (khac han vi tri cu chi doi PING_WAIT_TICKS
                 * roi doc bien du sequence chua kip doi). */
                measure_values[measure_idx] = -1;
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
                else if (need == 2)
                {
                    /* [THEM] TRACK: 2 lan do, khong median (can >=3 mau).
                     * Neu ca 2 hop le, lay gia tri GAN HON - an toan hon
                     * cho cac buoc dung/panic phia sau va giam rui ro bo
                     * sot vat neu 1 trong 2 lan bi nhieu cao hon thuc te.
                     * Neu chi 1 lan hop le, dung lan do; ca 2 that bai moi
                     * coi la mat vat that su o diem nay. */
                    int a = measure_values[0];
                    int b = measure_values[1];

                    if (a > 0 && b > 0)
                    {
                        result = (a < b) ? a : b;
                    }
                    else if (a > 0)
                    {
                        result = a;
                    }
                    else if (b > 0)
                    {
                        result = b;
                    }
                    else
                    {
                        result = -1;
                    }
                }
                else
                {
                    /* LOCK: 3 lan do + median, bu neu 1 lan bi loi. */
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
}


void Mode3_Follow_Update(void)
{
    int pwm_l = 0;
    int pwm_r = 0;

    Update_Ultrasonic(&pwm_l, &pwm_r);

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
        /* [SUA] Truoc day khong bao gio gan bien nay trong Mode 3 - GUI
         * luon hien "-- cm" du xe dang bam binh thuong. best_distance=999
         * la gia tri "chua thay vat" noi bo, doi thanh -1 (dung quy uoc
         * "khong do duoc" ma communication.py da hieu san). */
        log_distance_cm_x10 = (best_distance < 999) ? (best_distance * 10) : -1;

        telemetry_ready = 1;
    }
}
