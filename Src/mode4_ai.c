#include "mode4_ai.h"
#include "mode2_internal.h"
#include "hardware.h"
#include <math.h>   /* lroundf - nếu build lỗi "undefined reference to
                      * lroundf" do toolchain nano.specs không link libm
                      * float, thay bằng:
                      *   (int)(x >= 0.0f ? x + 0.5f : x - 0.5f)
                      */

/* ==================================================================
 * THÔNG SỐ PID (giống mode2_line_pid.c, có thể tune riêng)
 * ================================================================== */
static float Kp = 30.0f;
static float Ki = 0.0f;
static float Kd = 100.0f;

/* [SỬA] AI_BASE_SPEED tách khỏi AI_MIN_BASE_SPEED để "phanh tự động khi
 * vào cua" (base_speed - abs_e*25) có tác dụng thật thay vì luôn bị kẹp
 * về đúng 1 giá trị cố định bất kể abs_e lớn hay nhỏ. abs_e tối đa = 4.0
 * nên abs_e*25 tối đa = 100 - xem giá trị AI_BASE_SPEED/AI_MIN_BASE_SPEED
 * hiện hành ngay bên dưới để biết khoảng phanh thực tế đang là bao nhiêu. */
/* [SỬA - ĐỒNG BỘ VỚI MODE 2, rồi CHỈNH LẠI SAU TEST THẬT] Lần đầu đặt
 * AI_BASE_SPEED = 492 để khớp đúng tốc độ Mode 2 chạy thật ở slider mặc
 * định 60% (330 + 60%*(600-330) = 492) - nhưng test thực tế cho thấy xe
 * lệch hẳn quỹ đạo khi vào cua, NHANH HƠN mức Mode 2 xử lý được. Lý do:
 * vòng lặp điều khiển của Mode 4 có ĐỘ TRỄ LỚN HƠN NHIỀU so với Mode 2 -
 * Mode 2 đọc 5 mắt IR trực tiếp mỗi 10ms, còn Mode 4 phải đi hết chuỗi
 * chụp ảnh ESP32 -> nén JPEG -> gửi WiFi -> decode PC -> chạy model/
 * threshold -> gửi lệnh "I ..." xuống lại - cùng 1 PWM, xe đi được quãng
 * đường xa hơn nhiều trong khoảng trễ đó ở Mode 4, nên "tốc độ tối thiểu
 * để chạy được" của Mode 2 KHÔNG tự động đúng cho Mode 4. Hạ xuống 410
 * (giữa sàn 330 và mức 492 đã test lệch) để thử lại - nếu vẫn lệch, hạ
 * tiếp theo từng bước ~30-40 đơn vị, đừng nhảy thẳng về sàn 330 (đã biết
 * là "quá chậm" từ lần test trước). AI_MIN_BASE_SPEED giữ nguyên 330 -
 * đây vẫn là sàn lực quay bánh, không phải chỗ cần đổi cho vấn đề này. */
#define AI_BASE_SPEED             330
#define AI_MIN_BASE_SPEED         330
#define AI_MAX_BASE_SPEED         600   /* dự phòng cho GUI chỉnh tốc độ
                                          * nền sau này, hiện chưa có hàm
                                          * setter tương ứng nên chưa dùng
                                          * tới trong file này. */

#define AI_MAX_LOST_CYCLES        15    /* [SỬA] khớp MAX_LOST_CYCLES bên
                                          * mode2_line_pid.c (Mode 2 đã
                                          * kiểm chứng đủ dùng ở mức này,
                                          * không cần khoan dung gấp đôi
                                          * như trước). */
#define AI_PIVOT_SPEED            350

static volatile float I_term = 0.0f;
static volatile int ai_error = 0;
/* FIX BUG 3+4: đổi sang float - error AI vốn liên tục (-4.00..4.00), lưu
 * dạng int trước đây buộc phải lượng tử hoá thô, phá mất lợi thế regression
 * so với 5 mắt IR rời rạc. */
static volatile float last_ai_error = 0.0f;
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
    last_ai_error = 0.0f;
    lost_count = 0;
    line_detected = 0;
    last_turn_direction = 0;
    base_speed = AI_BASE_SPEED;
    ai_tele_count = 0;

    /* [SỬA] error/last_error là biến toàn cục dùng chung với Mode 2 (đóng
     * gói vào chuỗi telemetry gửi GUI). Không reset ở đây -> khi chuyển
     * từ Mode 2 sang Mode 4, GUI có thể hiển thị thoáng qua giá trị error
     * CŨ còn sót lại từ Mode 2 cho tới khi Mode 4 nhận được error thật
     * đầu tiên từ PC. Reset cho sạch, đồng bộ cách LinePID_Init() làm. */
    error = 0;
    last_error = 0;

    Set_Motors_Compensated(0, 0);
}

void Mode4_AI_SetError(int error_val)
{
    ai_error = error_val;
    if (ai_error != AI_NO_LINE_SENTINEL) {
        line_detected = 1;
    }
}

void Mode4_AI_Update(void)
{
    if (!line_detected) {
        Set_Motors_Compensated(0, 0);
        return;
    }

    if (ai_error == AI_NO_LINE_SENTINEL) {
        lost_count++;
        if (lost_count > AI_MAX_LOST_CYCLES) {
            Set_Motors_Compensated(0, 0);
            I_term = 0.0f;
            line_detected = 0;
            lost_count = 0;
            last_turn_direction = 0;
            /* FIX BUG 4: reset D-term reference khi bỏ cuộc tìm line -
             * lần bám lại line kế tiếp (line_detected=1 mới) sẽ không bị
             * giật D-term vì last_ai_error đã về 0 từ trước, giống hệt
             * hành vi Mode4_AI_Init(). */
            last_ai_error = 0.0f;
            return;
        }
        int pwm_l = (last_turn_direction >= 0) ? AI_PIVOT_SPEED : -AI_PIVOT_SPEED;
        int pwm_r = -pwm_l;
        Set_Motors_Compensated(pwm_l, pwm_r);
        return;
    }

    /* FIX BUG 3: dùng THẲNG error liên tục (-4.00..4.00) cho PID, KHÔNG
     * lượng tử hoá về bước nguyên -4..4 như trước (đoạn "if (e==0 &&
     * ai_error!=0) e = ±1" cũ khiến MỌI lệch dù chỉ 0.01 nhảy thẳng lên
     * step 1 - y hệt bang-bang, xoá sạch lợi thế continuous regression
     * của AI so với 5 mắt IR). Range giữ nguyên -4.00..4.00 như bản cũ
     * (chỉ khác continuous vs stepped) nên Kp/Ki/Kd KHÔNG cần retune. */
    float e = ai_error / 100.0f;

    /* error/last_error (global int, dùng chung protocol "LOG,...*100"
     * với main.c) CHỈ để hiển thị telemetry - làm tròn về gần nhất, mất
     * phần thập phân là chấp nhận được vì KHÔNG dùng để tính PID. PID
     * thật sự chạy trên `e`/`last_ai_error` (float) giữ nguyên độ chính
     * xác của model bên dưới. */
    error = (int)lroundf(e);
    last_error = (int)lroundf(last_ai_error);

    lost_count = 0;

    if (e > 0.05f) last_turn_direction = 1;
    else if (e < -0.05f) last_turn_direction = -1;

    float p_term = Kp * e;
    I_term += Ki * e;
    if (I_term > 200.0f) I_term = 200.0f;
    else if (I_term < -200.0f) I_term = -200.0f;

    float d_term = Kd * (e - last_ai_error);
    last_ai_error = e;

    int pid_value = (int)(p_term + I_term + d_term);
    float abs_e = (e >= 0.0f) ? e : -e;

    /* Phanh tự động khi vào cua - nay ĐÃ CÓ TÁC DỤNG THẬT (xem giải thích
     * ở định nghĩa AI_BASE_SPEED/AI_MIN_BASE_SPEED phía trên). */
    int base_pwm = base_speed - (int)(abs_e * 25.0f);
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
