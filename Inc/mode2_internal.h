#ifndef MODE2_INTERNAL_H
#define MODE2_INTERNAL_H

/* ============================================================================
 * Header NỘI BỘ - CHỈ dùng chung giữa 3 file: mode2_obstacle.c,
 * mode2_line_pid.c, mode2_avoid.c. KHÔNG include từ main.c hay bất kỳ file
 * nào khác - đây không phải API công khai của Mode 2 (API công khai nằm ở
 * mode2_obstacle.h, không đổi gì khi refactor này).
 * ============================================================================ */

/* Định nghĩa trong mode2_obstacle.c (file "gốc" của Mode 2, giữ chung
 * AUTO_TELE_DIV/auto_tele_count cho toàn bộ Mode 2). Cả mode2_line_pid.c
 * lẫn mode2_avoid.c đều gọi hàm này thay vì gọi thẳng Set_Motor_Outputs()
 * của hardware.c - đảm bảo MỌI lệnh động cơ trong Mode 2 luôn đi qua đúng
 * 1 chỗ bù trừ + throttle telemetry duy nhất. */
void Set_Motors_Compensated(int pwm_l, int pwm_r);

#endif
