#ifndef MODE4_AI_H
#define MODE4_AI_H

#include <stdint.h>

/* ============================================================================
 * Mode 4: AI nhận diện line qua video (thay thế cảm biến IR).
 * ----------------------------------------------------------------------------
 * Nhận error trực tiếp từ PC qua lệnh 'I <error_x100>'.
 * Chạy PID y hệt mode2_line_pid.c nhưng error do PC tính từ camera.
 * ============================================================================ */

/* FIX BUG 2: Giá trị đặc biệt báo "không thấy line" từ PC.
 * Dải error hợp lệ là -400..400 (tương ứng -4.00..4.00).
 * Dùng 9999 làm sentinel để phân biệt với error=0 (đi đúng giữa). */
#define AI_NO_LINE_SENTINEL   9999

void Mode4_AI_Init(void);

/* Gọi mỗi tick (10ms) khi car_mode == MODE_AI_LINE. */
void Mode4_AI_Update(void);

/* Gọi từ main.c khi parse lệnh 'I <error>' */
void Mode4_AI_SetError(int error);

#endif
