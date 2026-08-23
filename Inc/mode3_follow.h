#ifndef MODE3_FOLLOW_H
#define MODE3_FOLLOW_H

#include <stdint.h>

/* ============================================================================
 * Mode 3: Bám vật thể — 2 cách:
 *   • "Siêu âm"  : STM32 tự quét servo + HC-SR04 (lệnh "O")
 *   • "AI Camera": PC nhận diện vật qua OpenCV, gửi error xuống (lệnh "O <x> <y>")
 *
 * Khi ở AI mode, STM32 nhận error từ PC qua Mode3_Follow_SetError()
 * và điều khiển xe theo P-controller (không dùng servo quét nữa).
 * ============================================================================ */

void Mode3_Follow_Init(void);

/* Gọi mỗi tick (10ms) khi car_mode == MODE_FOLLOW. */
void Mode3_Follow_Update(void);

/* Nhận error từ PC khi chọn AI Camera mode. */
void Mode3_Follow_SetError(int x_err, int y_err);

#endif
