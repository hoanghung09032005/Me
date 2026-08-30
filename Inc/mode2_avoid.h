#ifndef MODE2_AVOID_H
#define MODE2_AVOID_H

#include <stdint.h>

/* ============================================================================
 * State machine né vật cản: tự quét siêu âm phát hiện vật cản phía trước,
 * lách qua, quay lại bám line. Độc lập với PID bám line/góc vuông
 * (mode2_line_pid.c/h) - chỉ gọi NGƯỢC LẠI qua đúng 1 hàm duy nhất:
 * LinePID_NotifyLineReacquired() khi vừa bám lại được line.
 * ============================================================================ */

void Avoid_Init(void);

/* Gọi MỖI TICK (10ms) khi Base_Speed > 0 (đã kiểm tra ở mode2_obstacle.c).
 * Tự đọc siêu âm/servo, tự quyết định có kích hoạt né hay không.
 * Trả về 1 nếu module này ĐANG chiếm quyền điều khiển động cơ ở tick này
 * (đã tự gọi Set_Motors_Compensated bên trong) - người gọi KHÔNG được
 * chạy LinePID_Run() nữa cho tick này.
 * Trả về 0 nếu không có vật cản/không đang né - người gọi cần tự chạy
 * LinePID_Run() như bình thường. */
int Avoid_Update(uint8_t raw_state, uint8_t side_left, uint8_t side_right);

#endif
