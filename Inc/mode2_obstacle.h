#ifndef MODE2_OBSTACLE_H
#define MODE2_OBSTACLE_H

#include <stdint.h>

void Mode2_Obstacle_Init(void);
void Mode2_Obstacle_Update(uint8_t raw_state, uint8_t side_left, uint8_t side_right);
void Mode2_Obstacle_SetSpeedPercent(uint8_t speed_pct);

/* Bật/tắt hành vi NÉ VẬT CẢN (dừng - quét servo - lách) mà không đụng
 * PID bám line 5 mắt IR. Chỉ chuyển tiếp xuống Avoid_SetEnabled() trong
 * mode2_avoid.c (xem chi tiết trong header đó).
 *
 * Thêm cho Mode 4 (mode4_ai.py qua communication.link.set_obstacle_avoid()):
 * Mode 4 tái dùng PID bám line của Mode 2 nhưng tự xử lý vật cản/ghi nhãn
 * bằng camera ở phía PC, nên không muốn xe tự dừng-lách như Mode 2 gốc.
 *
 * LƯU Ý: Mode2_Obstacle_Init() (chạy khi vào lại Auto bằng lệnh "A") LUÔN
 * reset cờ này về "bật" - đây là mặc định AN TOÀN cho Mode 2 gốc
 * (mode2_auto.py không cần gọi hàm này). Bên gọi muốn tắt (Mode 4) phải
 * gửi lệnh tắt SAU KHI đã vào Auto, không phải trước. */
void Mode2_Obstacle_SetAvoidEnabled(uint8_t enabled);

#endif // MODE2_OBSTACLE_H
