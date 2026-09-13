#ifndef MODE3_FOLLOW_H
#define MODE3_FOLLOW_H

#include <stdint.h>

/* ============================================================================
 * Mode 3: Bám vật thể bằng servo quét + HC-SR04 (LOCK/TRACK, lệnh "O").
 *
 * [SUA] Nhánh "AI Camera" (lệnh "O <x> <y>" gọi Mode3_Follow_SetError) đã
 * bị VÔ HIỆU HÓA có chủ đích - hàm này giờ là stub rỗng trong
 * mode3_follow.c, KHÔNG còn điều khiển xe theo P-controller như comment
 * cũ mô tả. Khai báo được giữ lại CHỈ để không phá API công khai mà
 * main.c/communication.py vẫn còn gọi tới; có thể xóa hẳn nếu sau này dọn
 * dẹp luôn 2 phía đó.
 * ============================================================================ */

void Mode3_Follow_Init(void);

/* Gọi mỗi tick (10ms) khi car_mode == MODE_FOLLOW. */
void Mode3_Follow_Update(void);

/* [SUA] Stub rỗng - xem giải thích ở đầu file. */
void Mode3_Follow_SetError(int x_err, int y_err);

/* [THEM] Hiệu chỉnh runtime qua lệnh UART "C3 <target_cm> <tol_cm> <max_cm>
 * <move_pwm> <turn_pwm>" - main.c gọi hàm này khi nhận lệnh 'C'. Tự clamp
 * về giới hạn an toàn bên trong, xem mode3_follow.c. */
void Mode3_Follow_SetTuning(int target_cm, int tolerance_cm, int max_cm,
                             int move_pwm, int turn_pwm);

#endif
