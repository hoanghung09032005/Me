#ifndef MODE2_LINE_PID_H
#define MODE2_LINE_PID_H

#include <stdint.h>

/* ============================================================================
 * Bám line bằng PID + xử lý bẻ góc vuông 90 độ tại giao lộ.
 * Đây là logic "lái xe khi KHÔNG có vật cản phía trước" - hoàn toàn KHÔNG
 * biết gì về việc né vật cản (mode2_avoid.c/h là module riêng, độc lập).
 * ============================================================================ */

void LinePID_Init(void);

/* Gọi mỗi tick (10ms) khi KHÔNG có vật cản/không đang né - tự đọc cảm
 * biến, chạy PID, xử lý góc vuông, xuất PWM qua Set_Motors_Compensated(). */
void LinePID_Run(uint8_t raw_state, uint8_t side_left, uint8_t side_right);

void LinePID_SetBaseSpeedPercent(uint8_t speed_pct);

/* Dùng bởi mode2_obstacle.c để kiểm tra Base_Speed<=0 (chế độ "khoá ga"
 * qua lệnh V0 từ GUI) TRƯỚC khi quyết định có chạy né/PID hay không. */
int LinePID_GetBaseSpeed(void);

/* Gọi bởi mode2_avoid.c NGAY LÚC vừa bám lại được line sau khi né vật cản
 * xong - reset đúng các biến trạng thái nội bộ của PID (line_detected,
 * error, last_error, I_term) giống hệt cách PID tự làm khi tìm lại line
 * sau khi bị mất tạm thời. Đặt thành hàm thay vì để mode2_avoid.c tự ý
 * đụng vào biến nội bộ của module này - tránh đúng kiểu lỗi "protocol
 * mismatch giữa các file" đã từng gặp trong project. */
void LinePID_NotifyLineReacquired(void);

#endif
