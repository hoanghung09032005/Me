#ifndef MODE2_OBSTACLE_H
#define MODE2_OBSTACLE_H

#include <stdint.h>

/* API công khai của Mode 2 - KHÔNG đổi gì so với trước khi tách file,
 * nên main.c không cần sửa bất kỳ dòng nào. Logic thật giờ nằm trong
 * mode2_line_pid.c/h và mode2_avoid.c/h - xem mode2_obstacle.c để biết
 * cách chúng phối hợp. */

void Mode2_Obstacle_Init(void);
void Mode2_Obstacle_Update(uint8_t raw_state, uint8_t side_left, uint8_t side_right);
void Mode2_Obstacle_SetSpeedPercent(uint8_t speed_pct);

#endif // MODE2_OBSTACLE_H
