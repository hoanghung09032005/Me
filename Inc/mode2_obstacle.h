#ifndef MODE2_OBSTACLE_H
#define MODE2_OBSTACLE_H

#include <stdint.h>

void Mode2_Obstacle_Init(void);
void Mode2_Obstacle_Update(uint8_t raw_state, uint8_t side_left, uint8_t side_right);

#endif // MODE2_OBSTACLE_H
