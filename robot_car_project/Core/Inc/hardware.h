#ifndef HARDWARE_H
#define HARDWARE_H

#include "stm32f4xx.h"

// Cho phép main.c đọc biến từ hardware.c
extern volatile char rx_buffer[32];
extern volatile int cmd_ready;

void System_Init(void);
void Set_Motor_Outputs(int s_l, int s_r);
void UART_SendChar(char c);
void UART_SendString(char* str);

#endif
