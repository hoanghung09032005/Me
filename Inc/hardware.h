#ifndef HARDWARE_H
#define HARDWARE_H

#include "stm32f1xx.h"
#include <stdint.h>

void SystemClock_Config(void);
void GPIO_Config(void);
void TIM_Init(void);
void USART1_Init(void);
void Set_Motor_Outputs(int pwm_l, int pwm_r);
float HCSR04_ReadDistance_cm(void);
void HCSR04_RequestMeasurement(void);
void HCSR04_Service(void);
int16_t HCSR04_GetDistance_cm_x10(void);

void DHT22_Service(void);
int16_t DHT22_GetTemperature_c_x10(void);
int16_t DHT22_GetHumidity_rh_x10(void);
void Servo_SetAngle(uint16_t deg);

void UART_SendChar(char c);
void UART_SendString(char *str);

#endif // HARDWARE_H
