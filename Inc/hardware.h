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

/* Independent watchdog (IWDG). Runs off the internal ~40 kHz LSI, so it
 * keeps counting even if the main HSE/PLL clock glitches from a supply
 * brownout. If main()'s loop ever stops calling IWDG_Refresh() in time
 * (e.g. the CPU got stuck after a marginal brownout that wasn't clean
 * enough to trigger a full power-on reset), the watchdog forces a reset
 * back to a known-safe boot state (car_mode = MODE_IDLE, motors off). */
void IWDG_Init(uint16_t timeout_ms);
void IWDG_Refresh(void);

#endif // HARDWARE_H
