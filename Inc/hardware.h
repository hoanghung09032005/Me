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

/* ======================================================================
 * BÙ TRỪ LỆCH MOTOR (MOTOR RIGHT COMPENSATION)
 * ----------------------------------------------------------------------
 * Thực trạng đo được trên xe thật: bánh phải quay nhanh/mạnh hơn bánh
 * trái 10-20% ở cùng 1 giá trị PWM - đây là ĐẶC TÍNH PHẦN CỨNG (dung sai
 * chế tạo motor/bánh xe/ma sát trục), KHÔNG liên quan gì tới thuật toán
 * điều khiển. Vì vậy hằng số này đặt DUY NHẤT ở đây (hardware.h) - dùng
 * CHUNG cho cả Mode 1 (lái tay, mode1.c) lẫn Mode 2 (tự động, xử lý
 * trong mode2_obstacle.c) - để xe có ĐÚNG 1 "tính cách" lái nhất quán ở
 * mọi mode, không bị lệch trái/phải khác nhau tuỳ đang ở mode nào.
 *
 * Nếu chạy thử thấy xe đi thẳng vẫn xỉa trái/phải, CHỈ cần chỉnh đúng
 * con số này ở MỘT chỗ duy nhất - không sửa riêng lẻ trong mode1.c hay
 * mode2_obstacle.c, tránh lặp lại đúng kiểu lỗi "mismatch giữa các file"
 * đã từng gặp trong project này. */
#define MOTOR_RIGHT_COMPENSATION_PCT 95

/* Independent watchdog (IWDG). Runs off the internal ~40 kHz LSI, so it
 * keeps counting even if the main HSE/PLL clock glitches from a supply
 * brownout. If main()'s loop ever stops calling IWDG_Refresh() in time
 * (e.g. the CPU got stuck after a marginal brownout that wasn't clean
 * enough to trigger a full power-on reset), the watchdog forces a reset
 * back to a known-safe boot state (car_mode = MODE_IDLE, motors off). */
void IWDG_Init(uint16_t timeout_ms);
void IWDG_Refresh(void);

/* EXTI trên 2 mắt biên (PB13/PB14): bắt cạnh tín hiệu ở tầng PHẦN CỨNG
 * (vài trăm ns) thay vì chỉ dựa vào TIM3_IRQHandler lấy mẫu mức GPIO mỗi
 * 10ms. Mục đích: khi xe đi đủ nhanh qua vạch ngang ở góc cua, xung có
 * thể ngắn hơn 1 chu kỳ TIM3 (10ms) và "lọt" giữa 2 lần lấy mẫu
 * (aliasing) - ISR đăng ký ở đây không thể bỏ lỡ dù xe đi nhanh cỡ nào.
 * Việc "chốt" sự kiện (latch) và đọc polarity cảm biến nằm ở main.c (nơi
 * đã có sẵn macro SIDE_SEES_LINE), hàm này ở hardware.c CHỈ lo phần thuần
 * cấu hình ngoại vi (AFIO/EXTI/NVIC), không biết gì về polarity/ý nghĩa
 * tín hiệu - giữ đúng ranh giới hardware.c (driver) vs main.c (logic). */
void EXTI_SideSensors_Init(void);

#endif // HARDWARE_H
