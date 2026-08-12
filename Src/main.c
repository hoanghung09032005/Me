#include "stm32f1xx.h"
#include "hardware.h"
#include "mode1.h"
#include "mode2_obstacle.h"
#include <stdlib.h>
#include <stdio.h>

#define MODE_IDLE   0   /* MỚI: trạng thái mặc định an toàn - đứng im, chờ lệnh */
#define MODE_MANUAL 1
#define MODE_AUTO   2

volatile int car_mode = MODE_IDLE;   /* SỬA: không còn mặc định = MODE_AUTO */
volatile int error = 0, last_error = 0, log_pwm_l = 0, log_pwm_r = 0, telemetry_ready = 0;
volatile uint8_t raw_state = 0;

#define RX_BUF_SIZE 32
char rx_buf[RX_BUF_SIZE];
volatile int rx_idx = 0;
volatile int cmd_ready = 0;

void USART1_IRQHandler(void) {
    if (USART1->SR & USART_SR_RXNE) {
        char c = (char)(USART1->DR & 0xFF);

        if (c == '\n' || c == '\r') {
            if (rx_idx > 0) {
                rx_buf[rx_idx] = '\0';
                cmd_ready = 1;
            }
        } else {
            if (rx_idx < RX_BUF_SIZE - 1) {
                rx_buf[rx_idx++] = c;
            }
        }
    }
}

void TIM3_IRQHandler(void) {
    if (!(TIM3->SR & TIM_SR_UIF)) return;
    TIM3->SR &= ~TIM_SR_UIF;

    raw_state = 0;
    if (GPIOB->IDR & (1U << 12)) raw_state |= (1U << 0);
    if (GPIOB->IDR & (1U << 11)) raw_state |= (1U << 1);
    if (GPIOB->IDR & (1U << 10)) raw_state |= (1U << 2);
    if (GPIOB->IDR & (1U << 9))  raw_state |= (1U << 3);
    if (GPIOB->IDR & (1U << 8))  raw_state |= (1U << 4);

    uint8_t side_left  = (GPIOB->IDR & (1U << 13)) ? 1 : 0;
    uint8_t side_right = (GPIOB->IDR & (1U << 14)) ? 1 : 0;

    if (car_mode == MODE_MANUAL) {
        Mode1_Update();
    }
    else if (car_mode == MODE_AUTO) {
        Mode2_Obstacle_Update(raw_state, side_left, side_right);
    }
    else {
        /* MODE_IDLE - đứng im, đây là trạng thái mặc định lúc boot */
        Set_Motor_Outputs(0, 0);
    }
}

int main(void) {
    SystemClock_Config();
    GPIO_Config();
    TIM_Init();
    USART1_Init();

    Mode1_Init();
    Mode2_Obstacle_Init();

    UART_SendString("\n[STM32] San sang! Cho lenh...\n");

    while (1) {
        if (cmd_ready) {
            char cmd_type = rx_buf[0];

            if (cmd_type == 'A') {
                car_mode = MODE_AUTO;
                UART_SendString("ACK:AUTO\n");
            }
            else if (cmd_type == 'M') {
                car_mode = MODE_MANUAL;
                Mode1_Init();   /* reset biến manual mỗi lần vào lại MANUAL */
                UART_SendString("ACK:MANUAL\n");
            }
            else if (cmd_type == 'S') {
                car_mode = MODE_IDLE;
                Set_Motor_Outputs(0, 0);
                UART_SendString("ACK:STOP\n");
            }
            else if (car_mode == MODE_MANUAL) {
                int speed = 0;
                if (rx_idx > 2) {
                    speed = atoi(&rx_buf[2]);
                }
                Mode1_Apply_Command(cmd_type, speed);
                /* Không ACK từng lệnh lái tay - communication.py gửi lệnh
                 * này lặp lại mỗi 120ms (MANUAL_REPEAT_MS), ACK liên tục
                 * sẽ làm ngập UART không cần thiết. */
            }
            else {
                UART_SendString("ERR:UNKNOWN_CMD\n");
            }

            rx_idx = 0;
            cmd_ready = 0;
        }

        if (telemetry_ready) {
            telemetry_ready = 0;

            char tx_buf[64];
            sprintf(tx_buf, "LOG,%02X,%d,%d,%d\n",
                    raw_state, last_error * 100, log_pwm_l, log_pwm_r);
            UART_SendString(tx_buf);
        }
    }
}
