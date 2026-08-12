#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

#include "hardware.h"
#include "mode1.h"
#include "mode2.h"

#define MODE_IDLE     0
#define MODE_AUTO     1
#define MODE_MANUAL   2

volatile int car_mode = MODE_IDLE;

// Các biến lưu trạng thái để gửi lên Python
volatile uint8_t raw_state = 0;
volatile int error = 0, last_error = 0;
volatile int log_pwm_l = 0, log_pwm_r = 0;
volatile int telemetry_ready = 0;

// Bộ đếm thời gian 10ms
void TIM3_IRQHandler() {
    if (!(TIM3->SR & TIM_SR_UIF)) return;
    TIM3->SR &= ~TIM_SR_UIF; // Xóa cờ ngắt

    raw_state = GPIOC->IDR & 0x1Fu; // Luôn đọc cảm biến

    if (car_mode == MODE_MANUAL) {
        Mode1_Update();
    }
    else if (car_mode == MODE_AUTO) {
        Mode2_Update(raw_state);
    }
    else {
        Set_Motor_Outputs(0, 0); // Đứng im
    }
}

int main(void) {
    System_Init();
    char tx_buf[64];
    char cmd[32];
    UART_SendString("\n[STM32] System Ready! Waiting for command...\n");

    while (1) {
        if (cmd_ready) {
            strncpy(cmd, (char*)rx_buffer, sizeof(cmd) - 1);
            cmd[sizeof(cmd) - 1] = '\0';
            cmd_ready = 0;

            // Nhận lệnh lái tay
            if (cmd[0] == 'M' && cmd[1] == ',' && cmd[3] == ',') {
                if (car_mode != MODE_MANUAL) {
                    car_mode = MODE_MANUAL;
                    Mode1_Init(); // Khởi tạo biến rác an toàn
                    GPIOC->BSRR = (1U << 29);
                    UART_SendString("ACK:MANUAL\n");
                }
                int spd = 0;
                for (char* p = cmd + 4; *p >= '0' && *p <= '9'; p++) {
                    spd = spd * 10 + (*p - '0');
                }
                Mode1_Apply_Command(cmd[2], spd);
            }
            // Chuyển chế độ lái tay
            else if (strncmp(cmd, "MANUAL", 6) == 0) {
                car_mode = MODE_MANUAL;
                Mode1_Init();
                GPIOC->BSRR = (1U << 29);
                UART_SendString("ACK:MANUAL\n");
            }
            // Chuyển chế độ Tự lái
            else if (strncmp(cmd, "START", 5) == 0) {
                car_mode = MODE_AUTO;
                Mode2_Init();
                GPIOC->BSRR = (1U << 29);
                UART_SendString("ACK:START\n");
            }
            // Phanh khẩn cấp
            else if (strncmp(cmd, "STOP", 4) == 0) {
                car_mode = MODE_IDLE;
                Set_Motor_Outputs(0, 0);
                GPIOC->BSRR = (1U << 13);
                UART_SendString("ACK:STOP\n");
            }
            // Ping
            else if (strncmp(cmd, "PING", 4) == 0) {
                sprintf(tx_buf, "ACK:PONG,%d\n", car_mode);
                UART_SendString(tx_buf);
            }
            else {
                UART_SendString("ERR:UNKNOWN_CMD\n");
            }
        }

        // Bắn gói tin trạng thái lên máy tính
        if (telemetry_ready && car_mode != MODE_IDLE) {
            sprintf(tx_buf, "LOG,%02X,%d,%d,%d\n", raw_state, last_error * 100, log_pwm_l, log_pwm_r);
            UART_SendString(tx_buf);
            telemetry_ready = 0;
        }
    }
}
