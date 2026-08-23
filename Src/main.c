#include "stm32f1xx.h"
#include "hardware.h"
#include "mode1.h"
#include "mode2_obstacle.h"
#include "mode3_follow.h"
#include "mode4_ai.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define MODE_IDLE       0
#define MODE_MANUAL     1
#define MODE_AUTO       2
#define MODE_FOLLOW     3
#define MODE_AI_LINE    4

#define RX_BUF_SIZE     32
#define DHT22_PERIOD_TICKS  200U
#define IWDG_TIMEOUT_MS      500U
#define AUTO_LINK_TIMEOUT_TICKS  150U

#define LINE_MID_ACTIVE_LOW     1
#define LINE_SIDE_ACTIVE_LOW    0

#if LINE_MID_ACTIVE_LOW
    #define MID_SEES_LINE(pinmask)   (!(GPIOB->IDR & (pinmask)))
#else
    #define MID_SEES_LINE(pinmask)   ((GPIOB->IDR & (pinmask)) != 0U)
#endif

#if LINE_SIDE_ACTIVE_LOW
    #define SIDE_SEES_LINE(pinmask)  (!(GPIOB->IDR & (pinmask)))
#else
    #define SIDE_SEES_LINE(pinmask)  ((GPIOB->IDR & (pinmask)) != 0U)
#endif

volatile int car_mode = MODE_IDLE;
volatile int error = 0;
volatile int last_error = 0;
volatile int log_pwm_l = 0;
volatile int log_pwm_r = 0;
volatile int log_distance_cm_x10 = -1;
volatile int telemetry_ready = 0;
volatile uint8_t raw_state = 0;
volatile uint8_t side_left_state = 0;
volatile uint8_t side_right_state = 0;
volatile uint32_t control_ticks = 0;
volatile uint32_t last_command_tick = 0;

volatile uint8_t side_edge_flag = 0;
volatile int8_t  side_edge_last_dir = 0;

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1U << 13)) {
        EXTI->PR = (1U << 13);
        side_edge_flag = 1;
        if (SIDE_SEES_LINE(1U << 13)) {
            side_edge_last_dir = -1;
        }
    }
    if (EXTI->PR & (1U << 14)) {
        EXTI->PR = (1U << 14);
        side_edge_flag = 1;
        if (SIDE_SEES_LINE(1U << 14)) {
            side_edge_last_dir = 1;
        }
    }
}

uint8_t Side_EdgeEvent_TakeAndClear(int8_t *out_dir)
{
    uint8_t happened;
    __disable_irq();
    happened = side_edge_flag;
    if (out_dir) {
        *out_dir = side_edge_last_dir;
    }
    side_edge_flag = 0;
    side_edge_last_dir = 0;
    __enable_irq();
    return happened;
}

static volatile char rx_assembly[RX_BUF_SIZE];
static volatile char rx_command[RX_BUF_SIZE];
static volatile uint8_t rx_idx = 0;
static volatile uint8_t rx_command_len = 0;
static volatile uint8_t cmd_ready = 0;

static void Stop_All(void)
{
    car_mode = MODE_IDLE;
    Mode1_Init();
    Mode2_Obstacle_Init();
    Mode3_Follow_Init();
    Mode4_AI_Init();
    log_pwm_l = 0;
    log_pwm_r = 0;
    log_distance_cm_x10 = HCSR04_GetDistance_cm_x10();
    telemetry_ready = 1;
    UART_SendString("ACK,S\n");
}

static uint8_t Take_Command(char *destination)
{
    uint8_t length;
    __disable_irq();
    if (!cmd_ready) {
        __enable_irq();
        return 0;
    }
    length = rx_command_len;
    for (uint8_t i = 0; i <= length; i++) {
        destination[i] = rx_command[i];
    }
    cmd_ready = 0;
    __enable_irq();
    return 1;
}

static void Process_Command(const char *command)
{
    char command_type = command[0];

    switch (command_type) {
        case 'S':
            Stop_All();
            break;

        case 'M':
            car_mode = MODE_MANUAL;
            Mode2_Obstacle_Init();
            Mode3_Follow_Init();
            Mode4_AI_Init();
            Mode1_Init();
            telemetry_ready = 1;
            UART_SendString("ACK,M\n");
            break;

        case 'A':
            Mode1_Init();
            Mode3_Follow_Init();
            Mode4_AI_Init();
            Mode2_Obstacle_Init();
            HCSR04_RequestMeasurement();
            car_mode = MODE_AUTO;
            telemetry_ready = 1;
            UART_SendString("ACK,A\n");
            break;

        case 'O': {
            /* Mode 3: Bám vật thể */
            /* Phân biệt: "O\n" = Siêu âm tự quét, "O <x> <y>\n" = AI Camera */
            int has_args = 0;
            int x_err = 0, y_err = 0;

            if (command[1] == ' ' || command[1] == '\t') {
                /* Có tham số: "O <x> <y>" */
                char *p = (char *)command + 1;
                x_err = (int)strtol(p, &p, 10);
                y_err = (int)strtol(p, NULL, 10);
                has_args = 1;
            }

            /* FIX BUG 1: Chỉ full-init khi mới vào MODE_FOLLOW lần đầu.
             * Nếu đã ở MODE_FOLLOW, chỉ cập nhật error — tránh re-init
             * làm servo giật về giữa và motor bị zero-pulse mỗi frame. */
            if (car_mode == MODE_FOLLOW) {
                if (has_args) {
                    Mode3_Follow_SetError(x_err, y_err);
                }
            } else {
                Mode1_Init();
                Mode2_Obstacle_Init();
                Mode4_AI_Init();
                Mode3_Follow_Init();
                if (has_args) {
                    Mode3_Follow_SetError(x_err, y_err);
                }
                car_mode = MODE_FOLLOW;
                telemetry_ready = 1;
            }
            UART_SendString("ACK,O\n");
            break;
        }

        case 'I': {
            /* Mode 4: AI nhận diện line */
            long ai_error = strtol(command + 1, NULL, 10);
            if (car_mode == MODE_AI_LINE) {
                Mode4_AI_SetError((int)ai_error);
            } else {
                Mode1_Init();
                Mode2_Obstacle_Init();
                Mode3_Follow_Init();
                Mode4_AI_Init();
                Mode4_AI_SetError((int)ai_error);
                car_mode = MODE_AI_LINE;
                telemetry_ready = 1;
            }
            UART_SendString("ACK,I\n");
            break;
        }

        case 'V': {
            long speed_pct = strtol(command + 1, NULL, 10);
            if (speed_pct < 0) speed_pct = 0;
            if (speed_pct > 100) speed_pct = 100;
            Mode2_Obstacle_SetSpeedPercent((uint8_t)speed_pct);
            UART_SendString("ACK,V\n");
            break;
        }

        case 'F':
        case 'B':
        case 'L':
        case 'R': {
            if (car_mode == MODE_MANUAL) {
                long speed_pct = strtol(command + 1, NULL, 10);
                if (speed_pct < 0) speed_pct = 0;
                if (speed_pct > 100) speed_pct = 100;
                Mode1_Apply_Command(command_type, (int)speed_pct);
                UART_SendString("ACK,DRIVE\n");
            }
            break;
        }

        case 'H':
            UART_SendString("ACK,H\n");
            break;

        default:
            break;
    }
}

void USART1_IRQHandler(void)
{
    if (!(USART1->SR & USART_SR_RXNE)) {
        return;
    }
    char c = (char)(USART1->DR & 0xFFU);
    if (c == '\n' || c == '\r') {
        if (rx_idx > 0 && !cmd_ready) {
            for (uint8_t i = 0; i < rx_idx; i++) {
                rx_command[i] = rx_assembly[i];
            }
            rx_command[rx_idx] = '\0';
            rx_command_len = rx_idx;
            cmd_ready = 1;
        }
        rx_idx = 0;
    } else if (rx_idx < (RX_BUF_SIZE - 1U)) {
        rx_assembly[rx_idx++] = c;
    }
}

void TIM3_IRQHandler(void)
{
    if (!(TIM3->SR & TIM_SR_UIF)) {
        return;
    }
    TIM3->SR &= ~TIM_SR_UIF;
    control_ticks++;

    raw_state = 0;
    if (MID_SEES_LINE(1U << 12)) raw_state |= (1U << 0);
    if (MID_SEES_LINE(1U << 11)) raw_state |= (1U << 1);
    if (MID_SEES_LINE(1U << 10)) raw_state |= (1U << 2);
    if (MID_SEES_LINE(1U << 9))  raw_state |= (1U << 3);
    if (MID_SEES_LINE(1U << 8))  raw_state |= (1U << 4);

    uint8_t side_left  = SIDE_SEES_LINE(1U << 13) ? 1U : 0U;
    uint8_t side_right = SIDE_SEES_LINE(1U << 14) ? 1U : 0U;
    side_left_state = side_left;
    side_right_state = side_right;

    if (car_mode == MODE_MANUAL) {
        Mode1_Update();
    } else if (car_mode == MODE_AUTO) {
        if ((uint32_t)(control_ticks - last_command_tick) > AUTO_LINK_TIMEOUT_TICKS) {
            car_mode = MODE_IDLE;
            Mode2_Obstacle_Init();
        } else {
            Mode2_Obstacle_Update(raw_state, side_left, side_right);
        }
    } else if (car_mode == MODE_FOLLOW) {
        if ((uint32_t)(control_ticks - last_command_tick) > AUTO_LINK_TIMEOUT_TICKS) {
            car_mode = MODE_IDLE;
            Mode3_Follow_Init();
        } else {
            Mode3_Follow_Update();
        }
    } else if (car_mode == MODE_AI_LINE) {
        if ((uint32_t)(control_ticks - last_command_tick) > AUTO_LINK_TIMEOUT_TICKS) {
            car_mode = MODE_IDLE;
            Mode4_AI_Init();
        } else {
            Mode4_AI_Update();
        }
    } else {
        Set_Motor_Outputs(0, 0);
    }
}

static void Report_Reset_Cause(void)
{
    uint32_t reset_cause = RCC->CSR;
    const char *reason;

    if (reset_cause & RCC_CSR_PORRSTF) {
        reason = "POWER_DIP";
    } else if (reset_cause & RCC_CSR_IWDGRSTF) {
        reason = "IWDG";
    } else if (reset_cause & RCC_CSR_WWDGRSTF) {
        reason = "WWDG";
    } else if (reset_cause & RCC_CSR_SFTRSTF) {
        reason = "SOFTWARE";
    } else if (reset_cause & RCC_CSR_LPWRRSTF) {
        reason = "LOW_POWER";
    } else if (reset_cause & RCC_CSR_PINRSTF) {
        reason = "NRST_PIN";
    } else {
        reason = "UNKNOWN";
    }

    RCC->CSR |= RCC_CSR_RMVF;

    char reset_message[40];
    snprintf(reset_message, sizeof(reset_message), "STM32,RESET_CAUSE,%s\n", reason);
    UART_SendString(reset_message);
}

int main(void)
{
    char command[RX_BUF_SIZE];
    uint32_t last_dht22_tick = 0;

    SystemClock_Config();
    GPIO_Config();
    EXTI_SideSensors_Init();
    TIM_Init();
    USART1_Init();

    Report_Reset_Cause();

    Mode1_Init();
    Mode2_Obstacle_Init();
    Mode3_Follow_Init();
    Mode4_AI_Init();
    DHT22_Service();

    {
        char boot_message[32];
        snprintf(boot_message, sizeof(boot_message),
                 "STM32,BOOT,%luHz\n", (unsigned long)SystemCoreClock);
        UART_SendString(boot_message);
    }

    IWDG_Init(IWDG_TIMEOUT_MS);

    while (1) {
        IWDG_Refresh();

        if (Take_Command(command)) {
            last_command_tick = control_ticks;
            Process_Command(command);
        }

        HCSR04_Service();

        uint32_t now = control_ticks;
        if ((uint32_t)(now - last_dht22_tick) >= DHT22_PERIOD_TICKS) {
            last_dht22_tick = now;
            DHT22_Service();
        }

        if (telemetry_ready) {
            char tx_buffer[96];
            int error_x100 = error * 100;

            telemetry_ready = 0;
            snprintf(tx_buffer, sizeof(tx_buffer),
                     "LOG,%u,%u,%u,%d,%d,%d,%d,%d,%d\n",
                     (unsigned int)raw_state,
                     (unsigned int)side_left_state,
                     (unsigned int)side_right_state,
                     error_x100,
                     log_pwm_l,
                     log_pwm_r,
                     log_distance_cm_x10,
                     DHT22_GetTemperature_c_x10(),
                     DHT22_GetHumidity_rh_x10());
            UART_SendString(tx_buffer);
        }
    }
}
