#include "stm32f1xx.h"
#include "hardware.h"
#include "mode1.h"
#include "mode2_obstacle.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define MODE_IDLE       0
#define MODE_MANUAL     1
#define MODE_AUTO       2

#define RX_BUF_SIZE     32
#define DHT22_PERIOD_TICKS  200U   /* 2 seconds at the 10 ms TIM3 tick. */

volatile int car_mode = MODE_IDLE;
volatile int error = 0;
volatile int last_error = 0;
volatile int log_pwm_l = 0;
volatile int log_pwm_r = 0;
volatile int log_distance_cm_x10 = -1;
volatile int telemetry_ready = 0;
volatile uint8_t raw_state = 0;
volatile uint32_t control_ticks = 0;

/* The ISR assembles one command while the foreground processes another.
 * If the foreground has not consumed a complete command yet, the next complete
 * line is intentionally dropped instead of corrupting the pending command. */
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
            /* Emergency stop works from IDLE, MANUAL and AUTO. */
            Stop_All();
            break;

        case 'M':
            car_mode = MODE_MANUAL;
            Mode2_Obstacle_Init();
            Mode1_Init();
            telemetry_ready = 1;
            UART_SendString("ACK,M\n");
            break;

        case 'A':
            Mode1_Init();
            Mode2_Obstacle_Init();
            HCSR04_RequestMeasurement();
            car_mode = MODE_AUTO;
            telemetry_ready = 1;
            UART_SendString("ACK,A\n");
            break;

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

        default:
            /* Ignore malformed commands instead of changing motor state. */
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
    if (GPIOB->IDR & (1U << 12)) raw_state |= (1U << 0);
    if (GPIOB->IDR & (1U << 11)) raw_state |= (1U << 1);
    if (GPIOB->IDR & (1U << 10)) raw_state |= (1U << 2);
    if (GPIOB->IDR & (1U << 9))  raw_state |= (1U << 3);
    if (GPIOB->IDR & (1U << 8))  raw_state |= (1U << 4);

    uint8_t side_left = (GPIOB->IDR & (1U << 13)) ? 1U : 0U;
    uint8_t side_right = (GPIOB->IDR & (1U << 14)) ? 1U : 0U;

    if (car_mode == MODE_MANUAL) {
        Mode1_Update();
    } else if (car_mode == MODE_AUTO) {
        Mode2_Obstacle_Update(raw_state, side_left, side_right);
    } else {
        Set_Motor_Outputs(0, 0);
    }
}

int main(void)
{
    char command[RX_BUF_SIZE];
    uint32_t last_dht22_tick = 0;

    SystemClock_Config();
    GPIO_Config();
    TIM_Init();
    USART1_Init();

    Mode1_Init();
    Mode2_Obstacle_Init();
    DHT22_Service();

    {
        char boot_message[32];
        snprintf(boot_message, sizeof(boot_message),
                 "STM32,BOOT,%luHz\n", (unsigned long)SystemCoreClock);
        UART_SendString(boot_message);
    }

    while (1) {
        if (Take_Command(command)) {
            Process_Command(command);
        }

        /* These polling drivers never execute in the PID interrupt. */
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
                     "LOG,%u,%d,%d,%d,%d,%d,%d\n",
                     (unsigned int)raw_state,
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
