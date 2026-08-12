#include "hardware.h"

/* =========================================================================
 * 1. HỆ THỐNG CLOCK & UART
 * ========================================================================= */
void SystemClock_Config(void) {
    /*
     * Nếu bạn dùng STM32CubeMX sinh code, hãy copy phần SystemClock_Config
     * được sinh tự động vào đây (nếu cần thiết).
     * Với project Bare-metal chuẩn CMSIS, file system_stm32f10x.c đã tự động
     * cấu hình clock 72MHz trước khi gọi main(), nên có thể để trống.
     */
}

void USART1_Init(void) {
    // Cấp xung nhịp cho USART1 (kết nối ESP32-S3) và GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;

    // Cấu hình PA9 (TX) và PA10 (RX)
    GPIOA->CRH &= ~(0xFFU << 4);
    GPIOA->CRH |=  (0x8BU << 4);         // PA9: Alt Push-Pull 50MHz, PA10: Input
    GPIOA->ODR |=  (1U << 10);           // Pull-up PA10 để chống nhiễu RX

    // Cấu hình Baudrate = 115200 (Giả sử PCLK2 = 72MHz)
    // BRR = 72.000.000 / 115200 = 0x271
    USART1->BRR = 0x271;

    // Bật USART, TX, RX và Ngắt nhận dữ liệu (RXNEIE)
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    // Kích hoạt ngắt USART1
    NVIC_EnableIRQ(USART1_IRQn);
}

/* =========================================================================
 * 2. CẤU HÌNH GPIO CHUNG
 * ========================================================================= */
void GPIO_Config(void) {
    // Cấp clock cho PORT A, B và AFIO
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;

    // Giải phóng JTAG để dùng PB3, PB4
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

    // ---------------------------------------------------------
    // PWM MOTOR: PA8 = TIM1_CH1, PA11 = TIM1_CH4
    // ---------------------------------------------------------
    GPIOA->CRH &= ~((0xFU << 0) | (0xFU << 12));
    GPIOA->CRH |=  ((0xBU << 0) | (0xBU << 12));

    // ---------------------------------------------------------
    // SERVO: PA0 = TIM2_CH1, AF push-pull 50MHz
    // ---------------------------------------------------------
    GPIOA->CRL &= ~(0xFU << 0);
    GPIOA->CRL |=  (0xBU << 0);

    // ---------------------------------------------------------
    // HC-SR04: PA1 = Trig (output), PA4 = Echo (input)
    // ---------------------------------------------------------
    GPIOA->CRL &= ~(0xFU << 4);
    GPIOA->CRL |=  (0x2U << 4);          // PA1 output push-pull 2MHz
    GPIOA->BSRR = (1U << (1 + 16));      // Trig mặc định LOW

    GPIOA->CRL &= ~(0xFU << 16);
    GPIOA->CRL |=  (0x8U << 16);         // PA4 input pull-up/down
    GPIOA->BSRR = (1U << (4 + 16));      // Echo mặc định LOW (Pull-down)

    // ---------------------------------------------------------
    // MOTOR DIR: PB4, PB5, PB6, PB7 (Output push-pull)
    // ---------------------------------------------------------
    GPIOB->CRL &= ~((0xFU << 16) | (0xFU << 20) | (0xFU << 24) | (0xFU << 28));
    GPIOB->CRL |=  ((0x2U << 16) | (0x2U << 20) | (0x2U << 24) | (0x2U << 28));

    // ---------------------------------------------------------
    // SENSORS: PB8 -> PB14 (Input pull-up/down)
    // ---------------------------------------------------------
    GPIOB->CRH &= ~0x0FFFFFFF;
    GPIOB->CRH |=  0x08888888;
}

/* =========================================================================
 * 3. CẤU HÌNH TIMERS
 * ========================================================================= */
void TIM_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM4EN;

    // TIM1: PWM Motor (1kHz)
    TIM1->PSC = 72 - 1;
    TIM1->ARR = 999;
    TIM1->CCMR1 |= (6U << 4) | TIM_CCMR1_OC1PE;
    TIM1->CCMR2 |= (6U << 12) | TIM_CCMR2_OC4PE;
    TIM1->CCER  |= TIM_CCER_CC1E | TIM_CCER_CC4E;
    TIM1->BDTR  |= TIM_BDTR_MOE;
    TIM1->CR1   |= TIM_CR1_ARPE | TIM_CR1_CEN;

    // TIM3: Ngắt 10ms điều khiển PID
    TIM3->PSC = 7200 - 1;
    TIM3->ARR = 100 - 1;
    TIM3->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1 |= TIM_CR1_CEN;

    // TIM2: PWM 50Hz cho servo
    TIM2->PSC = 72 - 1;
    TIM2->ARR = 20000 - 1;
    TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE);
    TIM2->CCMR1 |=  (6U << 4) | TIM_CCMR1_OC1PE;
    TIM2->CCR1 = 1500;
    TIM2->CCER |= TIM_CCER_CC1E;
    TIM2->CR1  |= TIM_CR1_ARPE | TIM_CR1_CEN;

    // TIM4: Bộ đếm tự do 1MHz (1 tick = 1us) cho HC-SR04
    TIM4->PSC = 72 - 1;
    TIM4->ARR = 0xFFFF;
    TIM4->CR1 |= TIM_CR1_CEN;
}

/* =========================================================================
 * 4. DRIVER MOTOR, SERVO VÀ HC-SR04
 * ========================================================================= */
void Set_Motor_Outputs(int pwm_l, int pwm_r) {
    if (pwm_l >= 0) {
        GPIOB->BSRR = (1U << 5) | (1U << (6 + 16));
        TIM1->CCR4 = pwm_l;
    } else {
        GPIOB->BSRR = (1U << (5 + 16)) | (1U << 6);
        TIM1->CCR4 = -pwm_l;
    }

    if (pwm_r >= 0) {
        GPIOB->BSRR = (1U << 7) | (1U << (4 + 16));
        TIM1->CCR1 = pwm_r;
    } else {
        GPIOB->BSRR = (1U << (7 + 16)) | (1U << 4);
        TIM1->CCR1 = -pwm_r;
    }
}

void Servo_SetAngle(uint16_t deg) {
    if (deg > 180) deg = 180;
    TIM2->CCR1 = 1000 + ((uint32_t)deg * 1000) / 180;
}

#define ECHO_TIMEOUT_US   3000   /* Timeout 3ms tương ứng tầm quét ~50cm */

float HCSR04_ReadDistance_cm(void) {
    // 1. Kích xung Trig 10us
    GPIOA->BSRR = (1U << 1);
    uint16_t t0 = (uint16_t)TIM4->CNT;
    while ((uint16_t)(TIM4->CNT - t0) < 10) { }
    GPIOA->BSRR = (1U << (1 + 16));

    // 2. Chờ bắt đầu nhận sóng
    uint16_t wait_start = (uint16_t)TIM4->CNT;
    while (!(GPIOA->IDR & (1U << 4))) {
        if ((uint16_t)(TIM4->CNT - wait_start) > ECHO_TIMEOUT_US) return -1.0f;
    }

    // 3. Đếm thời gian sóng dội
    uint16_t echo_start = (uint16_t)TIM4->CNT;
    while (GPIOA->IDR & (1U << 4)) {
        if ((uint16_t)(TIM4->CNT - echo_start) > ECHO_TIMEOUT_US) return -1.0f;
    }
    uint16_t echo_end = (uint16_t)TIM4->CNT;

    // 4. Tính khoảng cách
    uint16_t echo_us = echo_end - echo_start;
    return (float)echo_us / 58.0f;
}

/* =========================================================================
 * 5. GỬI DỮ LIỆU QUA UART - còn thiếu ở bản trước, khiến STM32 không phản
 *    hồi được gì về PC dù nhận lệnh đúng (nguyên nhân "gửi nhiều mà nhận 0"
 *    bên communication.py).
 * ========================================================================= */
void UART_SendChar(char c) {
    while (!(USART1->SR & USART_SR_TXE));   /* chờ thanh ghi TX trống */
    USART1->DR = (uint16_t)c;
}

void UART_SendString(char *str) {
    while (*str) {
        UART_SendChar(*str++);
    }
}
