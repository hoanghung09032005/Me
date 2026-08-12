#include "stm32f4xx.h"
#include "hardware.h"

// Khởi tạo biến toàn cục
volatile char rx_buffer[32];   // Bộ đệm chứa chữ gửi từ Máy tính xuống
volatile int rx_index = 0;     // Con trỏ vị trí chữ
volatile int cmd_ready = 0;    // Cờ báo hiệu: 1 = Đã nhận xong 1 lệnh


#define ENABLE_USB_DEBUG_UART   1

// Cấu hình pinout
void System_Init(void) {

    // Cấp xung cho khối ngoại vi
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_USART1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
#if ENABLE_USB_DEBUG_UART
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
#endif

    // Cảm biến (PC0-PC4), led (PC13)
    GPIOC->MODER &= ~(0x3FF | (3U << 26));
    GPIOC->MODER |= (1U << 26);
    GPIOC->PUPDR |= (1U << 0) | (1U << 2) | (1U << 4) | (1U << 6) | (1U << 8);

    // MOTOR
    // Động cơ (Băm xung PWM): Chân PA11 (Trái) và PA8 (Phải)
    GPIOA->MODER &= ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER11_Msk);
    GPIOA->MODER |= (2U << 16) | (2U << 22);
    GPIOA->AFR[1] |= (1U << 0) | (1U << 12); // AF1 = TIM1 cho PA8 và PA11

    // Mạch cầu H (Đảo chiều quay): Chân PB4, PB5, PB6, PB7 làm Output
    GPIOB->MODER &= ~(GPIO_MODER_MODER4_Msk | GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER7_Msk);
    GPIOB->MODER |= (1U << 8) | (1U << 10) | (1U << 12) | (1U << 14);

    // KHỐI BỘ ĐẾM THỜI GIAN (TIMER)
    // TIM1 (Xuất PWM) tần số băm xung 1000Hz, dải tốc độ 0 -> 999
    TIM1->PSC = 16 - 1; TIM1->ARR = 1000 - 1;
    TIM1->CCMR1 |= (6U << 4) | TIM_CCMR1_OC1PE; // Bật Kênh 1
    TIM1->CCMR2 |= (6U << 12) | TIM_CCMR2_OC4PE; // Bật Kênh 4
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC4E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_CEN; // Khởi động TIM1

    // TIM3 (Nhịp PID) đặt ngắt 10ms
    TIM3->PSC = 1600 - 1; TIM3->ARR = 100 - 1;
    TIM3->DIER |= TIM_DIER_UIE;
    TIM3->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM3_IRQn);

    // USART1 trên PA9/PA10: PA9  = TX, PA10 = RX
    GPIOA->MODER &= ~(GPIO_MODER_MODER9_Msk | GPIO_MODER_MODER10_Msk);
    GPIOA->MODER |= (2U << 18) | (2U << 20);   // chế độ Alternate Function

    // AF7 = USART1. Trong AFR[1], PA9 chiếm 4 bit thứ 2 (dịch 4),
    // PA10 chiếm 4 bit thứ 3 (dịch 8). Không đụng PA8/PA11 của TIM1 ở trên.
    GPIOA->AFR[1] |= (7U << 4) | (7U << 8);

    // Kéo trở lên cho chân RX
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD10_Msk;
    GPIOA->PUPDR |= (1U << 20);

    USART1->BRR = 16000000 / 115200;
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    NVIC_EnableIRQ(USART1_IRQn);

#if ENABLE_USB_DEBUG_UART
    // USART2 trên PA2/PA3 - ĐƯỜNG PHỤ QUA CÁP USB (ST-LINK)
    // Dùng cho test_stm32_usb.py.
    GPIOA->MODER &= ~(GPIO_MODER_MODER2_Msk | GPIO_MODER_MODER3_Msk);
    GPIOA->MODER |= (2U << 4) | (2U << 6);
    GPIOA->AFR[0] |= (7U << 8) | (7U << 12);   // AF7 = USART2 cho PA2, PA3

    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD3_Msk;
    GPIOA->PUPDR |= (1U << 6);

    USART2->BRR = 16000000 / 115200;           // APB1 cũng 16MHz
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    NVIC_EnableIRQ(USART2_IRQn);
#endif
}

// Điều khiển động cơ
void Set_Motor_Outputs(int s_l, int s_r) {
    // Nếu tốc độ âm -> Đảo chiều tín hiệu Mạch cầu H để xe lùi bánh
    if (s_l >= 0) { GPIOB->BSRR = GPIO_BSRR_BS5 | GPIO_BSRR_BR6; TIM1->CCR4 = s_l; }
    else          { GPIOB->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BS6; TIM1->CCR4 = -s_l; }

    if (s_r >= 0) { GPIOB->BSRR = GPIO_BSRR_BS7 | GPIO_BSRR_BR4; TIM1->CCR1 = s_r; }
    else          { GPIOB->BSRR = GPIO_BSRR_BR7 | GPIO_BSRR_BS4; TIM1->CCR1 = -s_r; }
}

// Xuất dữ liệu qua cả 2 cổng: esp và uart2
void UART_SendChar(char c) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = c;
#if ENABLE_USB_DEBUG_UART
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = c;
#endif
}

void UART_SendString(char* str) { while (*str) UART_SendChar(*str++); }

// Nhận dữ liệu
static void UART_HandleRxByte(char c) {
    // Nếu thấy phím Enter (\n), chốt lệnh và bật cờ cmd_ready
    if (c == '\n' || c == '\r') {
        rx_buffer[rx_index] = '\0';
        if (rx_index > 0) cmd_ready = 1;
        rx_index = 0;
    }
    else if (rx_index < 31) rx_buffer[rx_index++] = c; // Lưu dần các chữ cái
}

void USART1_IRQHandler(void) {
    if (USART1->SR & USART_SR_RXNE) UART_HandleRxByte((char)USART1->DR);
}

#if ENABLE_USB_DEBUG_UART
void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) UART_HandleRxByte((char)USART2->DR);
}
#endif
