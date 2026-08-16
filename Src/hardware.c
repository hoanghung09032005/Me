#include "hardware.h"

#define CLOCK_HSE_TIMEOUT_LOOPS 0x5000U
#define PWM_MAX                 999

#define HCSR04_TRIG_PIN         (1U << 1)
#define HCSR04_ECHO_PIN         (1U << 4)
#define ECHO_TIMEOUT_US         30000U

#define DHT22_DATA_PIN          (1U << 2)
#define DHT22_INVALID_X10       ((int16_t)-32768)

/* IWDG key register values, theo STM32F1 reference manual. */
#define IWDG_KEY_REFRESH         0xAAAAU
#define IWDG_KEY_WRITE_ENABLE    0x5555U
#define IWDG_KEY_START           0xCCCCU
#define IWDG_LSI_FREQ_HZ         40000U /* nominal; LSI thực tế có thể dao động ~30-60 kHz */
#define IWDG_PRESCALER_DIV       64U    /* PR = 4 -> divider 64 */

/* Số vòng lặp tối đa khi chờ LSI khởi động / IWDG đồng bộ thanh ghi, trước
 * khi CHỦ ĐỘNG BỎ QUA thay vì treo cứng vô thời hạn. Đây KHÔNG phải giá trị
 * thời gian chính xác (phụ thuộc tốc độ CPU), chỉ là một cận trên đủ lớn
 * (hàng chục nghìn vòng, thực tế LSI/IWDG sync chỉ mất vài trăm us nếu chạy
 * đúng) để đảm bảo firmware KHÔNG BAO GIỜ đứng im vĩnh viễn ở bước init chỉ
 * vì watchdog - lỗi này đã từng khiến main() không bao giờ chạy tới while(1)
 * dù không có gì báo lỗi ra UART (vì lệnh gửi UART nằm SAU đoạn bị treo). */
#define IWDG_WAIT_LOOP_LIMIT      100000U

static volatile uint8_t hcsr04_measurement_requested = 0;
static volatile int16_t hcsr04_distance_cm_x10 = -1;

static volatile int16_t dht22_temperature_c_x10 = DHT22_INVALID_X10;
static volatile int16_t dht22_humidity_rh_x10 = DHT22_INVALID_X10;

static void Delay_us(uint16_t delay_us)
{
    uint16_t start = (uint16_t)TIM4->CNT;
    while ((uint16_t)(TIM4->CNT - start) < delay_us) { }
}

static uint8_t WaitForPinLevel(uint32_t pin, uint8_t level, uint16_t timeout_us)
{
    uint16_t start = (uint16_t)TIM4->CNT;

    while (((GPIOA->IDR & pin) ? 1U : 0U) != level) {
        if ((uint16_t)(TIM4->CNT - start) > timeout_us) {
            return 0;
        }
    }
    return 1;
}

static void DHT22_SetOutput(void)
{
    /* PA2: general-purpose open-drain output, 2 MHz. */
    GPIOA->CRL = (GPIOA->CRL & ~(0xFU << 8)) | (0x6U << 8);
}

static void DHT22_SetInput(void)
{
    /* PA2: floating input. Điện trở kéo lên ngoài 4.7k-10k giữ đường dây ở mức cao. */
    GPIOA->CRL = (GPIOA->CRL & ~(0xFU << 8)) | (0x4U << 8);
}

static uint8_t DHT22_Read(int16_t *temperature_c_x10, int16_t *humidity_rh_x10)
{
    uint8_t data[5] = {0};
    uint8_t ok = 1;

    /* DHT22 là giao thức bit-bang: mỗi bit được phân biệt 0/1 bằng ĐỘ RỘNG
     * xung mức cao (~26-28us cho bit 0, ~70us cho bit 1), đo bằng cách so
     * sánh giá trị bộ đếm phần cứng TIM4 tại hai thời điểm.
     *
     * Nếu ngắt USART1 (byte lệnh từ PC tới) hoặc ngắt TIM3 (chu kỳ điều
     * khiển động cơ 10ms) xảy ra đúng lúc đang chờ đổi mức chân GPIO, CPU
     * bị "giữ chân" vài micro-giây trong ISR trước khi kịp đọc lại chân ->
     * phép đo bị cộng thêm thời gian trễ đó -> đôi khi một bit 0 (~27us)
     * bị "kéo dài" qua ngưỡng 50us và bị hiểu nhầm thành bit 1 -> sai
     * checksum -> đọc thất bại.
     *
     * Tắt ngắt toàn cục trong đúng khoảng bit-bang (tổng cộng ~5ms, lặp lại
     * mỗi 2 giây - xem DHT22_PERIOD_TICKS trong main.c) để đảm bảo phép đo
     * không bị nhiễu. Không mất tick điều khiển: bộ đếm TIM3 vẫn chạy bằng
     * phần cứng, ngắt đang "chờ" (pending) sẽ được xử lý ngay khi bật lại
     * ngắt, chỉ trễ tối đa một chu kỳ 10ms một lần mỗi 2 giây - không ảnh
     * hưởng gì tới việc lái xe hay bám line. */
    __disable_irq();

    DHT22_SetOutput();
    GPIOA->BRR = DHT22_DATA_PIN;
    Delay_us(1200);
    GPIOA->BSRR = DHT22_DATA_PIN;
    Delay_us(30);
    DHT22_SetInput();

    /* Sensor acknowledgement: 80 us low, 80 us high, then first bit low. */
    if (!WaitForPinLevel(DHT22_DATA_PIN, 0, 120) ||
        !WaitForPinLevel(DHT22_DATA_PIN, 1, 120) ||
        !WaitForPinLevel(DHT22_DATA_PIN, 0, 120)) {
        ok = 0;
    }

    for (uint8_t bit = 0; ok && bit < 40; bit++) {
        uint16_t high_start;

        if (!WaitForPinLevel(DHT22_DATA_PIN, 1, 80)) {
            ok = 0;
            break;
        }
        high_start = (uint16_t)TIM4->CNT;
        if (!WaitForPinLevel(DHT22_DATA_PIN, 0, 100)) {
            ok = 0;
            break;
        }

        data[bit / 8] <<= 1;
        if ((uint16_t)(TIM4->CNT - high_start) > 50U) {
            data[bit / 8] |= 1U;
        }
    }

    __enable_irq();

    if (!ok) {
        return 0;
    }

    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) {
        return 0;
    }

    uint16_t humidity = ((uint16_t)data[0] << 8) | data[1];
    int16_t temperature = (int16_t)(((uint16_t)(data[2] & 0x7FU) << 8) | data[3]);
    if (data[2] & 0x80U) {
        temperature = -temperature;
    }

    *humidity_rh_x10 = (int16_t)humidity;
    *temperature_c_x10 = temperature;
    return 1;
}

void SystemClock_Config(void)
{
    uint32_t timeout = 0;

    /* Blue Pill: external 8 MHz crystal -> PLL x9 = 72 MHz. */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {
        if (++timeout > CLOCK_HSE_TIMEOUT_LOOPS) {
            /* Giữ cấu hình clock mặc định lúc reset. main() bắt đầu ở MODE_IDLE. */
            SystemCoreClockUpdate();
            return;
        }
    }

    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 |
                               RCC_CFGR_PPRE2 | RCC_CFGR_ADCPRE |
                               RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE |
                               RCC_CFGR_PLLMULL)) |
                RCC_CFGR_HPRE_DIV1 |
                RCC_CFGR_PPRE1_DIV2 |
                RCC_CFGR_PPRE2_DIV1 |
                RCC_CFGR_ADCPRE_DIV6 |
                RCC_CFGR_PLLSRC |
                RCC_CFGR_PLLMULL9;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }

    SystemCoreClockUpdate();
}

void USART1_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;

    /* PA9: USART1 TX alternate push-pull. PA10: RX input với pull-up. */
    GPIOA->CRH = (GPIOA->CRH & ~(0xFFU << 4)) | (0x8BU << 4);
    GPIOA->BSRR = (1U << 10);

    /* PCLK2 = SystemCoreClock vì APB2 không chia ở đây. Tính BRR lúc chạy
     * để UART luôn giữ 115200 kể cả khi thạch anh HSE lỗi và
     * SystemClock_Config() rơi về HSI 8 MHz. */
    USART1->CR1 = 0;
    USART1->BRR = (SystemCoreClock + 57600U) / 115200U;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    NVIC_ClearPendingIRQ(USART1_IRQn);
    /* TIM3 (điều khiển động cơ thời gian thực) phải có độ ưu tiên CAO HƠN
     * (số nhỏ hơn) USART1, để một chuỗi byte UART đến dồn dập không thể
     * trì hoãn vòng điều khiển PID/motor. Xem NVIC_SetPriority(TIM3_IRQn, 0)
     * trong TIM_Init() bên dưới. */
    NVIC_SetPriority(USART1_IRQn, 1);
    NVIC_EnableIRQ(USART1_IRQn);
}

void GPIO_Config(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;

    /* Giải phóng PB4 cho hướng động cơ nhưng vẫn giữ khả năng debug SWD. */
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

    /* Motor PWM: PA8 = TIM1_CH1, PA11 = TIM1_CH4. */
    GPIOA->CRH = (GPIOA->CRH & ~((0xFU << 0) | (0xFU << 12))) |
                  ((0xBU << 0) | (0xBU << 12));

    /* Servo: PA0 = TIM2_CH1 alternate push-pull. */
    GPIOA->CRL = (GPIOA->CRL & ~(0xFU << 0)) | (0xBU << 0);

    /* HC-SR04: PA1 trigger output, PA4 echo input với pull-down. */
    GPIOA->CRL = (GPIOA->CRL & ~(0xFU << 4)) | (0x2U << 4);
    GPIOA->BRR = HCSR04_TRIG_PIN;
    GPIOA->CRL = (GPIOA->CRL & ~(0xFU << 16)) | (0x8U << 16);
    GPIOA->BRR = HCSR04_ECHO_PIN;

    /* DHT22: PA2 open-drain data line. Cần điện trở kéo lên vật lý 4.7k-10k
     * lên 3.3 V. */
    DHT22_SetOutput();
    GPIOA->BSRR = DHT22_DATA_PIN;

    /* L298N directions: PB4, PB5, PB6, PB7. */
    GPIOB->CRL = (GPIOB->CRL & ~((0xFU << 16) | (0xFU << 20) |
                                 (0xFU << 24) | (0xFU << 28))) |
                  ((0x2U << 16) | (0x2U << 20) |
                   (0x2U << 24) | (0x2U << 28));

    /* 5 mắt dò line PB8..PB12, 2 cảm biến hông PB13..PB14. */
    GPIOB->CRH = (GPIOB->CRH & ~0x0FFFFFFFU) | 0x08888888U;
}

void TIM_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;

    /* TIM1: 1 kHz PWM cho L298N ENA/ENB. */
    TIM1->CR1 = 0;
    TIM1->PSC = 72 - 1;
    TIM1->ARR = PWM_MAX;
    TIM1->CCR1 = 0;
    TIM1->CCR4 = 0;
    TIM1->CCMR1 = (6U << 4) | TIM_CCMR1_OC1PE;
    TIM1->CCMR2 = (6U << 12) | TIM_CCMR2_OC4PE;
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC4E;
    TIM1->BDTR = TIM_BDTR_MOE;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    /* TIM3: chu kỳ điều khiển 10 ms. APB1 chia 2 nên clock timer là 72 MHz. */
    TIM3->CR1 = 0;
    TIM3->PSC = 7200 - 1;
    TIM3->ARR = 100 - 1;
    TIM3->DIER = TIM_DIER_UIE;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0;
    NVIC_ClearPendingIRQ(TIM3_IRQn);
    /* Ưu tiên CAO HƠN USART1 (số nhỏ hơn = ưu tiên cao hơn trên Cortex-M3) -
     * đảm bảo vòng điều khiển động cơ luôn đúng nhịp 10ms bất kể UART có
     * đang bận xử lý bao nhiêu byte lệnh từ PC. */
    NVIC_SetPriority(TIM3_IRQn, 0);
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1 = TIM_CR1_CEN;

    /* TIM2: PWM servo 50 Hz, độ phân giải 1 us. */
    TIM2->CR1 = 0;
    TIM2->PSC = 72 - 1;
    TIM2->ARR = 20000 - 1;
    TIM2->CCR1 = 1500;
    TIM2->CCMR1 = (6U << 4) | TIM_CCMR1_OC1PE;
    TIM2->CCER = TIM_CCER_CC1E;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    /* TIM4: bộ đếm free-running 1 MHz dùng cho timing HC-SR04 và DHT22. */
    TIM4->CR1 = 0;
    TIM4->PSC = 72 - 1;
    TIM4->ARR = 0xFFFF;
    TIM4->EGR = TIM_EGR_UG;
    TIM4->CR1 = TIM_CR1_CEN;
}

void Set_Motor_Outputs(int pwm_l, int pwm_r)
{
    if (pwm_l > PWM_MAX) pwm_l = PWM_MAX;
    if (pwm_l < -PWM_MAX) pwm_l = -PWM_MAX;
    if (pwm_r > PWM_MAX) pwm_r = PWM_MAX;
    if (pwm_r < -PWM_MAX) pwm_r = -PWM_MAX;

    if (pwm_l >= 0) {
        GPIOB->BSRR = (1U << 5) | (1U << (6 + 16));
        TIM1->CCR4 = (uint16_t)pwm_l;
    } else {
        GPIOB->BSRR = (1U << (5 + 16)) | (1U << 6);
        TIM1->CCR4 = (uint16_t)(-pwm_l);
    }

    if (pwm_r >= 0) {
        GPIOB->BSRR = (1U << 7) | (1U << (4 + 16));
        TIM1->CCR1 = (uint16_t)pwm_r;
    } else {
        GPIOB->BSRR = (1U << (7 + 16)) | (1U << 4);
        TIM1->CCR1 = (uint16_t)(-pwm_r);
    }
}

void Servo_SetAngle(uint16_t deg)
{
    if (deg > 180U) deg = 180U;
    TIM2->CCR1 = 1000U + ((uint32_t)deg * 1000U) / 180U;
}

float HCSR04_ReadDistance_cm(void)
{
    GPIOA->BSRR = HCSR04_TRIG_PIN;
    Delay_us(10);
    GPIOA->BRR = HCSR04_TRIG_PIN;

    if (!WaitForPinLevel(HCSR04_ECHO_PIN, 1, ECHO_TIMEOUT_US)) {
        return -1.0f;
    }

    uint16_t echo_start = (uint16_t)TIM4->CNT;
    if (!WaitForPinLevel(HCSR04_ECHO_PIN, 0, ECHO_TIMEOUT_US)) {
        return -1.0f;
    }

    uint16_t echo_us = (uint16_t)(TIM4->CNT - echo_start);
    return (float)echo_us / 58.0f;
}

void HCSR04_RequestMeasurement(void)
{
    hcsr04_measurement_requested = 1;
}

void HCSR04_Service(void)
{
    if (!hcsr04_measurement_requested) {
        return;
    }

    hcsr04_measurement_requested = 0;
    float distance_cm = HCSR04_ReadDistance_cm();
    if (distance_cm > 0.0f && distance_cm < 3276.0f) {
        hcsr04_distance_cm_x10 = (int16_t)(distance_cm * 10.0f + 0.5f);
    } else {
        hcsr04_distance_cm_x10 = -1;
    }
}

int16_t HCSR04_GetDistance_cm_x10(void)
{
    return hcsr04_distance_cm_x10;
}

void DHT22_Service(void)
{
    int16_t temperature;
    int16_t humidity;

    if (DHT22_Read(&temperature, &humidity)) {
        dht22_temperature_c_x10 = temperature;
        dht22_humidity_rh_x10 = humidity;
    } else {
        dht22_temperature_c_x10 = DHT22_INVALID_X10;
        dht22_humidity_rh_x10 = DHT22_INVALID_X10;
    }
}

int16_t DHT22_GetTemperature_c_x10(void)
{
    return dht22_temperature_c_x10;
}

int16_t DHT22_GetHumidity_rh_x10(void)
{
    return dht22_humidity_rh_x10;
}

void UART_SendChar(char c)
{
    while (!(USART1->SR & USART_SR_TXE)) { }
    USART1->DR = (uint16_t)(c & 0xFF);
}

void UART_SendString(char *str)
{
    while (*str) {
        UART_SendChar(*str++);
    }
}

/* ======================================================================
 * IWDG - independent watchdog
 * ----------------------------------------------------------------------
 * Truy cập thanh ghi trực tiếp, cùng phong cách với phần còn lại của file
 * này (không dùng HAL_IWDG_*). Xem hardware.h để biết lý do.
 *
 * GHI CHÚ khi debug bằng ST-Link + breakpoint: mặc định IWDG vẫn đếm ngay
 * cả khi core đang dừng tại breakpoint, nên có thể reset chip giữa lúc
 * debug. main() đóng băng nó trong lúc debug halt qua DBGMCU->CR (xem
 * IWDG_Init bên dưới) để việc này không cản trở phát triển; watchdog vẫn
 * hoạt động đầy đủ trong chế độ vận hành bình thường.
 *
 * SỬA LỖI QUAN TRỌNG (đã xác nhận bằng debug thực tế qua Live Expressions):
 * Bản gốc dùng "while (IWDG->SR != 0U) { }" KHÔNG có timeout để chờ phần
 * cứng IWDG xác nhận đồng bộ PR/RLR vào miền clock LSI nội bộ. Trên board
 * thực tế đang dùng, LSI không bao giờ đồng bộ xong (nghi do lỗi hoặc khác
 * biệt hành vi trên chip cụ thể của board), khiến CPU TREO VĨNH VIỄN ngay
 * trong IWDG_Init(), main() KHÔNG BAO GIỜ chạy tới while(1) chính. Điều
 * này rất khó phát hiện qua log UART vì mọi lệnh UART_SendString() nằm
 * SAU đoạn bị treo trong main() đều không bao giờ chạy, còn ngắt (ISR)
 * vẫn hoạt động bình thường (USART1_IRQHandler, TIM3_IRQHandler) nên nhìn
 * qua tưởng hệ thống "còn sống" một phần.
 * Đã thêm timeout IWDG_WAIT_LOOP_LIMIT ở CẢ hai vòng chờ (LSI ready và
 * IWDG->SR) để đảm bảo firmware LUÔN LUÔN thoát ra và chạy tiếp, kể cả khi
 * watchdog thực sự không khởi động được trên phần cứng này. */
void IWDG_Init(uint16_t timeout_ms)
{
    uint32_t wait_count;

    /* Đóng băng IWDG khi core bị debugger dừng lại
     * (DBGMCU_CR bit 8 = DBG_IWDG_STOP), để breakpoint không gây reset
     * giả trong lúc phát triển. */
    DBGMCU->CR |= (1UL << 8);

    /* Bật LSI tường minh và chờ sẵn sàng TRƯỚC khi đụng tới IWDG - có
     * timeout để không treo cứng nếu LSI lỗi trên phần cứng này. */
    RCC->CSR |= RCC_CSR_LSION;
    wait_count = 0;
    while (!(RCC->CSR & RCC_CSR_LSIRDY)) {
        if (++wait_count > IWDG_WAIT_LOOP_LIMIT) {
            break;   /* LSI không lên được - vẫn tiếp tục, đừng treo main() */
        }
    }

    IWDG->KR = IWDG_KEY_WRITE_ENABLE;

    /* Prescaler /64 -> IWDG tick = 40000/64 Hz = 625 Hz = 1.6 ms/tick. */
    IWDG->PR = 4U;

    uint32_t reload = ((uint32_t)timeout_ms * (IWDG_LSI_FREQ_HZ / IWDG_PRESCALER_DIV)) / 1000U;
    if (reload > 0xFFFU) {
        reload = 0xFFFU;
    } else if (reload == 0U) {
        reload = 1U;
    }
    IWDG->RLR = (uint16_t)reload;

    wait_count = 0;
    while (IWDG->SR != 0U) {
        /* chờ PR/RLR được xác nhận cập nhật, có timeout - xem ghi chú ở
         * đầu hàm này để biết lý do timeout bắt buộc phải có. */
        if (++wait_count > IWDG_WAIT_LOOP_LIMIT) {
            break;
        }
    }

    IWDG->KR = IWDG_KEY_REFRESH; /* nạp lại bộ đếm trước khi start */
    IWDG->KR = IWDG_KEY_START;   /* khởi động watchdog - không thể dừng trừ khi reset */
}

void IWDG_Refresh(void)
{
    IWDG->KR = IWDG_KEY_REFRESH;
}
