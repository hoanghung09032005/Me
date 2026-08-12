#include "hardware.h"

#define CLOCK_HSE_TIMEOUT_LOOPS 0x5000U
#define PWM_MAX                 999

#define HCSR04_TRIG_PIN         (1U << 1)
#define HCSR04_ECHO_PIN         (1U << 4)
#define ECHO_TIMEOUT_US         30000U

#define DHT22_DATA_PIN          (1U << 2)
#define DHT22_INVALID_X10       ((int16_t)-32768)

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
    /* PA2: floating input. The external 4.7k-10k pull-up keeps the line high. */
    GPIOA->CRL = (GPIOA->CRL & ~(0xFU << 8)) | (0x4U << 8);
}

static uint8_t DHT22_Read(int16_t *temperature_c_x10, int16_t *humidity_rh_x10)
{
    uint8_t data[5] = {0};

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
        return 0;
    }

    for (uint8_t bit = 0; bit < 40; bit++) {
        uint16_t high_start;

        if (!WaitForPinLevel(DHT22_DATA_PIN, 1, 80)) {
            return 0;
        }
        high_start = (uint16_t)TIM4->CNT;
        if (!WaitForPinLevel(DHT22_DATA_PIN, 0, 100)) {
            return 0;
        }

        data[bit / 8] <<= 1;
        if ((uint16_t)(TIM4->CNT - high_start) > 50U) {
            data[bit / 8] |= 1U;
        }
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
            /* Keep the reset-clock configuration. main() starts in MODE_IDLE. */
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

    /* PA9: USART1 TX alternate push-pull. PA10: RX input with pull-up. */
    GPIOA->CRH = (GPIOA->CRH & ~(0xFFU << 4)) | (0x8BU << 4);
    GPIOA->BSRR = (1U << 10);

    /* PCLK2 equals SystemCoreClock because APB2 is never divided here.
     * Calculate BRR at runtime so UART remains 115200 even if the HSE crystal
     * fails and SystemClock_Config() intentionally falls back to 8 MHz HSI. */
    USART1->CR1 = 0;
    USART1->BRR = (SystemCoreClock + 57600U) / 115200U;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    NVIC_ClearPendingIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART1_IRQn);
}

void GPIO_Config(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;

    /* Free PB4 for motor direction while retaining SWD debugging. */
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

    /* Motor PWM: PA8 = TIM1_CH1, PA11 = TIM1_CH4. */
    GPIOA->CRH = (GPIOA->CRH & ~((0xFU << 0) | (0xFU << 12))) |
                  ((0xBU << 0) | (0xBU << 12));

    /* Servo: PA0 = TIM2_CH1 alternate push-pull. */
    GPIOA->CRL = (GPIOA->CRL & ~(0xFU << 0)) | (0xBU << 0);

    /* HC-SR04: PA1 trigger output, PA4 echo input with pull-down. */
    GPIOA->CRL = (GPIOA->CRL & ~(0xFU << 4)) | (0x2U << 4);
    GPIOA->BRR = HCSR04_TRIG_PIN;
    GPIOA->CRL = (GPIOA->CRL & ~(0xFU << 16)) | (0x8U << 16);
    GPIOA->BRR = HCSR04_ECHO_PIN;

    /* DHT22: PA2 open-drain data line. Add a physical 4.7k-10k pull-up to 3.3 V. */
    DHT22_SetOutput();
    GPIOA->BSRR = DHT22_DATA_PIN;

    /* L298N directions: PB4, PB5, PB6, PB7. */
    GPIOB->CRL = (GPIOB->CRL & ~((0xFU << 16) | (0xFU << 20) |
                                 (0xFU << 24) | (0xFU << 28))) |
                  ((0x2U << 16) | (0x2U << 20) |
                   (0x2U << 24) | (0x2U << 28));

    /* Five-array sensors PB8..PB12 and side sensors PB13..PB14. */
    GPIOB->CRH = (GPIOB->CRH & ~0x0FFFFFFFU) | 0x08888888U;
}

void TIM_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;

    /* TIM1: 1 kHz PWM for L298N ENA/ENB. */
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

    /* TIM3: 10 ms control tick. APB1 is divided by 2, so timer clock is 72 MHz. */
    TIM3->CR1 = 0;
    TIM3->PSC = 7200 - 1;
    TIM3->ARR = 100 - 1;
    TIM3->DIER = TIM_DIER_UIE;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0;
    NVIC_ClearPendingIRQ(TIM3_IRQn);
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1 = TIM_CR1_CEN;

    /* TIM2: 50 Hz servo PWM, 1 us resolution. */
    TIM2->CR1 = 0;
    TIM2->PSC = 72 - 1;
    TIM2->ARR = 20000 - 1;
    TIM2->CCR1 = 1500;
    TIM2->CCMR1 = (6U << 4) | TIM_CCMR1_OC1PE;
    TIM2->CCER = TIM_CCER_CC1E;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    /* TIM4: free-running 1 MHz counter used for HC-SR04 and DHT22 timing. */
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
