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
#define DHT22_PERIOD_TICKS  200U   /* 2 giây ở nhịp tick TIM3 10 ms. */
#define IWDG_TIMEOUT_MS      500U /* phải giữ dư nhiều so với thời gian vòng lặp xấu nhất */

/* Số tick TIM3 (10ms/tick) tối đa cho phép KHÔNG nhận được bất kỳ lệnh nào
 * từ GUI (kể cả heartbeat 'H') trong lúc đang ở MODE_AUTO, trước khi coi
 * như đã MẤT KẾT NỐI và tự dừng hẳn toàn bộ hoạt động Mode 2 (bám line,
 * né vật cản, servo quét) - xem TIM3_IRQHandler bên dưới.
 *
 * LÝ DO CẦN CƠ CHẾ NÀY: Mode 2 được thiết kế chạy tự động, không cần lệnh
 * liên tục từ PC - chỉ nhận "A" một lần rồi tự chạy tới khi có "S". Nếu
 * kết nối WiFi rớt "âm thầm" (mất sóng, không phải người dùng chủ động
 * bấm ngắt), việc ESP32 phát hiện TCP đã chết (car_bridge.cpp) có thể
 * không tức thời -> lệnh "S" tự động có thể chậm hoặc không bao giờ tới
 * -> xe tiếp tục chạy Mode 2 vô thời hạn dù không còn ai giám sát.
 * GUI (mode2_auto.py) phải gửi lệnh 'H' định kỳ (~300ms) trong lúc Auto
 * đang chạy để "làm mới" đồng hồ này; nếu không nhận được gì trong
 * AUTO_LINK_TIMEOUT_TICKS, firmware TỰ dừng, không phụ thuộc ESP32 có
 * phát hiện mất TCP hay không - đây là lớp bảo vệ độc lập, chắc chắn hơn.
 * 150 tick = 1.5s, dư khoảng 5 lần bỏ lỡ heartbeat liên tiếp trước khi
 * trigger - đủ chịu được jitter/độ trễ thường gặp của WiFi. */
#define AUTO_LINK_TIMEOUT_TICKS  150U

volatile int car_mode = MODE_IDLE;
volatile int error = 0;
volatile int last_error = 0;
volatile int log_pwm_l = 0;
volatile int log_pwm_r = 0;
volatile int log_distance_cm_x10 = -1;
volatile int telemetry_ready = 0;
volatile uint8_t raw_state = 0;
/* 2 "mắt biên" rời (side_left/side_right) đọc ở GPIOB13/14. Gửi RIÊNG 2
 * trường về GUI (không gộp vào 1 byte bitmask) vì communication.py /
 * mode1_manual.py / mode2_auto.py phía Python đang đọc trực tiếp
 * data["side_left"] / data["side_right"] như 2 khoá tách biệt trong gói
 * LOG - gộp lại thành bitmask sẽ làm lệch toàn bộ các trường phía sau
 * (pwm, distance...) khi GUI parse, mà lỗi này rất dễ im lặng không báo gì. */
volatile uint8_t side_left_state = 0;
volatile uint8_t side_right_state = 0;
volatile uint32_t control_ticks = 0;
/* Tick TIM3 gần nhất mà STM32 nhận được MỘT LỆNH HỢP LỆ BẤT KỲ từ GUI
 * (không riêng gì 'H' - mọi lệnh 'S'/'M'/'A'/'V'/'F'/'B'/'L'/'R'/'H' đều
 * tính, vì tất cả đều chứng minh đường truyền còn sống). Được cập nhật
 * trong main(), ngay sau khi Take_Command() lấy được lệnh - xem bên dưới.
 * Dùng để phát hiện mất kết nối khi đang MODE_AUTO, xem AUTO_LINK_TIMEOUT_TICKS. */
volatile uint32_t last_command_tick = 0;

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
            /* Dừng khẩn cấp hoạt động từ cả IDLE, MANUAL và AUTO. */
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

        case 'H':
            /* Heartbeat/giữ-kết-nối từ GUI trong lúc MODE_AUTO đang chạy.
             * KHÔNG đổi car_mode, KHÔNG đụng tới trạng thái PID/né vật cản -
             * chỉ dùng để chứng minh đường truyền còn sống. Việc "làm mới"
             * last_command_tick đã xảy ra ở nơi gọi hàm này (xem main()),
             * áp dụng chung cho MỌI lệnh chứ không riêng 'H'. Trả ACK nhẹ
             * để GUI có thể xác nhận round-trip nếu cần, không bắt buộc
             * GUI phải đọc dòng này. */
            UART_SendString("ACK,H\n");
            break;

        default:
            /* Bỏ qua lệnh sai định dạng thay vì đổi trạng thái động cơ. */
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
    side_left_state = side_left;
    side_right_state = side_right;

    if (car_mode == MODE_MANUAL) {
        Mode1_Update();
    } else if (car_mode == MODE_AUTO) {
        if ((uint32_t)(control_ticks - last_command_tick) > AUTO_LINK_TIMEOUT_TICKS) {
            /* Mất kết nối với GUI (không nhận được lệnh/heartbeat nào
             * trong AUTO_LINK_TIMEOUT_TICKS) - DỪNG HẲN mọi hoạt động
             * Mode 2 (bám line, né vật cản, servo quét, các yêu cầu đo
             * siêu âm mới) thay vì tiếp tục chạy tự động vô thời hạn mà
             * không còn ai giám sát.
             * KHÔNG gọi Stop_All() ở đây: hàm đó dùng UART_SendString()
             * (vòng lặp bận chờ TXE) - không an toàn để gọi trong ISR ưu
             * tiên cao nhất hệ thống, có thể làm trễ nhịp điều khiển 10ms.
             * Mode2_Obstacle_Init() chỉ ghi thanh ghi/biến trạng thái,
             * không block, nên gọi thẳng trong ISR là an toàn. */
            car_mode = MODE_IDLE;
            Mode2_Obstacle_Init();
        } else {
            Mode2_Obstacle_Update(raw_state, side_left, side_right);
        }
    } else {
        Set_Motor_Outputs(0, 0);
    }
}

/* Báo nguyên nhân của lần reset gần nhất lên PC qua UART, phát ngay sau khi
 * boot. Mục đích: giúp phân biệt "xe bị treo do phần mềm" với "xe bị RESET
 * do sụt áp nguồn" - vấn đề rất hay gặp khi motor kéo dòng lớn lúc khởi
 * động/đổi hướng đột ngột mà chưa tách nguồn động cơ khỏi nguồn logic.
 *
 * STM32F1 không có cờ "brown-out" riêng như một số dòng F4 - cờ gần nhất
 * với ý nghĩa đó là RCC_CSR_PORRSTF (Power-On/Power-Down Reset): cờ này
 * bật khi VDD từng tụt xuống dưới ngưỡng POR/PDR rồi hồi phục.
 *
 * CÁCH DÙNG: mở log GUI, nếu thấy dòng "STM32,RESET_CAUSE,POWER_DIP" xuất
 * hiện GIỮA LÚC ĐANG CHẠY (không phải lúc bạn tự cấp nguồn) -> gần như chắc
 * chắn nguồn đang bị sụt áp khi motor hoạt động, cần tách nguồn động cơ /
 * thêm tụ lọc như khuyến nghị. Nếu thấy "STM32,RESET_CAUSE,IWDG" thì đó là
 * lần watchdog (xem IWDG_Init trong hardware.c) đã cứu firmware khỏi bị
 * treo. */
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

    RCC->CSR |= RCC_CSR_RMVF;   /* xoá cờ để lần reset kế tiếp đọc đúng */

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
    TIM_Init();
    USART1_Init();

    Report_Reset_Cause();

    Mode1_Init();
    Mode2_Obstacle_Init();
    DHT22_Service();

    {
        char boot_message[32];
        snprintf(boot_message, sizeof(boot_message),
                 "STM32,BOOT,%luHz\n", (unsigned long)SystemCoreClock);
        UART_SendString(boot_message);
    }

    /* Khởi động watchdog SAU CÙNG, ngay trước vòng lặp chính, để mọi công
     * việc init phía trên (có giới hạn thời gian nhưng khác 0) không thể
     * làm nó timeout. Nếu CPU treo trong vòng lặp bên dưới - ví dụ sau một
     * lần brown-out biên (marginal) làm hỏng luồng thực thi mà không kích
     * hoạt reset power-on sạch - watchdog đảm bảo chip được reset về trạng
     * thái an toàn đã biết (car_mode = MODE_IDLE, động cơ tắt) trong vòng
     * IWDG_TIMEOUT_MS thay vì xe cứ đứng yên ở bất kỳ trạng thái nào nó
     * đang bị kẹt. */
    IWDG_Init(IWDG_TIMEOUT_MS);

    while (1) {
        IWDG_Refresh();

        if (Take_Command(command)) {
            /* Bất kỳ lệnh hợp lệ nào tới đây - kể cả 'H' heartbeat - đều
             * chứng minh đường truyền GUI<->STM32 còn sống. Cập nhật TRƯỚC
             * khi Process_Command() để không phụ thuộc lệnh cụ thể là gì. */
            last_command_tick = control_ticks;
            Process_Command(command);
        }

        /* Các driver polling này không bao giờ chạy trong ngắt PID. */
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
            /* Định dạng LOG:
             * LOG,line_mask,side_left,side_right,error_x100,pwm_l,pwm_r,
             *     distance_x10,temp_x10,humidity_x10 */
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
