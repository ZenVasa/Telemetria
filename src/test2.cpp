
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <stdbool.h>

// Константы для SPI
constexpr uint32_t SPI_DEVICE = SPI2;
constexpr uint32_t SPI_PORT = GPIOB;
constexpr uint16_t SPI_SCK_PIN = GPIO13;
constexpr uint16_t SPI_MISO_PIN = GPIO14;
constexpr uint16_t SPI_MOSI_PIN = GPIO15;
constexpr uint16_t SPI_NSS_PIN = GPIO12;

// Таймер
constexpr uint32_t MEASURE_TIMER = TIM2;

// Пины и периферия
constexpr uint32_t TRIG_PORT = GPIOA;
constexpr uint16_t TRIG_PIN = GPIO5;        // PA5 - TIM2_CH1 - TRIG
constexpr uint32_t ECHO_PORT = GPIOA;  
constexpr uint16_t ECHO_PIN = GPIO3;        // PA3 - TIM2_CH4 - ECHO

// LED
constexpr uint32_t LED_PORT = GPIOA; // GPIOA - чёрная, GPIOD - зелёная
constexpr uint16_t LED_PIN = GPIO1;  // GPIO1           GPIO14

// UART
constexpr uint32_t UART_PORT = GPIOA;                           // GPIOA - чёрная   GPIOD - зелёная  
constexpr uint16_t UART_TX_PIN = GPIO9;     // TX <- RX(надо)   // GPIO9            GPIO8       
constexpr uint16_t UART_RX_PIN = GPIO10;    // RX <- TX         // GPIO10           GPIO9 
constexpr uint32_t UART_DEVICE = USART1;                        // USART1           USART3

// Глобальные переменные
volatile uint32_t pulse_start = 0;
volatile uint32_t pulse_end = 0;
volatile uint32_t pulse_width = 0;
volatile bool measurement_ready = false;
volatile uint32_t measurement_count = 0;

// Прототипы функций
void uart_setup(void);
void gpio_setup(void);
void timer_setup(void);
void tim2_isr(void);
void uart_send_string(const char *str);
void send_distance_cm(uint32_t distance_cm);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void led_setup(void);
void clock_setup(void);
uint32_t measure_distance(void);
void my_usart_print_int(uint32_t usart, int32_t value);
// SPI
void spi2_slave_setup(void);
void spi2_slave_send(uint8_t data);
bool spi2_slave_selected(void);

void spi2_send_distance(uint32_t distance);  


void clock_setup(void){
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_84MHZ]);

    // Включаем тактирование для GPIOA, GPIOB, GPIOD, TIM2 и USART
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_clock_enable(RCC_SPI2);
    rcc_periph_clock_enable(RCC_USART1); // или RCC_USART3

}

// Основная функция
int main(void) {
    clock_setup(); 
    
    gpio_setup();
    spi2_slave_setup();
    timer_setup();
    uart_setup();
    
    uart_send_string(" Запуск HC-SR04 \r\n");
    
    while (true) {
        uint32_t distance = measure_distance(); // Замер и расчёт расстояния
        send_distance_cm(distance);             // Отправка расстояние в UART3 и моргание светодиодом

        if (spi2_slave_selected()) {
            spi2_send_distance(distance); // Отправляем расстояние по SPI
        }

        delay_ms(30);
    }
    
    return 0;
}

// Новая функция для отправки расстояния по SPI
void spi2_send_distance(uint32_t distance) {
    char buffer[10];
    int length = 0;
    
    // Начинаем с символа '$'
    spi2_slave_send('$');
    
    // Преобразуем число в строку
    if (distance == 0) {
        spi2_slave_send('0');
    } else {
        // Извлекаем цифры и сохраняем в буфер в обратном порядке
        while (distance > 0) {
            buffer[length++] = '0' + (distance % 10);
            distance /= 10;
        }

        // Отправляем цифры в правильном порядке (от старшей к младшей)
        for (int i = length - 1; i >= 0; i--) {
            spi2_slave_send(buffer[i]);
        }
    }
    
    // Завершаем символом ';'
    spi2_slave_send(';');
}


void uart_setup(void) {
    // Настраиваем пины UART
    gpio_mode_setup(UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, UART_TX_PIN | UART_RX_PIN);
    gpio_set_af(UART_PORT, GPIO_AF7, UART_TX_PIN | UART_RX_PIN);
    
    // Настраиваем UART
    usart_set_baudrate(UART_DEVICE, 115200);
    usart_set_databits(UART_DEVICE, 8);
    usart_set_stopbits(UART_DEVICE, USART_STOPBITS_1);
    usart_set_parity(UART_DEVICE, USART_PARITY_NONE);
    usart_set_flow_control(UART_DEVICE, USART_FLOWCONTROL_NONE);
    usart_set_mode(UART_DEVICE, USART_MODE_TX_RX);

    // Включаем UART
    usart_enable(UART_DEVICE);
}

void gpio_setup(void) { 
    // TRIG как выход
    gpio_mode_setup(TRIG_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TRIG_PIN);
    gpio_clear(TRIG_PORT, TRIG_PIN);
    
    // ECHO как альтернативная функция для TIM2
    gpio_mode_setup(ECHO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, ECHO_PIN);
    gpio_set_af(ECHO_PORT, GPIO_AF1, ECHO_PIN);

    // Настраиваем пин как выход        
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN); 
    // Гарантируем выключенное состояние 
    gpio_clear(LED_PORT, LED_PIN); 


    // SPI2 на выводах PB13(SCK), PB14(MISO), PB15(MOSI), PB12(NSS)
    gpio_mode_setup(SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI_SCK_PIN);   // PB13 SCK (вход)
    gpio_mode_setup(SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI_MOSI_PIN);  // PB15 MOSI (вход)
    gpio_mode_setup(SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI_MISO_PIN);  // PB14 MISO (выход)
    gpio_mode_setup(SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, SPI_NSS_PIN); // PB12 NSS (вход с подтяжкой к VCC)
    
    // Установка альтернативной функции SPI2 (AF5)
    gpio_set_af(SPI_PORT, GPIO_AF5, SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN | SPI_NSS_PIN);

}

// Настройка SPI2 в режиме ведомого
void spi2_slave_setup(void)
{
    // Выключаем SPI перед настройкой
    spi_disable(SPI_DEVICE);
    
    // Настройка режима ведомого
    spi_set_slave_mode(SPI_DEVICE);

    // Настройка формата данных
    spi_set_baudrate_prescaler(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_256);
    spi_set_clock_polarity_0(SPI_DEVICE);                               // CPOL = 0
    spi_set_clock_phase_0(SPI_DEVICE);                                  // CPHA = 0
    spi_set_full_duplex_mode(SPI_DEVICE);                               // Полнодуплексный режим
    spi_set_dff_8bit(SPI_DEVICE);                                       // 8 бит данных
    spi_send_msb_first(SPI_DEVICE);                                     // Старший бит первый
    
    // Аппаратное управление NSS
    spi_disable_software_slave_management(SPI_DEVICE);
    
    // Включаем SPI
    spi_enable(SPI_DEVICE);
}

void timer_setup(void) {
    // Останавливаем таймер перед настройкой
    timer_disable_counter(TIM2);

    // Настройка таймера на 1 МГц
    timer_set_prescaler(TIM2, 84 - 1); // 1 МГц
    timer_set_period(TIM2, 0xFFFFFFFF);
    timer_continuous_mode(TIM2);
    
    // Настройка Input Capture для канала 4
    timer_ic_set_input(TIM2, TIM_IC4, TIM_IC_IN_TI4);
    timer_ic_set_filter(MEASURE_TIMER, TIM_IC4, TIM_IC_OFF);
    timer_ic_set_prescaler(MEASURE_TIMER, TIM_IC4, TIM_IC_PSC_OFF);
    // ЗАХВАТ ПО ОБОИМ ФРОНТАМ 
    timer_ic_set_polarity(TIM2, TIM_IC4, TIM_IC_BOTH);
    
    timer_ic_enable(TIM2, TIM_IC4); // Включение канала
    
    // Включаем прерывание по захвату канала 4
    timer_enable_irq(TIM2, TIM_DIER_CC4IE);
    nvic_enable_irq(NVIC_TIM2_IRQ);
    nvic_set_priority(NVIC_TIM2_IRQ, 0);

    // Запускаем таймер
    timer_enable_counter(TIM2);

}

// Обработчик прерывания TIM2
void tim2_isr(void) {
    if (timer_get_flag(TIM2, TIM_SR_CC4IF)) {
        timer_clear_flag(TIM2, TIM_SR_CC4IF);
        
        uint32_t capture_value = TIM_CCR4(TIM2);
        if (gpio_get(ECHO_PORT, ECHO_PIN)) { 
            // Передний фронт - начало импульса
            pulse_start = capture_value;
        } 
        else {
            // Задний фронт - конец импульса
            pulse_end = capture_value;
            
            // Вычисляем ширину импульса
            if (pulse_end >= pulse_start) {
                pulse_width = pulse_end - pulse_start;
            } else {
                pulse_width = (0xFFFFFFFF - pulse_start) + pulse_end + 1;
            }
            measurement_ready = true;
            measurement_count++;
        }
    }
}

uint32_t measure_distance(void) {
    // Сброс 
    measurement_ready = false;
    pulse_start = 0;
    pulse_end = 0;
    pulse_width = 0;
    
    // TRIG импульс
    gpio_clear(TRIG_PORT, TRIG_PIN);
    delay_us(2);
    gpio_set(TRIG_PORT, TRIG_PIN);
    delay_us(10);
    gpio_clear(TRIG_PORT, TRIG_PIN);
    
    // Ожидание измерения
    for (volatile uint32_t i = 0; i < 840000; i++) { 
        if (measurement_ready) {
            uint32_t distance = (pulse_width*10) / 578; // Расёт расстояния ( *10-см, *100-мм )
            return distance;
        }
    }
    return 0; // Таймаут
}

// Всё для SPI

// Функция для отправки данных
void spi2_slave_send(uint8_t data)
{
    // Ждем, когда буфер передачи станет пустым
    while (!(SPI_SR(SPI_DEVICE) & SPI_SR_TXE)) {
        // Ожидание готовности
    }
    
    // Записываем данные для отправки
    //SPI_DR(SPI_DEVICE) = data;
    spi_send(SPI2, data);
}

// Проверка активности NSS (выбор ведомого)
bool spi2_slave_selected(void)
{
    // NSS активен в низком уровне
    return (gpio_get(SPI_PORT, SPI_NSS_PIN) == 0);
}

// Для отправки в UART

void send_distance_cm(uint32_t distance_cm) {
    uart_send_string("Расстояние: ");
    my_usart_print_int(USART1, distance_cm);
    uart_send_string(" См\r\n");
    
    if (distance_cm >= 2 && distance_cm <= 400) { // При правильных измерениях
        gpio_set(LED_PORT, LED_PIN);
        delay_ms(30);
        gpio_clear(LED_PORT, LED_PIN);
    }
}

void uart_send_string(const char *str) {
    while (*str) {
        usart_send_blocking(USART1, *str);
        str++;
    }
}

void my_usart_print_int(uint32_t usart, int32_t value) {
    int8_t i;
    int8_t nr_digits = 0;
    char buffer[25];

    if (value < 0) {
        usart_send_blocking(usart, '-');
        value = value * -1;
    }

    if (value == 0) {
        usart_send_blocking(usart, '0');
        return;
    }

    while (value > 0) {
        buffer[nr_digits++] = "0123456789"[value % 10];
        value /= 10;
    }

    for (i = nr_digits-1; i >= 0; i--) {
        usart_send_blocking(usart, buffer[i]);
    }
}

// Задержки
void delay_us(uint32_t us) {
    for (volatile uint32_t i = 0; i < us * 84; i++);
}

void delay_ms(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

// Команды для починки
// sudo chmod 666 /dev/ttyUSB0
// openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "init; reset; shutdown"