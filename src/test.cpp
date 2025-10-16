#include <libopencm3/stm32/rcc.h>
#include <libopencm32/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <stdbool.h>

// Настройка тактирования
void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    
    // Включаем тактирование SPI2 и GPIO
    rcc_periph_clock_enable(RCC_SPI2);
    rcc_periph_clock_enable(RCC_GPIOB);
}

// Настройка SPI2 в режиме ведомого
void spi2_slave_setup(void)
{
    // Сброс SPI2
    spi_disable(SPI2);
    
    // Настройка параметров SPI для ведомого
    spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_8, SPI_CR1_CPOL_CLK_TO_0,
                   SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT,
                   SPI_CR1_MSBFIRST);
    
    // Переключаем в режим ведомого
    spi_set_slave_mode(SPI2);
    
    // Аппаратное управление NSS
    spi_disable_software_slave_management(SPI2);
    spi_set_nss_low(SPI2);
    
    // Включаем SPI
    spi_enable(SPI2);
    
    printf("SPI2 Slave initialized\r\n");
}

// Улучшенная функция отправки данных
void spi2_slave_send(uint8_t data)
{
    // Ждем, когда буфер передачи станет пустым
    while (!(SPI_SR(SPI2) & SPI_SR_TXE)) {
        // Можно добавить таймаут
    }
    
    // Записываем данные для отправки
    SPI_DR(SPI2) = data;
    
    // Ждем завершения передачи
    while (SPI_SR(SPI2) & SPI_SR_BSY) {
        // Ожидание завершения
    }
}

// Проверка активности NSS
bool spi2_slave_selected(void)
{
    return (gpio_get(GPIOB, GPIO12) == 0); // NSS активен в низком уровне
}

// Функция для приема данных (если нужно)
uint8_t spi2_slave_receive(void)
{
    while (!(SPI_SR(SPI2) & SPI_SR_RXNE));
    return SPI_DR(SPI2);
}

// Настройка GPIO для SPI
void gpio_setup(void)
{
    // SPI2 на выводах PB13(SCK), PB14(MISO), PB15(MOSI), PB12(NSS)
    
    // SCK (вход)
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO13);
    
    // MOSI (вход)
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO15);
    
    // MISO (выход)
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO14);
    
    // NSS (вход)
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO12);
    
    // Установка альтернативной функции SPI2 (AF5)
    gpio_set_af(GPIOB, GPIO_AF5, GPIO12 | GPIO13 | GPIO14 | GPIO15);
}

// Простая задержка
void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms * 1000; i++) {
        __asm__("nop");
    }
}

int main(void)
{
    clock_setup();
    gpio_setup();
    spi2_slave_setup();
    
    uint8_t counter = 0;
    const char message[] = "HELLO";
    uint8_t msg_index = 0;
    
    printf("SPI Slave started\r\n");
    
    while (true) {
        // Проверяем, выбран ли ведомый
        if (spi2_slave_selected()) {
            printf("SPI Slave selected, sending data...\r\n");
            
            // Отправляем данные по одному байту
            spi2_slave_send(message[msg_index]);
            msg_index++;
            
            if (msg_index >= sizeof(message) - 1) {
                msg_index = 0;
            }
            
            // Небольшая задержка между байтами
            delay_ms(10);
        } else {
            // Сбрасываем индекс, когда не выбран
            msg_index = 0;
        }
        
        delay_ms(100);
    }
    
    return 0;
}









#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <stdbool.h>
#include <stdio.h>
#include <libopencm3/stm32/spi.h>

// Настройка тактирования
void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    
    // Включаем тактирование SPI2 и GPIO
    rcc_periph_clock_enable(RCC_SPI2);
    rcc_periph_clock_enable(RCC_GPIOB);
}

// __________________________________________________________________________________

// UART
constexpr uint32_t UART_PORT = GPIOD;
constexpr uint16_t UART_TX_PIN = GPIO8;     // PD8 - TX <- RX(надо)
constexpr uint16_t UART_RX_PIN = GPIO9;     // PD9 - RX <- TX
constexpr uint32_t UART_DEVICE = USART3;

// Таймер
constexpr uint32_t MEASURE_TIMER = TIM2;


// Прототипы функций
void uart_setup(void);
void gpio_setup(void);
void timer_setup(void);
void tim2_isr(void);
void led_setup(void);
void uart_send_string(const char *str);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
uint16_t get_distance_cm(void);
uint16_t measure_distance(void);
void indicate_measurement(uint16_t distance);

void send_distance_cm(int value);
void my_usart_print_int(uint32_t usart, int16_t value);

void clock_setup(void);
void spi2_slave_setup(void);
void spi2_slave_send(uint8_t data);
bool spi2_slave_selected(void);

void spi2_isr(void);

// Основная функция программы
int main(void) {
    // Настройка системных часов на 164 МГц 
    clock_setup();
    
    // Инициализация периферии
    uart_setup();
    gpio_setup();

    spi2_slave_setup();
    
    // Небольшая задержка для стабилизации системы
    delay_ms(100);
    
    //uint8_t tx_data = 0x00;
    //uint8_t rx_data;

    // Основной цикл программы
    while (true) {

        // Проверяем, выбран ли ведомый
        if (spi2_slave_selected()) {
            // Если есть данные для отправки - отправляем
            spi2_slave_send('$');
            spi2_slave_send('2');
            spi2_slave_send('1');
            spi2_slave_send('2');
            spi2_slave_send(';');
        
        }
        // Задержка между измерениями
        delay_ms(100);

    }
    
    return 0;
}

// Настройка SPI2 в режиме ведомого
void spi2_slave_setup(void)
{
    // Сброс SPI2
    spi_disable(SPI2);
    
    // Инициализация SPI в режиме ведомого
    SPI_CR1(SPI2) = 0;
    SPI_CR2(SPI2) = 0;
    
    // Настройка параметров SPI
    spi_set_slave_mode(SPI2);                    // 8-битный режим
    spi_set_baudrate_prescaler(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_32);
    spi_set_clock_polarity_0(SPI2);
    spi_set_clock_phase_0(SPI2);
    spi_set_dff_8bit(SPI2);
    spi_send_msb_first(SPI2);
    
    spi_set_full_duplex_mode(SPI2);

    // Аппаратное управление NSS
    spi_disable_software_slave_management(SPI2);
    spi_set_nss_low(SPI2);
    
    // Включаем прерывания (опционально)
    spi_enable_rx_buffer_not_empty_interrupt(SPI2);
    spi_enable_tx_buffer_empty_interrupt(SPI2);
    
    // Включаем SPI
    spi_enable(SPI2);
}

// Функция для отправки данных (когда мастер запрашивает)
void spi2_slave_send(uint8_t data)
{
    // Ждем, когда буфер передачи станет пустым
    while (!(SPI_SR(SPI2) & SPI_SR_TXE));
    
    // Записываем данные для отправки
    spi_send(SPI2, data);
}
/*
// Функция для чтения принятых данных
uint8_t spi2_slave_receive(void)
{
    // Ждем, когда появятся данные в буфере приема
    while (!(SPI_SR(SPI2) & SPI_SR_RXNE));
    
    // Читаем принятые данные
    return spi_read(SPI2);
}*/

// Проверка активности NSS (выбор ведомого)
bool spi2_slave_selected(void)
{
    return (gpio_get(GPIOB, GPIO12) == 0); // NSS активен в низком уровне
}

/*
// Обработчик прерываний SPI2 (если используются прерывания)
void spi2_isr(void)
{
    // Проверяем флаг приема данных
    if (SPI_SR(SPI2) & SPI_SR_RXNE) {
        uint8_t received_data = spi_read(SPI2);
        
        // Обработка принятых данных
        // ...
    }
    
    // Проверяем флаг готовности передачи
    if (SPI_SR(SPI2) & SPI_SR_TXE) {
        // Можно записать новые данные для отправки
        // spi_send8(SPI2, next_data);
    }
}
*/

// Настройка NVIC для прерываний SPI2
void nvic_setup(void)
{
    nvic_enable_irq(NVIC_SPI2_IRQ);
    nvic_set_priority(NVIC_SPI2_IRQ, 1);
}


//-----------UART----------------------------------------------------------------------

void uart_setup(void) {
    // Включаем тактирование порта D и USART3
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_USART3);
    
    // Настраиваем пины UART
    gpio_mode_setup(UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, 
                   UART_TX_PIN | UART_RX_PIN);
    gpio_set_af(UART_PORT, GPIO_AF7, UART_TX_PIN | UART_RX_PIN);
    
    // Настраиваем USART3
    usart_set_baudrate(UART_DEVICE, 115200);
    usart_set_databits(UART_DEVICE, 8);
    usart_set_stopbits(UART_DEVICE, USART_STOPBITS_1);
    usart_set_parity(UART_DEVICE, USART_PARITY_NONE);
    usart_set_flow_control(UART_DEVICE, USART_FLOWCONTROL_NONE);
    usart_set_mode(UART_DEVICE, USART_MODE_TX_RX);

    // Включаем USART3
    usart_enable(UART_DEVICE);
}
//---------------------------------------------------------------------------------

// Отправка строки по UART
void uart_send_string(const char *str) {
    while (*str) {
        usart_send_blocking(UART_DEVICE, *str);
        str++;
    }
}

// Отправка числа int по UART
void send_distance_cm(int value) {
    char buffer[16];
    
    // Преобразуем число в строку
    snprintf(buffer, sizeof(buffer), "%d\r\n", value);
    
    // Отправляем строку
    for (char *p = buffer; *p; p++) {
        usart_send_blocking(UART_DEVICE, *p);
    }
}

void my_usart_print_int(uint32_t usart, int16_t value) {
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

    usart_send_blocking(usart, '\r');
    usart_send_blocking(usart, '\n');
}

//---------------------------------------------------------------------------------
// Инициализация GPIO
void gpio_setup(void) {

    // SPI2 на выводах PB13, PB14, PB15, PB12
    // PB13 - SCK (вход), PB14 - MISO (выход), PB15 - MOSI (вход), PB12 - NSS (вход)
    
    // SCK (вход)
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO13);
    
    // MOSI (вход)
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO15);
    
    // MISO (выход)
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO14);
    
    // NSS (вход с подтяжкой)
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO12);
    
    // Установка альтернативной функции SPI2 (AF5)
    gpio_set_af(GPIOB, GPIO_AF5, GPIO12 | GPIO13 | GPIO14 | GPIO15);

}


// Задержка в микросекундах
void delay_us(uint32_t us) {
    for (volatile uint32_t i = 0; i < us * 84; i++); // Тактовая частота 84 МГц
}

// Задержка в миллисекундах
void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}
