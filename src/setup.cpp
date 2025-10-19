#include "../inc/setup.hpp"




// Настройка тактирования
void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
}


// Универсальный шаблон UART

template<uint32_t DEVICE>
void uart_setup(uint32_t port, uint16_t tx_pin, uint16_t rx_pin, uint32_t gpio_af) {

    // Определения тактирования
    uint32_t port_rcc = 0;
    uint32_t device_rcc = 0;
    
    // Определяем тактирование порта
    switch(port) {
        case GPIOA: port_rcc = RCC_GPIOA; break;
        case GPIOB: port_rcc = RCC_GPIOB; break;
        case GPIOC: port_rcc = RCC_GPIOC; break;
        case GPIOD: port_rcc = RCC_GPIOD; break;
        case GPIOE: port_rcc = RCC_GPIOE; break;
        case GPIOF: port_rcc = RCC_GPIOF; break;
    }
    // Определяем тактирование устройства
    switch(DEVICE) {
        case USART1: device_rcc = RCC_USART1; break;
        case USART2: device_rcc = RCC_USART2; break;
        case USART3: device_rcc = RCC_USART3; break;
        case UART4:  device_rcc = RCC_UART4; break;
        case UART5:  device_rcc = RCC_UART5; break;
        case USART6: device_rcc = RCC_USART6; break;
    }

    rcc_periph_clock_enable(port_rcc);
    rcc_periph_clock_enable(device_rcc);
    
    gpio_mode_setup(port, GPIO_MODE_AF, GPIO_PUPD_NONE, tx_pin | rx_pin);
    gpio_set_af(port, gpio_af, tx_pin | rx_pin);
    
    usart_set_baudrate(DEVICE, 115200);
    usart_set_databits(DEVICE, 8);
    usart_set_stopbits(DEVICE, USART_STOPBITS_1);
    usart_set_parity(DEVICE, USART_PARITY_NONE);
    usart_set_flow_control(DEVICE, USART_FLOWCONTROL_NONE);
    usart_set_mode(DEVICE, USART_MODE_TX_RX);
    usart_enable(DEVICE);
}




// Пины и периферия
constexpr uint32_t TRIG_PORT = GPIOA;
constexpr uint16_t TRIG_PIN = GPIO5;        // PA5 - TIM2_CH1 (вместо PA0)
constexpr uint32_t ECHO_PORT = GPIOA;  
constexpr uint16_t ECHO_PIN = GPIO3;        // PA3 - TIM2_CH4 (вместо PA1)

// Инициализация GPIO
void gpio_setup(void) {
    // Включаем тактирование порта A
    rcc_periph_clock_enable(RCC_GPIOA);
    
    // Настраиваем TRIG как выход
    gpio_mode_setup(TRIG_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TRIG_PIN);
    gpio_set_output_options(TRIG_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, TRIG_PIN);
    
    // Настраиваем ECHO как вход с альтернативной функцией (TIM2_CH2)
    gpio_mode_setup(ECHO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, ECHO_PIN);
    gpio_set_af(ECHO_PORT, GPIO_AF1, ECHO_PIN);
    
    // Изначально устанавливаем TRIG в низкий уровень
    gpio_clear(TRIG_PORT, TRIG_PIN);

}
 
// LED
constexpr uint32_t LED_PORT = GPIOA;
constexpr uint16_t LED_PIN = GPIO1;

// Инициализация светодиода
void led_setup(void) {
    // Включаем тактирование порта A
    rcc_periph_clock_enable(RCC_GPIOA); 
    // Настраиваем пин как выход        
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN); 
    // Гарантируем выключенное состояние 
    gpio_clear(LED_PORT, LED_PIN); 
}

// Индикация состояния измерения светодиодом
void indicate_measurement(uint16_t distance) {
    if (distance < 2  || distance > 400) {   // При ошибочных измерениях
        gpio_set(LED_PORT, LED_PIN);
        delay_ms(60);
        gpio_clear(LED_PORT, LED_PIN);
    } 
    else if (distance >= 2 && distance <= 400) { // При правильных измерениях
        gpio_set(LED_PORT, LED_PIN);
        delay_ms(20);
        gpio_clear(LED_PORT, LED_PIN);
        delay_ms(20);
        gpio_set(LED_PORT, LED_PIN);
        delay_ms(20);
        gpio_clear(LED_PORT, LED_PIN);
    }
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













