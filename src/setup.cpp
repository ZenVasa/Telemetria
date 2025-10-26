#include "../inc/setup.hpp"


void USART1_setup(void){
    // Включаем тактирование GPIOA и USART1
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);
    
    // Настраиваем USART1 (для CH340) - PA9 (TX), PA10 (RX)
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);
    
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_enable(USART1);

}


// Пины и периферия
constexpr uint32_t TRIG_PORT = GPIOA;
constexpr uint16_t TRIG_PIN = GPIO5;        // PA5 - TIM2_CH1 (вместо PA0)
constexpr uint32_t ECHO_PORT = GPIOA;  
constexpr uint16_t ECHO_PIN = GPIO3;        // PA3 - TIM2_CH4 (вместо PA1)

// Инициализация GPIO
void gpio_setup(void) {
    // Включаем тактирование порта A
    //rcc_periph_clock_enable(RCC_GPIOA);
    
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
    //rcc_periph_clock_enable(RCC_GPIOA); 
    // Настраиваем пин как выход        
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN); 
    // Гарантируем выключенное состояние 
    gpio_clear(LED_PORT, LED_PIN); 
}

// Индикация состояния измерения светодиодом
void indicate_measurement(uint16_t distance) {
    if (distance < 2  || distance > 400) {   // При ошибочных измерениях
        gpio_toggle(LED_PORT, LED_PIN);
        delay_ms(500);
    } 
    else if (distance >= 2 && distance <= 400) { // При правильных измерениях
        gpio_toggle(LED_PORT, LED_PIN);
        delay_ms(100);

    }
}

// Задержка в микросекундах
void delay_us(uint32_t us) {
    for (volatile uint32_t i = 0; i < us * 168; i++); // Тактовая частота 84 МГц
}

// Задержка в миллисекундах
void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}













