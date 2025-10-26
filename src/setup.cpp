#include "../inc/setup.hpp"

// Настройка тактирования
void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
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
/*
// Индикация состояния измерения светодиодом
void indicate_measurement(uint16_t distance) {
    if (distance < 2  || distance > 400) {   // При ошибочных измерениях
        gpio_set(LED_PORT, LED_PIN);
        delay_ms(40);
        gpio_clear(LED_PORT, LED_PIN);
        delay_ms(40);
    } 
    else if (distance >= 2 && distance <= 400) { // При правильных измерениях
        gpio_set(LED_PORT, LED_PIN);
        delay_ms(20);
        gpio_clear(LED_PORT, LED_PIN);
        delay_ms(20);

    }
}*/

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













