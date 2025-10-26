#ifndef SETUP_HPP
#define SETUP_HPP

// Библиотеки:
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <stdbool.h>
#include <stdio.h>

// Универсальный шаблон UART
template<uint32_t DEVICE>
void uart_setup(uint32_t port, uint16_t tx_pin, uint16_t rx_pin, uint8_t gpio_af) {

    // Включаем тактирование порта
    if (port == GPIOA) rcc_periph_clock_enable(RCC_GPIOA);
    else if (port == GPIOB) rcc_periph_clock_enable(RCC_GPIOB);
    else if (port == GPIOC) rcc_periph_clock_enable(RCC_GPIOC);
    else if (port == GPIOD) rcc_periph_clock_enable(RCC_GPIOD);
    else if (port == GPIOE) rcc_periph_clock_enable(RCC_GPIOE);
    else if (port == GPIOF) rcc_periph_clock_enable(RCC_GPIOF);
    
    // Включаем тактирование устройства
    if (DEVICE == USART1) rcc_periph_clock_enable(RCC_USART1);
    else if (DEVICE == USART2) rcc_periph_clock_enable(RCC_USART2);
    else if (DEVICE == USART3) rcc_periph_clock_enable(RCC_USART3);
    else if (DEVICE == UART4) rcc_periph_clock_enable(RCC_UART4);
    else if (DEVICE == UART5) rcc_periph_clock_enable(RCC_UART5);
    else if (DEVICE == USART6) rcc_periph_clock_enable(RCC_USART6);
    
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


// Прототипы функций:
void gpio_setup(void);
void timer_setup(void);

void clock_setup(void); // Настройка тактирования

// Функции для отправки
void uart_send_string(const char *str);
void send_distance_cm(int value);
void my_usart_print_int(uint32_t usart, int16_t value);

uint16_t measure_distance(void); // Функция измерения расстояния

uint16_t get_distance_cm(void); // Вычисление расстояния в сантиметрах

void led_setup(void);   // Индкация светодиодом
void indicate_measurement(uint16_t distance);

// Задержки
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

// Прерывание
void tim2_isr(void);


#endif