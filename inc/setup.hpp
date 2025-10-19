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

// Прототип шаблона UART
template<uint32_t DEVICE>
void uart_setup(uint32_t port, uint16_t tx_pin, uint16_t rx_pin, uint32_t gpio_af);

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

// Прототипы функций
void uart_setup(void);
void gpio_setup(void);
void timer_setup(void);


void tim2_isr(void);

void led_setup(void);

// Функции для отправки
void uart_send_string(const char *str);
uint16_t get_distance_cm(void);
uint16_t measure_distance(void);
void indicate_measurement(uint16_t distance);
void my_usart_print_int(uint32_t usart, int16_t value);


void send_distance_cm(int value);


void clock_setup(void);




#endif