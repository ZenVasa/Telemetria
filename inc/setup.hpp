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


// Прототипы функций:
void gpio_setup(void);
void timer_setup(void);
void USART1_setup(void);

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