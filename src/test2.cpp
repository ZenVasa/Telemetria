
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <stdbool.h>

// Определения пинов
#define TRIG_PORT GPIOA
#define TRIG_PIN  GPIO5    // PA5 - TRIG
#define ECHO_PORT GPIOA  
#define ECHO_PIN  GPIO3    // PA3 - TIM2_CH3

//#define UART_DEVICE USART1
#define MEASURE_TIMER TIM2

// UART
constexpr uint32_t UART_PORT = GPIOD;
constexpr uint16_t UART_TX_PIN = GPIO8;     // PD8 - TX <- RX(надо)
constexpr uint16_t UART_RX_PIN = GPIO9;     // PD9 - RX <- TX
constexpr uint32_t UART_DEVICE = USART3;

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
uint32_t get_distance_cm(void);
uint32_t measure_distance(void);
void my_usart_print_int(uint32_t usart, int32_t value);

// Основная функция
int main(void) {
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    
    uart_setup();
    gpio_setup();
    timer_setup();
    
    uart_send_string(" Запуск HC-SR04 \r\n");
    
    while (true) {
        uint32_t distance = measure_distance();
        send_distance_cm(distance);
        delay_ms(100);
    }
    
    return 0;
}


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

void uart_send_string(const char *str) {
    while (*str) {
        usart_send_blocking(USART3, *str);
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

    usart_send_blocking(usart, '\r');
    usart_send_blocking(usart, '\n');
}



void send_distance_cm(uint32_t distance_cm) {
    uart_send_string("Расстояние: ");
    my_usart_print_int(USART3, distance_cm);
    uart_send_string(" см\r\n");
}

void gpio_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    
    // TRIG как выход
    gpio_mode_setup(TRIG_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TRIG_PIN);
    gpio_clear(TRIG_PORT, TRIG_PIN);
    
    // ECHO как альтернативная функция для TIM2
    gpio_mode_setup(ECHO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, ECHO_PIN);
    gpio_set_af(ECHO_PORT, GPIO_AF1, ECHO_PIN);
}

void timer_setup(void) {
    rcc_periph_clock_enable(RCC_TIM2);
    
    // Останавливаем таймер перед настройкой
    timer_disable_counter(TIM2);
    
    // Настройка таймера на 1 МГц
    timer_set_prescaler(TIM2, 168 - 1); // 168 МГц / 168 = 1 МГц
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
        
        const uint32_t capture_value = TIM_CCR4(TIM2);
        
        if (gpio_get(ECHO_PORT, ECHO_PIN)) { //
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


uint32_t get_distance_cm(void) {
    return pulse_width / 58.0;
}

uint32_t measure_distance(void) {
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
    for (volatile uint32_t i = 0; i < 1000000; i++) {
        if (measurement_ready) {
            uint32_t distance = get_distance_cm();
            //if (distance >= 2 && distance <= 400) {
            return distance;
            //}
            //return 0;
        }
    }
    
    return 0; // Таймаут
}

void delay_us(uint32_t us) {
    for (volatile uint32_t i = 0; i < us * 168; i++);
}

void delay_ms(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}




/*
void tim2_isr(void) {
    if (timer_get_flag(TIM2, TIM_SR_CC3IF)) {
        timer_clear_flag(TIM2, TIM_SR_CC3IF);
        
        // Если ECHO в LOW - это задний фронт, значение = длительность импульса
        if (!gpio_get(ECHO_PORT, ECHO_PIN)) {
            pulse_width = TIM_CCR3(TIM2);
            measurement_ready = true;
            measurement_count++;
           
        }
    }
}*/


/*

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>

void delay_us(uint32_t us) {
    for (volatile uint32_t i = 0; i < us * 168; i++);
}

void gpio_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO7);
}

void timer_setup(void) {
    rcc_periph_clock_enable(RCC_TIM2);
    timer_set_prescaler(TIM2, 167); // 1 МГц = 1 мкс/тик
    timer_set_period(TIM2, 0xFFFFFFFF);
    timer_enable_counter(TIM2);
}

void uart_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);
}

void uart_send_string(const char* str) {
    while (*str) usart_send_blocking(USART1, *str++);
}

void uart_send_int(uint32_t value) {
    char buffer[16];
    char* ptr = buffer + 15;
    *ptr = '\0';
    if (value == 0) {
        uart_send_string("0");
        return;
    }
    do {
        *--ptr = '0' + (value % 10);
        value /= 10;
    } while (value > 0);
    uart_send_string(ptr);
}

int main(void) {
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    uart_setup();
    gpio_setup();
    timer_setup();
    
    uart_send_string("HC-SR04 with Correct Timeouts\r\n");
    
    while (1) {
        // TRIG импульс
        gpio_clear(GPIOA, GPIO6);
        delay_us(4);
        gpio_set(GPIOA, GPIO6);
        delay_us(10);
        gpio_clear(GPIOA, GPIO6);
        
        // ПРАВИЛЬНЫЕ ТАЙМАУТЫ:
        uint32_t timeout_echo_start = 50000;  // 50ms - ждем начала ECHO
        uint32_t timeout_echo_end = 30000;    // 30ms - ждем конца ECHO (макс 400 см)
        
        uint32_t measurement_start = timer_get_counter(TIM2);
        bool echo_detected = false;
        
        // Ждем начала ECHO (передний фронт)
        while (!gpio_get(GPIOA, GPIO7)) {
            if ((timer_get_counter(TIM2) - measurement_start) > timeout_echo_start) {
                uart_send_string("Timeout: No ECHO start within 50ms\r\n");
                break;
            }
        }
        
        if (gpio_get(GPIOA, GPIO7)) {
            // ECHO начался
            uint32_t pulse_start = timer_get_counter(TIM2);
            uint32_t echo_start_time = pulse_start;
            
            // Ждем конца ECHO (задний фронт)
            while (gpio_get(GPIOA, GPIO7)) {
                if ((timer_get_counter(TIM2) - echo_start_time) > timeout_echo_end) {
                    uart_send_string("Timeout: ECHO too long (>30ms)\r\n");
                    break;
                }
            }
            
            if (!gpio_get(GPIOA, GPIO7)) {
                // ECHO закончился
                uint32_t pulse_end = timer_get_counter(TIM2);
                uint32_t duration_us;
                
                if (pulse_end >= pulse_start) {
                    duration_us = pulse_end - pulse_start;
                } else {
                    duration_us = (0xFFFFFFFF - pulse_start) + pulse_end;
                }
                
                uint32_t distance_cm = duration_us / 58;
                
                // Фильтр реальных значений
                if (distance_cm >= 2 && distance_cm <= 400) {
                    uart_send_string("Distance: ");
                    uart_send_int(distance_cm);
                    uart_send_string(" cm (");
                    uart_send_int(duration_us);
                    uart_send_string(" us)\r\n");
                } else {
                    uart_send_string("Invalid: ");
                    uart_send_int(distance_cm);
                    uart_send_string(" cm\r\n");
                }
            }
        }
        
        delay_us(100000); // 100ms между измерениями
    }
    
    return 0;
}

*/





/*

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

void uart_send_string(const char *str) {
    while (*str) {
        usart_send_blocking(USART1, *str);
        str++;
    }
}



int main(void) {
    // Тактирование
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    
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
    
    // Настраиваем светодиод на PA1
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
    gpio_clear(GPIOA, GPIO1); // Изначально выключен
    
    uart_send_string("CH340 + LED on PA1 - STM32F407!\r\n");
    
    int counter = 0;
    
    while (1) {
        // Мигаем светодиодом на PA1
        gpio_toggle(GPIOA, GPIO1);
        
        // Отправляем сообщение в UART
        uart_send_string("LED PA1 toggled! Count: ");
        
        // Отправляем число
        char buffer[10];
        char *p = buffer;
        int num = counter;
        
        if (num == 0) {
            usart_send_blocking(USART1, '0');
        } else {
            while (num > 0) {
                *p++ = '0' + (num % 10);
                num /= 10;
            }
            while (p > buffer) {
                usart_send_blocking(USART1, *--p);
            }
        }
        
        uart_send_string("\r\n");
        counter++;
        
        // Задержка ~0.5 секунды
        for (volatile int i = 0; i < 1000000; i++);
    }
    
    return 0;
}
*/
