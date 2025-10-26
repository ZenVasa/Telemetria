
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
