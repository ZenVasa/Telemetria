
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <stdbool.h>

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


void clock_setup(void){
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_84MHZ]);

    // Включаем тактирование для GPIOA, GPIOD, TIM2 и USART
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_clock_enable(RCC_USART1);
    //rcc_periph_clock_enable(RCC_USART3);
}

// Основная функция
int main(void) {
    clock_setup();
    
    gpio_setup();
    timer_setup();
    uart_setup();
    
    uart_send_string(" Запуск HC-SR04 \r\n");
    
    while (true) {
        uint32_t distance = measure_distance();
        send_distance_cm(distance); // Отправка расстояние в UART3 и моргание светодиодом
        delay_ms(30);
    }
    
    return 0;
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
            uint32_t distance = (pulse_width*100) / 578; // Расёт расстояния ( *10-см, *100-мм )
            return distance;
        }
    }
    return 0; // Таймаут
}


// Для отправки в UART
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

void send_distance_cm(uint32_t distance_cm) {
    uart_send_string("Расстояние: ");
    my_usart_print_int(USART1, distance_cm);
    uart_send_string(" мм\r\n");
    
    if (distance_cm >= 20 && distance_cm <= 4000) { // При правильных измерениях
        gpio_set(LED_PORT, LED_PIN);
        delay_ms(30);
        gpio_clear(LED_PORT, LED_PIN);
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

//sudo chmod 666 /dev/ttyUSB0
