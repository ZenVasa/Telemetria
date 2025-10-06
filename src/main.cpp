#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <stdbool.h>
#include <stdio.h>

// Пины и периферия
constexpr uint32_t TRIG_PORT = GPIOA;
constexpr uint16_t TRIG_PIN = GPIO0;        // PA0 - TIM2_CH1
constexpr uint32_t ECHO_PORT = GPIOA;
constexpr uint16_t ECHO_PIN = GPIO1;        // PA1 - TIM2_CH2

// UART
constexpr uint32_t UART_PORT = GPIOD;
constexpr uint16_t UART_TX_PIN = GPIO8;     // PD8 - TX <- RX(надо)
constexpr uint16_t UART_RX_PIN = GPIO9;     // PD9 - RX <- TX
constexpr uint32_t UART_DEVICE = USART3;

// Таймер
constexpr uint32_t MEASURE_TIMER = TIM2;

// LED
constexpr uint32_t LED_PORT = GPIOD;
constexpr uint16_t LED_PIN = GPIO15;

// Структура для хранения состояния измерения
struct MeasurementState 
{
    uint32_t pulse_start{0};        // Время начала импульса ECHO
    uint32_t pulse_end{0};          // Время окончания импульса ECHO  
    uint32_t pulse_width{0};        // Длительность импульса (тики)
    bool measurement_ready{false};  // Флаг готовности измерения
    uint32_t measurement_count{0};  // Флаг таймаута измерения

    
};

// Глобальные переменные
static volatile MeasurementState g_measurement;



// Прототипы функций
void uart_setup(void);
void gpio_setup(void);
void timer_setup(void);
void tim2_isr(void);
void led_setup(void);
void uart_send_string(const char *str);
void send_distance_cm(float distance_cm);
void uart_send_message(float float_value, const char *text);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
float get_distance_cm(void);
float measure_distance(void);
void indicate_measurement(float distance);
void my_usart_print_int(uint32_t usart, int16_t value);

// Основная функция программы
int main(void) {
    // Настройка системных часов на 84 МГц от HSE 8 МГц
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
    
    // Инициализация периферии
    uart_setup();
    gpio_setup();
    timer_setup();
    led_setup();
    
    // Переменная для хранения расстояния
    float distance_cm = 0;
    
    // Небольшая задержка для стабилизации системы
    delay_ms(100);
    
    uart_send_string("HC-SR04 Distance Measurement Started\r\n");
    
    // Основной цикл программы
    while (true) {
        // Выполняем измерение расстояния
        distance_cm = measure_distance();
        
        // Индикация результата
        indicate_measurement(distance_cm);
        
        // Отправляем расстояние по UART
        uart_send_message(distance_cm, " См");
        //send_distance_cm(distance_cm);

        // Задержка между измерениями
        delay_ms(60);
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
//---------------------------------------------------------------------------------

// Функция для отправки форматированного сообщения с текстом и значением
void uart_send_message(float float_value, const char *text)
{
    char buffer1[80];
    
    // Форматируем строку с текстом и числами
    snprintf(buffer1, sizeof(buffer1)," %.2f %s \r\n", float_value, text);
    
    // Отправляем сформированное сообщение
    for (const char *p = buffer1; *p; p++) 
    {
        usart_send_blocking(UART_DEVICE, *p);
    }
}

// Отправка строки по UART
void uart_send_string(const char *str) {
    while (*str) {
        usart_send_blocking(UART_DEVICE, *str);
        str++;
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


// Прямой вызов с текстом и числом
// uart_send_message("Текст:", distance);

// Отправка расстояния по UART

void send_distance_cm(float distance_cm) {
    char uart_buffer[64];
    
    if (distance_cm < 0) {
        my_usart_print_int(UART_DEVICE, static_cast<int16_t>(distance_cm*10));
        //snprintf(uart_buffer, sizeof(uart_buffer), "Ошибка: тайм-аут измерения или расстояние\r\n");
        //uart_send_string(uart_buffer);
    } else {
        my_usart_print_int(UART_DEVICE, static_cast<int16_t>(distance_cm*10));
    }
}

//---------------------------------------------------------------------------------
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

// Инициализация таймера TIM2 для Capture измерений
void timer_setup(void) {
    rcc_periph_clock_enable(RCC_TIM2);
    timer_disable_counter(MEASURE_TIMER);
    
    // Настройка предделителя для получения 1 МГц
    // 84 МГц / 84 = 1 МГц (1 тик = 1 микросекунда)
    timer_set_prescaler(MEASURE_TIMER, 84 - 1);

    // Установка максимального периода для 32-битного таймера
    // 0xFFFFFFFF = 4,294,967,295 тиков ≈ 4295 секунд при 1 МГц
    // Гарантирует отсутствие переполнения во время измерений HC-SR04
    timer_set_period(MEASURE_TIMER, 0xFFFFFFFF);

    // Непрерывный режим 
    timer_continuous_mode(MEASURE_TIMER);
    
    // Настройка канала 2 для Input Capture
    timer_ic_set_input(MEASURE_TIMER, TIM_IC2, TIM_IC_IN_TI2);
    timer_ic_set_filter(MEASURE_TIMER, TIM_IC2, TIM_IC_OFF);
    timer_ic_set_prescaler(MEASURE_TIMER, TIM_IC2, TIM_IC_PSC_OFF);
    timer_ic_set_polarity(MEASURE_TIMER, TIM_IC2, TIM_IC_BOTH);
    timer_ic_enable(MEASURE_TIMER, TIM_IC2);
    
    // Включаем прерывания
    timer_enable_irq(MEASURE_TIMER, TIM_DIER_CC2IE);
    nvic_enable_irq(NVIC_TIM2_IRQ);
    nvic_set_priority(NVIC_TIM2_IRQ, 0);
    
    timer_enable_counter(MEASURE_TIMER);
}

// Обработчик прерывания TIM2
void tim2_isr(void) {
    if (timer_get_flag(MEASURE_TIMER, TIM_SR_CC2IF)) {
        timer_clear_flag(MEASURE_TIMER, TIM_SR_CC2IF);
        
        const uint32_t capture_value = TIM_CCR2(MEASURE_TIMER);
        
        if (gpio_get(ECHO_PORT, ECHO_PIN)) {
            // Передний фронт - начало импульса
            g_measurement.pulse_start = capture_value;
        } else {
            // Задний фронт - конец импульса
            g_measurement.pulse_end = capture_value;
            
            // Вычисляем ширину импульса с учетом переполнения
            if (g_measurement.pulse_end >= g_measurement.pulse_start) 
            {
                g_measurement.pulse_width = g_measurement.pulse_end - g_measurement.pulse_start;
            }
            else {
                g_measurement.pulse_width = (0xFFFFFFFF - g_measurement.pulse_start) + 
                                          g_measurement.pulse_end + 1;
            }
            // true в прерывании таймера, когда получен задний фронт ECHO-импульса:
            g_measurement.measurement_ready = true;
            g_measurement.measurement_count++;
        }
    }
}

// Функция измерения расстояния с таймаутом
float measure_distance(void) {

    constexpr uint32_t MEASUREMENT_TIMEOUT_MS = 100; // Таймаут 100 мс
    
    // Сбрасываем состояние измерения
    g_measurement.pulse_start = 0;
    g_measurement.pulse_end = 0;
    g_measurement.pulse_width = 0;
    g_measurement.measurement_ready = false;
    
    // Отправка запускающего импульса на HC-SR04
    constexpr uint32_t TRIG_LOW_DELAY_US = 2;   // Ожидание перед импульсом
    constexpr uint32_t TRIG_HIGH_DELAY_US = 10; // Длительность триггерного импульса

    // Низкий уровень перед импульсом
    gpio_clear(TRIG_PORT, TRIG_PIN);
    delay_us(TRIG_LOW_DELAY_US);

    // Триггерный импульс 10-15 мкс
    gpio_set(TRIG_PORT, TRIG_PIN);
    delay_us(TRIG_HIGH_DELAY_US);

    // Возвращаем низкий уровень
    gpio_clear(TRIG_PORT, TRIG_PIN);

    // Ожидаем результат измерения
    constexpr uint32_t TIMEOUT_LOOPS = MEASUREMENT_TIMEOUT_MS * 100;

    for (uint32_t timeout = 0; timeout < TIMEOUT_LOOPS; timeout++) // Цикл ожидания
    {
        if (g_measurement.measurement_ready) // Ожидаем true в прерывании таймера
        {
            float distance = get_distance_cm();
            
        // Проверяем диапазон 2-400 см
        if (distance > 2.0f && distance < 400.0f)
        {
            return distance; // Возвращаем валидное расстояние
        }
        else
        {
            // Вывод сообщения об ошибке в UART
            if (distance < 2.0f)
            {
                uart_send_string("ОШИБКА: Cлишком маленькое расстояние (< 2 см)\r\n");
            }
            else if (distance > 400.0f)
            {
                uart_send_string("ОШИБКА: Cлишком большое расстояние (> 400 см)\r\n");
            }
            else
            {
                uart_send_string("ОШИБКА: Неверное измерение расстояния\r\n");
            }
            //return -1.0f; // Ошибка
        }
    }
    
    // Таймаут срабатывает если за 100 мс не получен ECHO-импульс
    return -1.0f;
    }
}

// Вычисление расстояния в сантиметрах
float get_distance_cm(void) 
{
    constexpr float SOUND_SPEED_CM_PER_US = 0.034f; // Скорость звука в см/мкс
    return (float(g_measurement.pulse_width) * SOUND_SPEED_CM_PER_US) / 2.0f; // Расстояние до объекта в см/мск
}

// Инициализация светодиода
void led_setup(void) {
    // Включаем тактирование порта D
    rcc_periph_clock_enable(RCC_GPIOD); 
    // Настраиваем пин как выход        
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN); 
    // Скорость переключения
    gpio_set_output_options(LED_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, LED_PIN);
    // Гарантируем выключенное состояние 
    gpio_clear(LED_PORT, LED_PIN); 
}

// Индикация состояния измерения светодиодом
void indicate_measurement(float distance) {
    if (distance < 2.0f  || distance > 400.0f) {   // При ошибочных измерениях
        gpio_set(LED_PORT, LED_PIN);
        delay_ms(60);
        gpio_clear(LED_PORT, LED_PIN);
    } 
    else if (distance > 2.0f && distance < 400.0f) { // При правильных измерениях
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
