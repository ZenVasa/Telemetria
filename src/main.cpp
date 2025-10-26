#include "../inc/setup.hpp"

// Таймер
constexpr uint32_t MEASURE_TIMER = TIM2;

// Пины и периферия
constexpr uint32_t TRIG_PORT = GPIOA;
constexpr uint16_t TRIG_PIN = GPIO5;        // PA5 - TIM2_CH1 (вместо PA0)
constexpr uint32_t ECHO_PORT = GPIOA;  
constexpr uint16_t ECHO_PIN = GPIO3;        // PA3 - TIM2_CH4 (вместо PA1)

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

// ============== MAIN =======================

int main(void) {
    // Настройка системных часов на 84 МГц от HSE 8 МГц
    clock_setup();
    
    // Инициализация периферии

    //uart_setup<USART1>(GPIOA, GPIO9, GPIO10, GPIO_AF7);  // USART1 на PA9 (TX), PA10 (RX)
    uart_setup<USART3>(GPIOD, GPIO8, GPIO9, GPIO_AF7);  // USART1 на PA9 (TX), PA10 (RX)

    gpio_setup();
    timer_setup();
    led_setup();
    
    // Переменная для хранения расстояния
    uint16_t distance_cm = 0;
    
    // Небольшая задержка для стабилизации системы
    delay_ms(100);
    
    uart_send_string("HC-SR04 Начато измерение расстояния\r\n");
    

    // Основной цикл программы
    while (true) {
        
         // Выполняем измерение расстояния
        distance_cm = measure_distance();
        
        // Индикация результата на светодиодах
        //indicate_measurement(distance_cm);
        
        // Отправляем расстояние по UART
        //send_distance_cm(distance_cm);
        uart_send_string("См");
        my_usart_print_int(USART3, distance_cm);

        // Задержка между измерениями
        delay_ms(100);

    }
    
    return 0;
}

//---------------------------------------------------------------------------------

// Отправка строки по UART
void uart_send_string(const char *str) {
    while (*str) {
        usart_send_blocking(USART3, *str);
        str++;
    }
}

// Отправка числа int по UART
void send_distance_cm(int value) {
    char buffer[16];
    
    // Преобразуем число в строку
    snprintf(buffer, sizeof(buffer), "%d\r\n", value);
    
    // Отправляем строку
    for (char *p = buffer; *p; p++) {
        usart_send_blocking(USART3, *p);
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

//---------------------------------------------------------------------------------


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

// Функция измерения расстояния
uint16_t measure_distance(void) {

    constexpr uint32_t MEASUREMENT_TIMEOUT_MS = 100; // Таймаут 100 мс
    
    // Сбрасываем состояние измерения
    g_measurement.pulse_start = 0;
    g_measurement.pulse_end = 0;
    g_measurement.pulse_width = 0;
    g_measurement.measurement_ready = false;
    
    // Отправка запускающего импульса на HC-SR04
    constexpr uint32_t TRIG_LOW_DELAY_US = 4;   // Ожидание перед импульсом
    constexpr uint32_t TRIG_HIGH_DELAY_US = 12; // Длительность триггерного импульса

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
            uint16_t distance = get_distance_cm();

            // Проверяем диапазон 2-400 см
            if (distance >= 2 && distance <= 400)
            {
                return distance; // Возвращаем валидное расстояние
            }
        }
    }
    // Таймаут срабатывает если за 100 мс не получен ECHO-импульс
    return 0;

}

// Вычисление расстояния в сантиметрах
uint16_t get_distance_cm(void) 
{
    return (uint16_t)((g_measurement.pulse_width * 17) / 1000); // Расстояние до объекта в см/мск
}
