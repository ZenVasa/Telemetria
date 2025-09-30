#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>


// Определения пинов и периферии
#define TRIG_PORT GPIOA
#define TRIG_PIN  GPIO0    // PA0 - TIM2_CH1 
#define ECHO_PORT GPIOA  
#define ECHO_PIN  GPIO1    // PA1 - TIM2_CH2 

// UART3 пины (PD8 - TX, PD9 - RX)
#define UART_PORT GPIOD
#define UART_TX_PIN GPIO8
#define UART_RX_PIN GPIO9
#define UART_DEVICE USART3

#define MEASURE_TIMER TIM2

// Глобальные переменные для измерения
volatile uint32_t pulse_start = 0;        // Время начала импульса ECHO
volatile uint32_t pulse_end = 0;          // Время окончания импульса ECHO  
volatile uint32_t pulse_width = 0;        // Длительность импульса (тики)
volatile bool measurement_ready = false;  // Флаг готовности измерения
volatile bool measurement_timeout = false;// Флаг таймаута измерения
volatile uint32_t measurement_count = 0;  // Счетчик измерений

// Прототипы функций
void uart_setup(void);
void gpio_setup(void);
void timer_setup(void);
void tim2_isr(void);
void led_setup(void);
void uart_send_string(const char *str);
void send_distance_cm(float distance_cm);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void send_trigger_pulse(void);
float get_distance_cm(void);
bool is_valid_distance(float distance_cm);
float measure_distance(void);

void indicate_measurement(float distance)



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
    uint32_t measurement_counter = 0;
    
    // Небольшая задержка для стабилизации системы
    delay_ms(100);
    
    // Основной цикл программы
    while (true) {
        // Выполняем измерение расстояния
        distance_cm = measure_distance();
        measurement_counter++;
        
        // Индикация результата (опционально)
        indicate_measurement(distance_cm);
        
         // Отправляем расстояние по UART
        send_distance_cm(distance_cm);


        // Задержка между измерениями (рекомендуется >60 мс)
        delay_ms(60);
    }
    
    return 0;
}

void uart_setup(void) {
    // Включаем тактирование порта D и USART3
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_USART3);
    
    // Настраиваем PD8 как альтернативную функцию (USART3_TX)
    gpio_mode_setup(UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, UART_TX_PIN);
    gpio_set_af(UART_PORT, GPIO_AF7, UART_TX_PIN);
    
    // Настраиваем PD9 как альтернативную функцию (USART3_RX)
    gpio_mode_setup(UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, UART_RX_PIN);
    gpio_set_af(UART_PORT, GPIO_AF7, UART_RX_PIN);
    
    USART_BRR(USART3) = 42000000 / 115200;
    /*
    // Настраиваем USART3
    usart_set_baudrate(UART_DEVICE, 115200);
    usart_set_databits(UART_DEVICE, 8);
    usart_set_stopbits(UART_DEVICE, USART_STOPBITS_1);
    usart_set_parity(UART_DEVICE, USART_PARITY_NONE);
    usart_set_flow_control(UART_DEVICE, USART_FLOWCONTROL_NONE);
    usart_set_mode(UART_DEVICE, USART_MODE_TX_RX);
    */

    // Включаем USART3
    usart_enable(UART_DEVICE);
}

// Отправка строки по UART
void uart_send_string(const char *str) {
    while (*str) {
        usart_send_blocking(UART_DEVICE, *str);
        str++;
    }
}
// Буфер для форматирования строки UART
char uart_buffer[64];
// Отправка расстояния по UART
void send_distance_cm(float distance_cm) {
    if (distance_cm > 0) {
        snprintf(uart_buffer, sizeof(uart_buffer), "%.2f cm\r\n", distance_cm);
    } else {
        snprintf(uart_buffer, sizeof(uart_buffer), "Error\r\n");
    }
    
    uart_send_string(uart_buffer);
}

    // Инициализация GPIO
    // PA0 (TRIG) - выход для запуска измерения
    // PA1 (ECHO) - вход с альтернативной функцией для захвата таймером
void gpio_setup(void) {
    // Включаем тактирование порта A
    rcc_periph_clock_enable(RCC_GPIOA);
    
    // Настраиваем PA0 (TRIG) как выход push-pull
    gpio_mode_setup(TRIG_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TRIG_PIN);
    gpio_set_output_options(TRIG_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, TRIG_PIN);
    
    // Настраиваем PA1 (ECHO) как вход с альтернативной функцией (TIM2_CH2)
    gpio_mode_setup(ECHO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, ECHO_PIN);
    gpio_set_af(ECHO_PORT, GPIO_AF1, ECHO_PIN); // AF1 для TIM2
    
    // Изначально устанавливаем TRIG в низкий уровень
    gpio_clear(TRIG_PORT, TRIG_PIN);
}

    // Инициализация таймера TIM2 для Capture измерений
    // Таймер настроен на частоту 1 МГц (1 тик = 1 микросекунда)
    // Канал 2 настроен на захват по обоим фронтам сигнала ECHO

void timer_setup(void) {
    
    rcc_periph_clock_enable(RCC_TIM2);      // Включаем тактирование TIM2
    timer_disable_counter(MEASURE_TIMER);   // Останавливаем таймер перед настройкой
    timer_reset(MEASURE_TIMER); // Сбрасываем настройки таймера
    
    timer_set_prescaler(MEASURE_TIMER, 84 - 1); // Настройка предделителя: 84 МГц / 84 = 1 МГц (1 мкс на тик)
    timer_set_period(MEASURE_TIMER, 0xFFFFFFFF); // Устанавливаем максимальный период (32 бита)
    
    timer_continuous_mode(MEASURE_TIMER);   // Режим работы: непрерывный счет
    
    // Настройка канала 2 (PA1) для Input Capture
    timer_ic_set_input(MEASURE_TIMER, TIM_IC2, TIM_IC_IN_TI1);
    timer_ic_set_filter(MEASURE_TIMER, TIM_IC2, TIM_IC_OFF);
    timer_ic_set_prescaler(MEASURE_TIMER, TIM_IC2, TIM_IC_PSC_OFF);
    
    // Захват по обоим фронтам (передний и задний)
    timer_ic_set_polarity(MEASURE_TIMER, TIM_IC2, TIM_IC_BOTH_EDGES);
    
    timer_cc_enable(MEASURE_TIMER, TIM_CC2);  // Включаем канал capture/compare
    timer_enable_irq(MEASURE_TIMER, TIM_DIER_CC2IE);   // Включаем прерывание по захвату канала 2
    
    nvic_enable_irq(NVIC_TIM2_IRQ);         // Разрешаем прерывание TIM2 в NVIC (приоритет 0)
    nvic_set_priority(NVIC_TIM2_IRQ, 0);
    
    timer_enable_counter(MEASURE_TIMER);    // Запускаем таймер
}


 //   Обработчик прерывания TIM2 - Capture прерывание
  
 //   Вызывается при срабатывании захвата по каналу 2 (ECHO сигнал)
 //   Определяет фронты импульса и вычисляет его длительность

void tim2_isr(void) {
    // Проверяем флаг прерывания Capture/Compare канала 2
    if (timer_get_flag(MEASURE_TIMER, TIM_SR_CC2IF)) {
        // Сбрасываем флаг прерывания
        timer_clear_flag(MEASURE_TIMER, TIM_SR_CC2IF);
        
        // Читаем значение регистра захвата канала 2
        uint32_t capture_value = timer_get_capture_value(MEASURE_TIMER, TIM_IC2);
        
        // Определяем тип фронта по состоянию пина ECHO
        if (gpio_get(ECHO_PORT, ECHO_PIN)) {
            // CAPTURE по ПЕРЕДНЕМУ фронту - начало импульса ECHO
            pulse_start = capture_value;
        } else {
            // CAPTURE по ЗАДНЕМУ фронту - конец импульса ECHO
            pulse_end = capture_value;
            
            // Вычисляем ширину импульса с учетом возможного переполнения таймера
            if (pulse_end >= pulse_start) {
                pulse_width = pulse_end - pulse_start;
            } else {
                // Обработка переполнения таймера (32-битный счетчик)
                pulse_width = (0xFFFFFFFF - pulse_start) + pulse_end + 1;
            }
            
            // Устанавливаем флаг готовности измерения
            measurement_ready = true;
            measurement_count++;
        }
    }
}



 // Вычисление расстояния в сантиметрах

//   Формула: расстояние = (время_полета * скорость_звука) / 2
//  Скорость звука: 340 м/с = 0.034 см/мкс
//  Делим на 2 потому что звук проходит путь до объекта и обратно

float get_distance_cm(void) {
    // pulse_width в микросекундах (т.к. таймер на 1 МГц)
    return (pulse_width * 0.034) / 2.0;
}

// Вычисление расстояния в миллиметрах 

uint32_t get_distance_mm(void) {
    // Более точный расчет в мм
    // Скорость звука: 340000 мм/с = 0.34 мм/мкс
    return (pulse_width * 0.34) / 2.0;
}

// Проверка валидности измерения
 
    // Расстояние в см для проверки (true false) 
bool is_valid_distance(float distance_cm) {
    // HC-SR04 рабочий диапазон: 2-400 см
    // Отсекаем значения вне диапазона
    return (distance_cm >= 2.0f && distance_cm <= 400.0f);
}


    // Функция измерения расстояния с таймаутом
    // Расстояние в см, или -1.0 при ошибке/таймауте

float measure_distance(void) {
    // Сбрасываем флаги перед измерением
    measurement_ready = false;
    measurement_timeout = false;
    
    // Сбрасываем значения захвата
    pulse_start = 0;
    pulse_end = 0;
    pulse_width = 0;
    
    // Отправляем запускающий импульс
    send_trigger_pulse();
    
    // Ожидаем результат измерения (таймаут 100 мс = ~34 метра)
    for (volatile uint32_t timeout = 0; timeout < 100000; timeout++) {
        if (measurement_ready) {
            float distance = get_distance_cm();
            
            // Проверяем валидность измерения
            if (is_valid_distance(distance)) {
                return distance;
            } else {
                return -1.0f; // Невалидное расстояние
            }
        }
        
        // Короткая задержка для цикла ожидания
        delay_us(10);
    }
    
    // Таймаут - объект слишком далеко или ошибка измерения
    return -1.0f;
}

// Отправка запускающего импульса на HC-SR04
  //  Генерирует импульс длительностью 10 микросекунд на пине TRIG

void send_trigger_pulse(void) {
    // Гарантируем начальный низкий уровень (минимум 2 мкс)
    gpio_clear(TRIG_PORT, TRIG_PIN);
    delay_us(5);
    
    // Устанавливаем высокий уровень на 10-15 мкс
    gpio_set(TRIG_PORT, TRIG_PIN);
    delay_us(12); // 12 мкс
    
    // Возвращаем низкий уровень
    gpio_clear(TRIG_PORT, TRIG_PIN);
}


// Инициализация светодиода для индикации

void led_setup(void) {
    // Включаем тактирование порта D
    rcc_periph_clock_enable(RCC_GPIOD);
    
    // Настраиваем PD15
    gpio_mode_setup(GPIOD,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO13);
    gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO13);
    
    // Изначально выключаем светодиод
    gpio_clear(GPIOD, GPIO13);
}


// Индикация состояния измерения светодиодом
void indicate_measurement(float distance) {
    // Мигаем светодиодом в зависимости от результата
    if (distance > 5) {

        gpio_set(GPIOD, GPIO13);
        delay_ms(50);
        gpio_clear(GPIOD, GPIO13);
    } 
    else if(distance < 5) {
        
        gpio_set(GPIOD, GPIO13);
        delay_ms(25);
        gpio_clear(GPIOD, GPIO13);
    }
    else{

    }
}


// Точная задержка в микросекундах
//    количество микросекунд для задержки

void delay_us(uint32_t us) {
    // Используем простой цикл для задержки
    // При 84 МГц: 84 цикла ≈ 1 мкс
    for (volatile uint32_t i = 0; i < us * 84; i++);
}

//  Точная задержка в миллисекундах
 //   количество миллисекунд для задержки

void delay_ms(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}



/*
int main()
{

    rcc_periph_clock_enable(RCC_GPIOD);//Группа портов вывода D
    gpio_mode_setup(GPIOD,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO15);

    while (true)
    {
        gpio_toggle(GPIOD, GPIO15);
        for ( volatile uint32_t i = 0; i< 2000000; i+=2 ); 
    }
}
*/