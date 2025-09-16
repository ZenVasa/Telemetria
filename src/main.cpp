#include <libopencm3/stm32/rcc.h>   // reset and clock control
#include <libopencm3/stm32/gpio.h>  // general purpose input-output


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
