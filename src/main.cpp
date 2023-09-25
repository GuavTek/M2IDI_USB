#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    gpio_init(LEDH);
    gpio_init(LEDD);
    gpio_set_dir(LEDH, GPIO_OUT);
    gpio_set_dir(LEDD, GPIO_OUT);
    while (true) {
        gpio_put(LEDH, 1);
        gpio_put(LEDD, 0);
        sleep_ms(250);
        gpio_put(LEDH, 0);
        gpio_put(LEDD, 1);
        sleep_ms(250);
    }
}