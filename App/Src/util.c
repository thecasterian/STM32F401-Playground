#include <string.h>
#include "main.h"
#include "util.h"

void delay_us(uint32_t us) {
    volatile uint32_t x = us * (SystemCoreClock / 1000000UL);

    while (x--) {}
}

int16_t to_signed_16(uint16_t x) {
    int16_t res;

    (void)memcpy(&res, &x, sizeof(res));

    return res;
}
