#include <stdio.h>
#include "application.h"
#include "usart.h"

w25q128jv_t w25q128jv;

void setup(void) {
    w25q128jv_init(&w25q128jv, &hspi1, W25Q128JV_NSS_GPIO_Port, W25Q128JV_NSS_Pin);
}

void loop(void) {

}

int _write(int file, char *ptr, int len) {
    UNUSED(file);

    if (HAL_UART_Transmit(&huart6, (uint8_t *)ptr, len, HAL_MAX_DELAY) != HAL_OK)
        return -1;

    return len;
}
