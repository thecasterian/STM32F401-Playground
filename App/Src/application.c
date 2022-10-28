#include <stdio.h>
#include "application.h"
#include "usart.h"

w25q128jv_t w25q128jv;
mpu9250_t mpu9250;

void setup(void) {
    w25q128jv_init(&w25q128jv, &hspi1, W25Q128JV_NSS_GPIO_Port, W25Q128JV_NSS_Pin);
    mpu9250_init(&mpu9250, &hspi2, MPU9250_NSS_GPIO_Port, MPU9250_NSS_Pin);
}

void loop(void) {
    float acc[3], gyro[3];

    mpu9250_read_sensor(&mpu9250, acc, gyro, NULL);
    _write(0, acc, 12);
    _write(0, gyro, 12);
}

int _write(int file, char *ptr, int len) {
    UNUSED(file);

    if (HAL_UART_Transmit(&huart6, (uint8_t *)ptr, len, HAL_MAX_DELAY) != HAL_OK)
        return -1;

    return len;
}
