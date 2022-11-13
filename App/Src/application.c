#include <stdio.h>
#include <string.h>
#include "ahrs.h"
#include "application.h"
#include "math.h"
#include "timer_wrapper.h"
#include "quaternion.h"
#include "usart.h"

#define PERIOD 0.001f

static w25q128jv_t w25q128jv;
static mpu9250_t mpu9250;
static ahrs_t ahrs;

int _write(int file, char *ptr, int len);

static ahrs_status_t read_acc(void *aux, float *data);
static ahrs_status_t read_gyro(void *aux, float *data);
static ahrs_status_t read_mag(void *aux, float *data);

void setup(void) {
    timer_start(&timer);

    w25q128jv_init(&w25q128jv, &hspi1, W25Q128JV_NSS_GPIO_Port, W25Q128JV_NSS_Pin);
    mpu9250_init(&mpu9250, &hspi2, MPU9250_NSS_GPIO_Port, MPU9250_NSS_Pin);
    ahrs_init(&ahrs, PERIOD, read_acc, &mpu9250, read_gyro, &mpu9250, read_mag, &mpu9250);

    ahrs_init_attitude(&ahrs, 1000U);
}

void loop(void) {
    float rpy_acc_mag[3], rpy_gyro[3];
    uint8_t buf[25];

    if (timer.period_elapsed) {
        ahrs_update(&ahrs);

        quaternion_to_euler(&ahrs.q_acc_mag, &rpy_acc_mag[0], &rpy_acc_mag[1], &rpy_acc_mag[2]);
        quaternion_to_euler(&ahrs.q_gyro, &rpy_gyro[0], &rpy_gyro[1], &rpy_gyro[2]);

        buf[0] = 0x42;
        memcpy(&buf[1], rpy_acc_mag, sizeof(rpy_acc_mag));
        memcpy(&buf[13], rpy_gyro, sizeof(rpy_gyro));
        _write(0, (char *)buf, sizeof(buf));

        timer.period_elapsed = false;
    }
}

int _write(int file, char *ptr, int len) {
    UNUSED(file);

    if (HAL_UART_Transmit(&huart6, (uint8_t *)ptr, len, HAL_MAX_DELAY) != HAL_OK)
        return -1;

    return len;
}

static ahrs_status_t read_acc(void *aux, float *data) {
    return mpu9250_read_acc(aux, data) == MPU9250_OK ? AHRS_OK : AHRS_ERR;
}

static ahrs_status_t read_gyro(void *aux, float *data) {
    return mpu9250_read_gyro(aux, data) == MPU9250_OK ? AHRS_OK : AHRS_ERR;
}

static ahrs_status_t read_mag(void *aux, float *data) {
    return mpu9250_read_mag(aux, data) == MPU9250_OK ? AHRS_OK : AHRS_ERR;
}
