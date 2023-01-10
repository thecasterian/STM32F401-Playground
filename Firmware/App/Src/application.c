#include <stdio.h>
#include <string.h>
#include "ahrs.h"
#include "application.h"
#include "math.h"
#include "timer_wrapper.h"
#include "quaternion.h"
#include "usart.h"

#define PERIOD 0.002f

static mpu9250_t mpu9250;
static ahrs_t ahrs;

int _write(int file, char *ptr, int len);

static ahrs_status_t read_acc(void *aux, float *data);
static ahrs_status_t read_gyro(void *aux, float *data);
static ahrs_status_t read_mag(void *aux, float *data);

void setup(void) {
    timer_start();

    mpu9250_init(&mpu9250, &hspi2, MPU9250_NSS_GPIO_Port, MPU9250_NSS_Pin);
    ahrs_init(&ahrs, PERIOD, read_acc, &mpu9250, read_gyro, &mpu9250, read_mag, &mpu9250);

    ahrs_init_attitude(&ahrs, 1000U);
}

void loop(void) {
    float roll, pitch, yaw;
    uint8_t buf[13];

    if (timer.period_elapsed) {
        ahrs_update(&ahrs);

        quaternion_to_euler(&ahrs.q, &roll, &pitch, &yaw);
        buf[0] = 0x42;
        memcpy(&buf[1], &roll, sizeof(roll));
        memcpy(&buf[5], &pitch, sizeof(pitch));
        memcpy(&buf[9], &yaw, sizeof(yaw));
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
