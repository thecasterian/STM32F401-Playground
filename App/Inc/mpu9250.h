#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>
#include "spi.h"

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *nss_port;
    uint16_t nss_pin;

    uint8_t who_am_i;

    float acc_resol;
    float gyro_resol;
} mpu9250_t;

typedef enum {
    MPU9250_OK = 0,
    MPU9250_ERR = -1,
} mpu9250_status_t;

/* Full scale range of the accelerometer. */
typedef enum {
    MPU9250_ACC_RANGE_2G  = 0,                  /* +/- 2g. */
    MPU9250_ACC_RANGE_4G  = 1,                  /* +/- 4g. */
    MPU9250_ACC_RANGE_8G  = 2,                  /* +/- 8g. */
    MPU9250_ACC_RANGE_16G = 3,                  /* +/- 16g. */
} mpu9250_acc_range_t;

/* Full scale range of the gyroscope. */
typedef enum {
    MPU9250_GYRO_RANGE_250DPS  = 0,             /* +/- 250 deg/s. */
    MPU9250_GYRO_RANGE_500DPS  = 1,             /* +/- 500 deg/s. */
    MPU9250_GYRO_RANGE_1000DPS = 2,             /* +/- 1000 deg/s. */
    MPU9250_GYRO_RANGE_2000DPS = 3,             /* +/- 2000 deg/s. */
} mpu9250_gyro_range_t;

mpu9250_status_t mpu9250_init(mpu9250_t *m, SPI_HandleTypeDef *hspi, GPIO_TypeDef *nss_port, uint16_t nss_pin);
mpu9250_status_t mpu9250_set_range(mpu9250_t *m, mpu9250_acc_range_t acc_range, mpu9250_gyro_range_t gyro_range);

/* Unit: m/s^2. */
mpu9250_status_t mpu9250_read_acc(mpu9250_t *m, float *acc);
/* Unit: rad/s. */
mpu9250_status_t mpu9250_read_gyro(mpu9250_t *m, float *gyro);
mpu9250_status_t mpu9250_read_mag(mpu9250_t *m, float *mag);

#endif
