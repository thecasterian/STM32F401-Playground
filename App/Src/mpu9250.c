#include "mpu9250.h"
#include "util.h"

#define ADC_MAX    32768.0f
#define G          9.80665f
#define DEG_TO_RAD 0.0174533f

#define FLAG_READ_REG (1 << 7)

#define REG_WHO_AM_I     0x75
#define REG_ACCEL_CONFIG 0x1C
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_ACCEL_XOUT_L 0x3C
#define REG_ACCEL_YOUT_H 0x3D
#define REG_ACCEL_YOUT_L 0x3E
#define REG_ACCEL_ZOUT_H 0x3F
#define REG_ACCEL_ZOUT_L 0x40
#define REG_GYRO_XOUT_H  0x43
#define REG_GYRO_XOUT_L  0x44
#define REG_GYRO_YOUT_H  0x45
#define REG_GYRO_YOUT_L  0x46
#define REG_GYRO_ZOUT_H  0x47
#define REG_GYRO_ZOUT_L  0x48

/* The value of "Who am I" depends on the variants. */
#define WHO_AM_I_MPU9250 0x71
#define WHO_AM_I_MPU9255 0x73

#define ACCEL_FS_SEL_POS 3
#define GYRO_FS_SEL_POS  3

static mpu9250_status_t mpu9250_transmit(mpu9250_t *m, uint8_t *buf, uint32_t len);
static mpu9250_status_t mpu9250_receive(mpu9250_t *m, uint8_t *buf, uint32_t len);
static mpu9250_status_t mpu9250_transmit_receive(mpu9250_t *m, uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len);

static mpu9250_status_t mpu9250_read_byte(mpu9250_t *m, uint8_t reg, uint8_t *data);
static mpu9250_status_t mpu9250_write_byte(mpu9250_t *m, uint8_t reg, uint8_t data);

static void nss_low(mpu9250_t *m);
static void nss_high(mpu9250_t *m);

mpu9250_status_t mpu9250_init(mpu9250_t *m, SPI_HandleTypeDef *hspi, GPIO_TypeDef *nss_port, uint16_t nss_pin) {
    mpu9250_status_t res;

    m->hspi = hspi;
    m->nss_port = nss_port;
    m->nss_pin = nss_pin;

    res = mpu9250_read_byte(m, REG_WHO_AM_I, &m->who_am_i);
    if (res == MPU9250_OK) {
        switch (m->who_am_i) {
        case WHO_AM_I_MPU9250:
        case WHO_AM_I_MPU9255:
            break;
        default:
            DBG("mpu9250: unknown who am i: 0x%02X", m->who_am_i);
            res = MPU9250_ERR;
        }
    }

    m->acc_resol = 2.0f / ADC_MAX * G;
    m->gyro_resol = 250.0f / ADC_MAX * DEG_TO_RAD;

    return res;
}

mpu9250_status_t mpu9250_set_range(mpu9250_t *m, mpu9250_acc_range_t acc_range, mpu9250_gyro_range_t gyro_range) {
    mpu9250_status_t res = MPU9250_OK;

    switch (acc_range) {
    case MPU9250_ACC_RANGE_2G:
        m->acc_resol = 2.0f / ADC_MAX * G;
        break;
    case MPU9250_ACC_RANGE_4G:
        m->acc_resol = 4.0f / ADC_MAX * G;
        break;
    case MPU9250_ACC_RANGE_8G:
        m->acc_resol = 8.0f / ADC_MAX * G;
        break;
    case MPU9250_ACC_RANGE_16G:
        m->acc_resol = 16.0f / ADC_MAX * G;
        break;
    default:
        res = MPU9250_ERR;
    }

    if (mpu9250_write_byte(m, REG_ACCEL_CONFIG, (uint8_t)acc_range << ACCEL_FS_SEL_POS) != MPU9250_OK)
        res = MPU9250_ERR;

    switch (gyro_range) {
    case MPU9250_GYRO_RANGE_250DPS:
        m->gyro_resol = 250.0f / ADC_MAX * DEG_TO_RAD;
        break;
    case MPU9250_GYRO_RANGE_500DPS:
        m->gyro_resol = 500.0f / ADC_MAX * DEG_TO_RAD;
        break;
    case MPU9250_GYRO_RANGE_1000DPS:
        m->gyro_resol = 1000.0f / ADC_MAX * DEG_TO_RAD;
        break;
    case MPU9250_GYRO_RANGE_2000DPS:
        m->gyro_resol = 2000.0f / ADC_MAX * DEG_TO_RAD;
        break;
    default:
        res = MPU9250_ERR;
    }

    if (mpu9250_write_byte(m, REG_GYRO_CONFIG, (uint8_t)gyro_range << GYRO_FS_SEL_POS) != MPU9250_OK)
        res = MPU9250_ERR;

    return res;
}

mpu9250_status_t mpu9250_read_sensor(mpu9250_t *m, float *acc, float *gyro, float *mag) {
    uint8_t axh, axl, ayh, ayl, azh, azl;
    uint8_t gxh, gxl, gyh, gyl, gzh, gzl;
    uint16_t ax, ay, az, gx, gy, gz;
    mpu9250_status_t res = MPU9250_ERR;

    if (mpu9250_read_byte(m, REG_ACCEL_XOUT_H, &axh) == MPU9250_OK &&
        mpu9250_read_byte(m, REG_ACCEL_XOUT_L, &axl) == MPU9250_OK &&
        mpu9250_read_byte(m, REG_ACCEL_YOUT_H, &ayh) == MPU9250_OK &&
        mpu9250_read_byte(m, REG_ACCEL_YOUT_L, &ayl) == MPU9250_OK &&
        mpu9250_read_byte(m, REG_ACCEL_ZOUT_H, &azh) == MPU9250_OK &&
        mpu9250_read_byte(m, REG_ACCEL_ZOUT_L, &azl) == MPU9250_OK &&
        mpu9250_read_byte(m, REG_GYRO_XOUT_H, &gxh) == MPU9250_OK &&
        mpu9250_read_byte(m, REG_GYRO_XOUT_L, &gxl) == MPU9250_OK &&
        mpu9250_read_byte(m, REG_GYRO_YOUT_H, &gyh) == MPU9250_OK &&
        mpu9250_read_byte(m, REG_GYRO_YOUT_L, &gyl) == MPU9250_OK &&
        mpu9250_read_byte(m, REG_GYRO_ZOUT_H, &gzh) == MPU9250_OK &&
        mpu9250_read_byte(m, REG_GYRO_ZOUT_L, &gzl) == MPU9250_OK) {
        ax = PACK_2(axh, axl);
        ay = PACK_2(ayh, ayl);
        az = PACK_2(azh, azl);
        gx = PACK_2(gxh, gxl);
        gy = PACK_2(gyh, gyl);
        gz = PACK_2(gzh, gzl);

        acc[0] = *(int16_t *)&ax * m->acc_resol;
        acc[1] = *(int16_t *)&ay * m->acc_resol;
        acc[2] = *(int16_t *)&az * m->acc_resol;

        gyro[0] = *(int16_t *)&gx * m->gyro_resol;
        gyro[1] = *(int16_t *)&gy * m->gyro_resol;
        gyro[2] = *(int16_t *)&gz * m->gyro_resol;

        // TODO: get mag.
        UNUSED(mag);

        res = MPU9250_OK;
    }

    return res;
}

static mpu9250_status_t mpu9250_transmit(mpu9250_t *m, uint8_t *buf, uint32_t len) {
    return HAL_SPI_Transmit(m->hspi, buf, len, HAL_MAX_DELAY) == HAL_OK ? MPU9250_OK : MPU9250_ERR;
}

static mpu9250_status_t mpu9250_receive(mpu9250_t *m, uint8_t *buf, uint32_t len) {
    return HAL_SPI_Receive(m->hspi, buf, len, HAL_MAX_DELAY) == HAL_OK ? MPU9250_OK : MPU9250_ERR;
}

static mpu9250_status_t mpu9250_transmit_receive(mpu9250_t *m, uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len) {
    return HAL_SPI_TransmitReceive(m->hspi, tx_buf, rx_buf, len, HAL_MAX_DELAY) == HAL_OK ? MPU9250_OK : MPU9250_ERR;
}

static mpu9250_status_t mpu9250_read_byte(mpu9250_t *m, uint8_t reg, uint8_t *data) {
    uint8_t tx_buf[2] = {reg | FLAG_READ_REG};
    uint8_t rx_buf[2];
    mpu9250_status_t res;

    nss_low(m);
    res = mpu9250_transmit_receive(m, tx_buf, rx_buf, sizeof(tx_buf));
    nss_high(m);

    if (res == MPU9250_OK)
        *data = rx_buf[1];

    return res;
}

static mpu9250_status_t mpu9250_write_byte(mpu9250_t *m, uint8_t reg, uint8_t data) {
    uint8_t tx_buf[2] = {reg, data};
    mpu9250_status_t res;

    nss_low(m);
    res = mpu9250_transmit(m, tx_buf, sizeof(tx_buf));
    nss_high(m);

    return res;
}

static void nss_low(mpu9250_t *m) {
    HAL_GPIO_WritePin(m->nss_port, m->nss_pin, GPIO_PIN_RESET);
}

static void nss_high(mpu9250_t *m) {
    HAL_GPIO_WritePin(m->nss_port, m->nss_pin, GPIO_PIN_SET);
}
