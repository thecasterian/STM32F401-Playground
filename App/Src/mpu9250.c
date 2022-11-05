#include <math.h>
#include "mpu9250.h"
#include "timer_wrapper.h"
#include "util.h"

#define ADC_MAX    32768.f
#define G          9.80665f
#define DEG_TO_RAD 0.01745329252f
#define MAG_RESOL  (4912.f / 32760.f)

#define AK8963_I2C_ADDR 0x0C

#define FLAG_READ_REG (1 << 7)

/* MPU9250 registers. */
#define REG_SMPLRT_DIV       0x19
#define REG_CONFIG           0x1A
#define REG_GYRO_CONFIG      0x1B
#define REG_ACCEL_CONFIG     0x1C
#define REG_ACCEL_CONFIG2    0x1D
#define REG_I2C_MST_CTRL     0x24
#define REG_I2C_SLV0_ADDR    0x25
#define REG_I2C_SLV0_REG     0x26
#define REG_I2C_SLV0_CTRL    0x27
#define REG_ACCEL_XOUT_H     0x3B
#define REG_GYRO_XOUT_H      0x43
#define REG_EXT_SENS_DATA_00 0x49
#define REG_I2C_SLV0_DO      0x63
#define REG_USER_CTRL        0x6A
#define REG_PWR_MGMT_1       0x6B
#define REG_PWR_MGMT_2       0x6C
#define REG_WHO_AM_I         0x75

/* AK8963 registers. */
#define REG_WIA   0x00
#define REG_HXL   0x03
#define REG_HXH   0x04
#define REG_HYL   0x05
#define REG_HYH   0x06
#define REG_HZL   0x07
#define REG_HZH   0x08
#define REG_CNTL1 0x0A
#define REG_CNTL2 0x0B
#define REG_ASAX  0x10
#define REG_ASAY  0x11
#define REG_ASAZ  0x12

/* The value of who-am-i depends on the variants. */
#define WHO_AM_I_MPU9250 0x71
#define WHO_AM_I_MPU9255 0x73
#define WHO_AM_I_AK8963  0x48

#define CLKSEL_PLL       0x01
#define DLPF_184         0x01
#define I2C_MST_EN       0x20
#define I2C_MST_CLK_400K 0x0D
#define I2C_SLV0_EN      0x80
#define H_RESET          0x80

#define ACCEL_FS_SEL_POS 3
#define GYRO_FS_SEL_POS  3

#define MODE_POWER_DOWN 0x00
#define MODE_FUSE_ROM   0x0F
#define MODE_CONT_MEAS2 0x06
#define SRST            0x01
#define BIT_16          0x10
#define HOFL            0x08

static mpu9250_status_t mpu9250_transmit(mpu9250_t *m, uint8_t *buf, uint16_t len);
static mpu9250_status_t mpu9250_receive(mpu9250_t *m, uint8_t *buf, uint16_t len);
static mpu9250_status_t mpu9250_transmit_receive(mpu9250_t *m, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len);

static mpu9250_status_t mpu9250_read_byte(mpu9250_t *m, uint8_t reg, uint8_t *data);
static mpu9250_status_t mpu9250_read(mpu9250_t *m, uint8_t reg, uint8_t *data, uint8_t size);
static mpu9250_status_t mpu9250_write_byte(mpu9250_t *m, uint8_t reg, uint8_t data);

static mpu9250_status_t mpu9250_read_byte_ak8963(mpu9250_t *m, uint8_t reg, uint8_t *data);
static mpu9250_status_t mpu9250_read_ak8963(mpu9250_t *m, uint8_t reg, uint8_t *data, uint8_t size);
static mpu9250_status_t mpu9250_write_byte_ak8963(mpu9250_t *m, uint8_t reg, uint8_t data);

static void mpu9250_nss_low(mpu9250_t *m);
static void mpu9250_nss_high(mpu9250_t *m);

mpu9250_status_t mpu9250_init(mpu9250_t *m, SPI_HandleTypeDef *hspi, GPIO_TypeDef *nss_port, uint16_t nss_pin) {
    uint8_t buf[7];
    uint8_t asa[3];

    m->hspi = hspi;
    m->nss_port = nss_port;
    m->nss_pin = nss_pin;

    /* Select clock source to PLL. */
    if (mpu9250_write_byte(m, REG_PWR_MGMT_1, CLKSEL_PLL) != MPU9250_OK)
        return MPU9250_ERR;
    /* Enable I2C master mode and set its speed to 400 kHz. */
    if (mpu9250_write_byte(m, REG_USER_CTRL, I2C_MST_EN) != MPU9250_OK)
        return MPU9250_ERR;
    if (mpu9250_write_byte(m, REG_I2C_MST_CTRL, I2C_MST_CLK_400K) != MPU9250_OK)
        return MPU9250_ERR;

    /* Power down AK8963. */
    if (mpu9250_write_byte_ak8963(m, REG_CNTL1, MODE_POWER_DOWN) != MPU9250_OK)
        return MPU9250_ERR;
    HAL_Delay(100);
    /* Reset MPU9250. */
    if (mpu9250_write_byte(m, REG_PWR_MGMT_1, H_RESET) != MPU9250_OK)
        return MPU9250_ERR;
    HAL_Delay(100);
    /* Reset AK8963. */
    if (mpu9250_write_byte_ak8963(m, REG_CNTL2, SRST) != MPU9250_OK)
        return MPU9250_ERR;
    /* Select clock source to PLL. */
    if (mpu9250_write_byte(m, REG_PWR_MGMT_1, CLKSEL_PLL) != MPU9250_OK)
        return MPU9250_ERR;

    /* Validate who-am-i of MPU9250. */
    if (mpu9250_read_byte(m, REG_WHO_AM_I, &m->who_am_i) != MPU9250_OK)
        return MPU9250_ERR;
    if ((m->who_am_i != WHO_AM_I_MPU9250) && (m->who_am_i != WHO_AM_I_MPU9255)) {
        DBG("MPU9250: unknown who am i: 0x%02X", m->who_am_i);
        return MPU9250_ERR;
    }

    /* Enable accelerometer and gyro. */
    if (mpu9250_write_byte(m, REG_PWR_MGMT_2, 0x00) != MPU9250_OK)
        return MPU9250_ERR;
    /* Set accelerometer and gyro full scale range to their maximum value. */
    if (mpu9250_set_range(m, MPU9250_ACC_RANGE_16G, MPU9250_GYRO_RANGE_2000DPS) != MPU9250_OK)
        return MPU9250_ERR;
    /* Set bandwidth to 184 Hz. */
    if (mpu9250_write_byte(m, REG_CONFIG, DLPF_184) != MPU9250_OK)
        return MPU9250_ERR;
    if (mpu9250_write_byte(m, REG_ACCEL_CONFIG2, DLPF_184) != MPU9250_OK)
        return MPU9250_ERR;
    /* Set sample rate divider to 0 (no division). */
    if (mpu9250_write_byte(m, REG_SMPLRT_DIV, 0x00) != MPU9250_OK)
        return MPU9250_ERR;

    /* Enable I2C master mode and set its speed to 400 kHz. */
    if (mpu9250_write_byte(m, REG_USER_CTRL, I2C_MST_EN) != MPU9250_OK)
        return MPU9250_ERR;
    if (mpu9250_write_byte(m, REG_I2C_MST_CTRL, I2C_MST_CLK_400K) != MPU9250_OK)
        return MPU9250_ERR;

    /* Validate who-am-i of AK8963. */
    if (mpu9250_read_byte_ak8963(m, REG_WIA, &m->who_am_i_ak8963) != MPU9250_OK)
        return MPU9250_ERR;
    if (m->who_am_i_ak8963 != WHO_AM_I_AK8963) {
        DBG("AK8963: unknown who am i: 0x%02X", m->who_am_i_ak8963);
        return MPU9250_ERR;
    }

    /* Power down AK8963 and enter to FUSE ROM access mode. */
    if (mpu9250_write_byte_ak8963(m, REG_CNTL1, MODE_POWER_DOWN) != MPU9250_OK)
        return MPU9250_ERR;
    HAL_Delay(100);
    if (mpu9250_write_byte_ak8963(m, REG_CNTL1, MODE_FUSE_ROM) != MPU9250_OK)
        return MPU9250_ERR;
    HAL_Delay(100);

    /* Read sensitivity adjustment values from AK8963. */
    if (mpu9250_read_ak8963(m, REG_ASAX, asa, sizeof(asa)) != MPU9250_OK)
        return MPU9250_ERR;
    m->xadj = ((asa[0] - 128.f) / 256.f) + 1.f;
    m->yadj = ((asa[1] - 128.f) / 256.f) + 1.f;
    m->zadj = ((asa[2] - 128.f) / 256.f) + 1.f;

    /* Power down AK8963. */
    if (mpu9250_write_byte_ak8963(m, REG_CNTL1, MODE_POWER_DOWN) != MPU9250_OK)
        return MPU9250_ERR;
    HAL_Delay(100);
    /* Enter to continuous measurement mode 2 and set resolution to 16-bit. */
    if (mpu9250_write_byte_ak8963(m, REG_CNTL1, MODE_CONT_MEAS2 | BIT_16) != MPU9250_OK)
        return MPU9250_ERR;
    HAL_Delay(100);

    /* Select clock source to PLL. */
    if (mpu9250_write_byte(m, REG_PWR_MGMT_1, CLKSEL_PLL) != MPU9250_OK)
        return MPU9250_ERR;

    /* Start reading magnetometer value (6 bytes starting from HXL). */
    if (mpu9250_read_ak8963(m, REG_HXL, buf, sizeof(buf)) != MPU9250_OK)
        return MPU9250_ERR;

    return MPU9250_OK;
}

mpu9250_status_t mpu9250_set_range(mpu9250_t *m, mpu9250_acc_range_t acc_range, mpu9250_gyro_range_t gyro_range) {
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
        return MPU9250_ERR;
    }

    if (mpu9250_write_byte(m, REG_ACCEL_CONFIG, (uint8_t)acc_range << ACCEL_FS_SEL_POS) != MPU9250_OK)
        return MPU9250_ERR;

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
        return MPU9250_ERR;
    }

    if (mpu9250_write_byte(m, REG_GYRO_CONFIG, (uint8_t)gyro_range << GYRO_FS_SEL_POS) != MPU9250_OK)
        return MPU9250_ERR;

    return MPU9250_OK;
}

mpu9250_status_t mpu9250_read_acc(mpu9250_t *m, float *acc) {
    uint8_t buf[6];
    uint16_t ax, ay, az;
    mpu9250_status_t res = MPU9250_ERR;

    if (mpu9250_read(m, REG_ACCEL_XOUT_H, buf, sizeof(buf)) == MPU9250_OK) {
        ax = (buf[0] << 8) | buf[1];
        ay = (buf[2] << 8) | buf[3];
        az = (buf[4] << 8) | buf[5];

        acc[0] = to_signed_16(ax) * m->acc_resol;
        acc[1] = to_signed_16(ay) * m->acc_resol;
        acc[2] = to_signed_16(az) * m->acc_resol;

        res = MPU9250_OK;
    }

    return res;
}

mpu9250_status_t mpu9250_read_gyro(mpu9250_t *m, float *gyro) {
    uint8_t buf[6];
    uint16_t gx, gy, gz;
    mpu9250_status_t res = MPU9250_ERR;

    if (mpu9250_read(m, REG_GYRO_XOUT_H, buf, sizeof(buf)) == MPU9250_OK) {
        gx = (buf[0] << 8) | buf[1];
        gy = (buf[2] << 8) | buf[3];
        gz = (buf[4] << 8) | buf[5];

        gyro[0] = to_signed_16(gx) * m->gyro_resol;
        gyro[1] = to_signed_16(gy) * m->gyro_resol;
        gyro[2] = to_signed_16(gz) * m->gyro_resol;

        res = MPU9250_OK;
    }

    return res;
}

mpu9250_status_t mpu9250_read_mag(mpu9250_t *m, float *mag) {
    uint8_t buf[7];
    uint16_t mx, my, mz;
    double mag_raw[3];
    mpu9250_status_t res = MPU9250_ERR;

    if ((mpu9250_read(m, REG_EXT_SENS_DATA_00, buf, sizeof(buf)) == MPU9250_OK) && ((buf[6] & HOFL) == 0U)) {
        mx = (buf[1] << 8) | buf[0];
        my = (buf[3] << 8) | buf[2];
        mz = (buf[5] << 8) | buf[4];

        mag_raw[0] = to_signed_16(mx) * MAG_RESOL * m->xadj;
        mag_raw[1] = to_signed_16(my) * MAG_RESOL * m->yadj;
        mag_raw[2] = to_signed_16(mz) * MAG_RESOL * m->zadj;

        /* Change axes. */
        mag[0] = mag_raw[1];
        mag[1] = mag_raw[0];
        mag[2] = -mag_raw[2];

        res = MPU9250_OK;
    }

    return res;
}

static mpu9250_status_t mpu9250_transmit(mpu9250_t *m, uint8_t *buf, uint16_t len) {
    return HAL_SPI_Transmit(m->hspi, buf, len, HAL_MAX_DELAY) == HAL_OK ? MPU9250_OK : MPU9250_ERR;
}

static mpu9250_status_t mpu9250_receive(mpu9250_t *m, uint8_t *buf, uint16_t len) {
    return HAL_SPI_Receive(m->hspi, buf, len, HAL_MAX_DELAY) == HAL_OK ? MPU9250_OK : MPU9250_ERR;
}

static mpu9250_status_t mpu9250_transmit_receive(mpu9250_t *m, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len) {
    return HAL_SPI_TransmitReceive(m->hspi, tx_buf, rx_buf, len, HAL_MAX_DELAY) == HAL_OK ? MPU9250_OK : MPU9250_ERR;
}

static mpu9250_status_t mpu9250_read_byte(mpu9250_t *m, uint8_t reg, uint8_t *data) {
    uint8_t tx_buf[2] = {reg | FLAG_READ_REG};
    uint8_t rx_buf[2];
    mpu9250_status_t res;

    mpu9250_nss_low(m);
    res = mpu9250_transmit_receive(m, tx_buf, rx_buf, sizeof(tx_buf));
    mpu9250_nss_high(m);

    if (res == MPU9250_OK)
        *data = rx_buf[1];

    return res;
}

static mpu9250_status_t mpu9250_read(mpu9250_t *m, uint8_t reg, uint8_t *data, uint8_t size) {
    uint8_t tx_buf[1] = {reg | FLAG_READ_REG};
    mpu9250_status_t res = MPU9250_ERR;

    mpu9250_nss_low(m);
    if (mpu9250_transmit(m, tx_buf, sizeof(tx_buf)) == MPU9250_OK && mpu9250_receive(m, data, size) == MPU9250_OK)
        res = MPU9250_OK;
    mpu9250_nss_high(m);

    return res;
}

static mpu9250_status_t mpu9250_write_byte(mpu9250_t *m, uint8_t reg, uint8_t data) {
    uint8_t tx_buf[2] = {reg, data};
    mpu9250_status_t res;

    mpu9250_nss_low(m);
    res = mpu9250_transmit(m, tx_buf, sizeof(tx_buf));
    mpu9250_nss_high(m);

    return res;
}

static mpu9250_status_t mpu9250_read_byte_ak8963(mpu9250_t *m, uint8_t reg, uint8_t *data) {
    mpu9250_status_t res = MPU9250_ERR;

    if (mpu9250_write_byte(m, REG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | FLAG_READ_REG) == MPU9250_OK &&
        mpu9250_write_byte(m, REG_I2C_SLV0_REG, reg) == MPU9250_OK &&
        mpu9250_write_byte(m, REG_I2C_SLV0_CTRL, I2C_SLV0_EN | 1) == MPU9250_OK) {
        /* Wait until registers to be filled. */
        HAL_Delay(1);

        if (mpu9250_read_byte(m, REG_EXT_SENS_DATA_00, data) == MPU9250_OK)
            res = MPU9250_OK;
    }

    return res;
}

static mpu9250_status_t mpu9250_read_ak8963(mpu9250_t *m, uint8_t reg, uint8_t *data, uint8_t size) {
    mpu9250_status_t res = MPU9250_ERR;

    if (mpu9250_write_byte(m, REG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | FLAG_READ_REG) == MPU9250_OK &&
        mpu9250_write_byte(m, REG_I2C_SLV0_REG, reg) == MPU9250_OK &&
        mpu9250_write_byte(m, REG_I2C_SLV0_CTRL, I2C_SLV0_EN | size) == MPU9250_OK) {
        /* Wait until registers to be filled. */
        HAL_Delay(1);

        if (mpu9250_read(m, REG_EXT_SENS_DATA_00, data, size) == MPU9250_OK)
            res = MPU9250_OK;
    }

    return res;
}

static mpu9250_status_t mpu9250_write_byte_ak8963(mpu9250_t *m, uint8_t reg, uint8_t data) {
    mpu9250_status_t res = MPU9250_ERR;

    if (mpu9250_write_byte(m, REG_I2C_SLV0_ADDR, AK8963_I2C_ADDR) == MPU9250_OK &&
        mpu9250_write_byte(m, REG_I2C_SLV0_REG, reg) == MPU9250_OK &&
        mpu9250_write_byte(m, REG_I2C_SLV0_DO, data) == MPU9250_OK &&
        mpu9250_write_byte(m, REG_I2C_SLV0_CTRL, I2C_SLV0_EN | 1) == MPU9250_OK)
        res = MPU9250_OK;

    return res;
}

static void mpu9250_nss_low(mpu9250_t *m) {
    HAL_GPIO_WritePin(m->nss_port, m->nss_pin, GPIO_PIN_RESET);
}

static void mpu9250_nss_high(mpu9250_t *m) {
    HAL_GPIO_WritePin(m->nss_port, m->nss_pin, GPIO_PIN_SET);
}
