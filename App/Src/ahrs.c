#include <math.h>
#include "ahrs.h"
#include "timer_wrapper.h"

#define GYRO_BIAS_X -0.02060605f
#define GYRO_BIAS_Y -0.00268946f
#define GYRO_BIAS_Z -0.02197092f

#define MAG_X_MAX  76.139702f
#define MAG_X_MIN   3.460896f
#define MAG_Y_MAX  40.802139f
#define MAG_Y_MIN -39.527073f
#define MAG_Z_MAX 203.973785f
#define MAG_Z_MIN 100.664680f

#define MAG_IIR_ALPHA 0.1f

ahrs_status_t ahrs_init(ahrs_t *a, float dt, ahrs_read_t read_acc, void *aux_acc, ahrs_read_t read_gyro, void *aux_gyro,
                        ahrs_read_t read_mag, void *aux_mag) {
    float dmag[3], davg;
    ahrs_status_t res = AHRS_ERR;

    if (dt > 0 && read_acc && read_gyro && read_mag) {
        a->dt = dt;
        a->read_acc = read_acc;
        a->aux_acc = aux_acc;
        a->read_gyro = read_gyro;
        a->aux_gyro = aux_gyro;
        a->read_mag = read_mag;
        a->aux_mag = aux_mag;

        a->gyro_bias[0] = GYRO_BIAS_X;
        a->gyro_bias[1] = GYRO_BIAS_Y;
        a->gyro_bias[2] = GYRO_BIAS_Z;

        a->mag_max[0] = MAG_X_MAX;
        a->mag_max[1] = MAG_Y_MAX;
        a->mag_max[2] = MAG_Z_MAX;
        a->mag_min[0] = MAG_X_MIN;
        a->mag_min[1] = MAG_Y_MIN;
        a->mag_min[2] = MAG_Z_MIN;

        for (int i = 0; i < 3; i++) {
            a->mag_bias[i] = (a->mag_max[i] + a->mag_min[i]) / 2;
            dmag[i] = (a->mag_max[i] - a->mag_min[i]) / 2.f;
        }
        davg = (dmag[0] + dmag[1] + dmag[2]) / 3.f;
        for (int i = 0; i < 3; i++) {
            a->mag_scale[i] = davg / dmag[i];
        }

        res = AHRS_OK;
    }

    return res;
}

ahrs_status_t ahrs_init_attitude(ahrs_t *a, uint16_t num_sample) {
    float acc[3], acc_sum[3] = {0.f, 0.f, 0.f};
    float mag[3], mag_sum[3] = {0.f, 0.f, 0.f};
    float roll, pitch;
    uint16_t cnt;

    cnt = 0UL;
    while (cnt < num_sample) {
        if (timer.period_elapsed) {
            if (a->read_acc(a->aux_acc, acc) == AHRS_OK && a->read_mag(a->aux_mag, mag) == AHRS_OK) {
                for (int i = 0; i < 3; i++) {
                    acc_sum[i] += acc[i];
                    mag_sum[i] += mag[i];
                }

                cnt++;
            }

            timer.period_elapsed = false;
        }
    }

    /* Initial values of accelerometer and magnetometer. */
    for (int i = 0; i < 3; i++) {
        acc[i] = acc_sum[i] / num_sample;
        a->mag_init[i] = mag_sum[i] / num_sample;
        a->mag_iir[i] = a->mag_init[i];
    }

    /* Initial value of quaternion. */
    roll = atan2f(acc[1], acc[2]);
    pitch = atan2f(acc[0], sqrtf(acc[1] * acc[1] + acc[2] * acc[2]));
    quaternion_from_euler(&a->q, roll, pitch, 0.f);

    return AHRS_OK;
}

ahrs_status_t ahrs_update(ahrs_t *a) {
    float acc[3], gyro[3], mag[3];
    quaternion_t q_acc, q_gyro, q_tmp;
    float roll, pitch;

    a->read_acc(a->aux_acc, acc);
    a->read_gyro(a->aux_gyro, gyro);
    a->read_mag(a->aux_mag, mag);

    /* Remove gyro bias. */
    for (int i = 0; i < 3; i++) {
        gyro[i] -= a->gyro_bias[i];
    }

    /* Hard and soft iron calibration. */
    for (int i = 0; i < 3; i++) {
        mag[i] = (mag[i] - a->mag_bias[i]) * a->mag_scale[i];
    }
    /* Apply IIR filter. */
    for (int i = 0; i < 3; i++) {
        a->mag_iir[i] = ((1.f - MAG_IIR_ALPHA) * a->mag_iir[i]) + (MAG_IIR_ALPHA * mag[i]);
    }

    /* Calculate quaternion with accelerometer. */
    roll = atan2f(acc[1], acc[2]);
    pitch = atan2f(acc[0], sqrtf(acc[1] * acc[1] + acc[2] * acc[2]));
    quaternion_from_euler(&q_acc, roll, pitch, 0.f);

    /* Calculate quaternion with gyro. */
    q_tmp.x = gyro[0];
    q_tmp.y = gyro[1];
    q_tmp.z = gyro[2];
    q_tmp.w = 0.f;

    quaternion_mul(&a->q, &q_tmp, &q_tmp);
    quaternion_scale(&q_tmp, -a->dt / 2.f, &q_tmp);
    quaternion_add(&a->q, &q_tmp, &q_gyro);
    quaternion_normalize(&q_gyro, &q_gyro);

    // TODO: implement Kalman filter.
    a->q = q_acc;

    return AHRS_OK;
}

ahrs_status_t ahrs_get_quaternion(ahrs_t *a, quaternion_t *q) {
    *q = a->q;

    return AHRS_OK;
}
