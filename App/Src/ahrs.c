#include <math.h>
#include "ahrs.h"
#include "timer_wrapper.h"
#include "util.h"

#define GYRO_BIAS_X -0.02060605f
#define GYRO_BIAS_Y -0.00268946f
#define GYRO_BIAS_Z -0.02197092f

#define MAG_X_MAX  76.139702f
#define MAG_X_MIN   3.460896f
#define MAG_Y_MAX  40.802139f
#define MAG_Y_MIN -39.527073f
#define MAG_Z_MAX 203.973785f
#define MAG_Z_MIN 100.664680f

#define MAG_IIR_ALPHA 0.5f

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
                    mag_sum[i] += (mag[i] - a->mag_bias[i]) * a->mag_scale[i];
                }

                cnt++;
            }

            timer.period_elapsed = false;
        }
    }

    /* Initial values of accelerometer and magnetometer. */
    for (int i = 0; i < 3; i++) {
        acc[i] = acc_sum[i] / num_sample;
        a->mag_iir[i] = mag_sum[i] / num_sample;
    }

    /* Initial value of quaternion. */
    roll = atan2f(-acc[1], -acc[2]);
    pitch = atan2f(acc[0], NORM_2(acc[1], acc[2]));
    quaternion_from_euler(&a->q_acc_mag, roll, pitch, 0.f);
    a->q_gyro = a->q_acc_mag;
    a->q_kalman = a->q_acc_mag;

    /* Magnetic field in inertial frame. */
    quaternion_rot_vec_inv(&a->q_acc_mag, a->mag_iir, a->mag_inert);

    return AHRS_OK;
}

ahrs_status_t ahrs_update(ahrs_t *a) {
    float acc[3], gyro[3], mag[3];
    quaternion_t q_acc, q_tmp;
    float roll, pitch;
    float mag_acc[3], yaw, mag_inert_norm, mag_acc_norm, cy;

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

    /* Calculate quaternion with accelerometer and magnetometer. */
    roll = atan2f(-acc[1], -acc[2]);
    pitch = atan2f(acc[0], NORM_2(acc[1], acc[2]));
    quaternion_from_euler(&q_acc, roll, pitch, 0.f);

    quaternion_rot_vec_inv(&q_acc, a->mag_iir, mag_acc);
    mag_inert_norm = NORM_2(a->mag_inert[0], a->mag_inert[1]);
    mag_acc_norm = NORM_2(mag_acc[0], mag_acc[1]);
    cy = ((a->mag_inert[0] * mag_acc[0]) + (a->mag_inert[1] * mag_acc[1])) / (mag_inert_norm * mag_acc_norm);
    yaw = acosf(cy);
    if (a->mag_inert[0] * mag_acc[1] - a->mag_inert[1] * mag_acc[0] > 0.f) {
        yaw = -yaw;
    }

    quaternion_from_euler(&q_tmp, 0.f, 0.f, yaw);
    quaternion_mul(&q_acc, &q_tmp, &a->q_acc_mag);
    quaternion_normalize(&a->q_acc_mag, &a->q_acc_mag);

    /* Calculate quaternion with gyro. */
    quaternion_init(&q_tmp, 1.f, -a->dt / 2.f * gyro[0], -a->dt / 2.f * gyro[1], -a->dt / 2.f * gyro[2]);
    quaternion_mul(&q_tmp, &a->q_gyro, &a->q_gyro);
    quaternion_normalize(&a->q_gyro, &a->q_gyro);

    // TODO: implement Kalman filter.

    return AHRS_OK;
}
