#include <math.h>
#include "ahrs.h"
#include "timer_wrapper.h"

ahrs_status_t ahrs_init(ahrs_t *a, float dt, ahrs_read_t read_acc, void *aux_acc, ahrs_read_t read_gyro, void *aux_gyro,
                        ahrs_read_t read_mag, void *aux_mag) {
    ahrs_status_t res = AHRS_ERR;

    if (dt > 0 && read_acc && read_gyro && read_mag) {
        a->dt = dt;
        a->read_acc = read_acc;
        a->aux_acc = aux_acc;
        a->read_gyro = read_gyro;
        a->aux_gyro = aux_gyro;
        a->read_mag = read_mag;
        a->aux_mag = aux_mag;

        a->alpha_acc = 0.1f;
        a->alpha_gyro = 0.01f;
        a->alpha_mag = 0.1f;

        res = AHRS_OK;
    }

    return res;
}

ahrs_status_t ahrs_set_iir(ahrs_t *a, float alpha_acc, float alpha_gyro, float alpha_mag) {
    ahrs_status_t res = AHRS_ERR;

    if (alpha_acc > 0 && alpha_acc <= 1 && alpha_gyro > 0 && alpha_gyro <= 1 && alpha_mag > 0 && alpha_mag <= 1) {
        a->alpha_acc = alpha_acc;
        a->alpha_gyro = alpha_gyro;
        a->alpha_mag = alpha_mag;

        res = AHRS_OK;
    }

    return res;
}

ahrs_status_t ahrs_calibrate_acc_gyro(ahrs_t *a, uint16_t num_sample) {
    float acc[3], acc_sum[3] = {0.f, 0.f, 0.f};
    float gyro[3], gyro_sum[3] = {0.f, 0.f, 0.f};
    float roll, pitch;
    uint16_t i;

    i = 0UL;
    while (i < num_sample) {
        if (timer.period_elapsed) {
            if (a->read_acc(a->aux_acc, acc) == AHRS_OK && a->read_gyro(a->aux_gyro, gyro) == AHRS_OK) {
                acc_sum[0] += acc[0];
                acc_sum[1] += acc[1];
                acc_sum[2] += acc[2];

                gyro_sum[0] += gyro[0];
                gyro_sum[1] += gyro[1];
                gyro_sum[2] += gyro[2];

                i++;
            }

            timer.period_elapsed = false;
        }
    }

    a->gyro_bias[0] = gyro_sum[0] / num_sample;
    a->gyro_bias[1] = gyro_sum[1] / num_sample;
    a->gyro_bias[2] = gyro_sum[2] / num_sample;

    /* Initial values of IIR-filtered acc and gyro. */
    for (int i = 0; i < 3; i++) {
        a->acc_iir[i] = acc_sum[i] / num_sample;
        a->gyro_iir[i] = 0.f;
    }

    /* Initial value of quaternion. */
    roll = atan2f(a->acc_iir[1], a->acc_iir[2]);
    pitch = atan2f(a->acc_iir[0], sqrtf(a->acc_iir[1] * a->acc_iir[1] + a->acc_iir[2] * a->acc_iir[2]));
    quaternion_from_euler(&a->q, roll, pitch, 0.f);

    return AHRS_OK;
}

ahrs_status_t ahrs_update(ahrs_t *a) {
    float acc[3], gyro[3];
    quaternion_t q_acc, q_gyro, q_tmp;
    float roll, pitch;

    a->read_acc(a->aux_acc, acc);
    a->read_gyro(a->aux_gyro, gyro);

    /* First-order IIR filter. */
    for (int i = 0; i < 3; i++) {
        a->acc_iir[i] = (1.f - a->alpha_acc) * a->acc_iir[i] + a->alpha_acc * acc[i];
        a->gyro_iir[i] = (1.f - a->alpha_gyro) * a->gyro_iir[i] + a->alpha_gyro * gyro[i];
    }

    /* Calculate quaternion with accelerometer. */
    roll = atan2f(a->acc_iir[1], a->acc_iir[2]);
    pitch = atan2f(a->acc_iir[0], sqrtf(a->acc_iir[1] * a->acc_iir[1] + a->acc_iir[2] * a->acc_iir[2]));
    quaternion_from_euler(&q_acc, roll, pitch, 0.f);

    /* Calculate quaternion with gyro. */
    q_tmp.x = a->gyro_iir[0];
    q_tmp.y = a->gyro_iir[1];
    q_tmp.z = a->gyro_iir[2];
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
