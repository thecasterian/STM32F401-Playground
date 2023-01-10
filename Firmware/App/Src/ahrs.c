#include <math.h>
#include "ahrs.h"
#include "cmsis_compiler.h"
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

static void quaternion_acc_mag(const float *acc, const float *mag, const float *mag_inert, quaternion_t *q);

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
            a->mag_bias[i] = (a->mag_max[i] + a->mag_min[i]) / 2.f;
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

    const float P_elem[4] = {0.1f, 0.1f, 0.1f, 0.1f};

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
        mag[i] = ((mag_sum[i] / num_sample) - a->mag_bias[i]) * a->mag_scale[i];
    }

    /* Initial value of quaternion. */
    roll = atan2f(-acc[1], -acc[2]);
    pitch = atan2f(acc[0], NORM_2(acc[1], acc[2]));
    quaternion_from_euler(&a->q, roll, pitch, 0.f);

    /* Magnetic field in inertial frame. */
    quaternion_rot_vec_inv(&a->q, mag, a->mag_inert);

    /* Initial covariance matrix. */
    matrix_diag(&a->P, 4, P_elem);

    return AHRS_OK;
}

ahrs_status_t ahrs_update(ahrs_t *a) {
    float acc[3], gyro[3], mag[3];
    float A_elem[16], pdt2, qdt2, rdt2, x_elem[4], z_elem[4];
    const float Q_elem[4] = {0.01f, 0.01f, 0.01f, 0.01f}, R_elem[4] = {1.f, 1.f, 1.f, 1000.f};
    matrix_t A, A_T, x, x_priori, P_priori, Q, R, S, S_inv, K, z, y, x_corr, P_corr;
    quaternion_t q_z;

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

    /* Calculate state transition matrix with gyro. */
    pdt2 = a->dt / 2.f * gyro[0];
    qdt2 = a->dt / 2.f * gyro[1];
    rdt2 = a->dt / 2.f * gyro[2];

    A_elem[0] = 1.f;
    A_elem[1] = pdt2;
    A_elem[2] = qdt2;
    A_elem[3] = rdt2;
    A_elem[4] = -pdt2;
    A_elem[5] = 1.f;
    A_elem[6] = rdt2;
    A_elem[7] = -qdt2;
    A_elem[8] = -qdt2;
    A_elem[9] = -rdt2;
    A_elem[10] = 1.f;
    A_elem[11] = pdt2;
    A_elem[12] = -rdt2;
    A_elem[13] = qdt2;
    A_elem[14] = -pdt2;
    A_elem[15] = 1.f;
    matrix_init(&A, 4, 4, A_elem);
    matrix_transpose(&A, &A_T);

    /* Prediction. */
    x_elem[0] = a->q.w;
    x_elem[1] = a->q.x;
    x_elem[2] = a->q.y;
    x_elem[3] = a->q.z;
    matrix_init(&x, 4, 1, x_elem);
    matrix_diag(&Q, 4, Q_elem);

    matrix_mul(&A, &x, &x_priori);
    matrix_mul(&A, &a->P, &P_priori);
    matrix_mul(&P_priori, &A_T, &P_priori);
    matrix_add(&P_priori, &Q, &P_priori);

    /* Correction. */
    matrix_diag(&R, 4, R_elem);
    matrix_add(&P_priori, &R, &S);
    matrix_inv_sym(&S, &S_inv);
    matrix_mul(&P_priori, &S_inv, &K);

    quaternion_acc_mag(acc, mag, a->mag_inert, &q_z);
    z_elem[0] = q_z.w;
    z_elem[1] = q_z.x;
    z_elem[2] = q_z.y;
    z_elem[3] = q_z.z;
    matrix_init(&z, 4, 1, z_elem);
    matrix_sub(&z, &x_priori, &y);
    matrix_mul(&K, &y, &x_corr);
    matrix_add(&x_priori, &x_corr, &x);
    quaternion_init(&a->q, x.e[0], x.e[1], x.e[2], x.e[3]);
    quaternion_normalize(&a->q, &a->q);

    matrix_mul(&K, &P_priori, &P_corr);
    matrix_sub(&P_priori, &P_corr, &a->P);

    return AHRS_OK;
}

/* Calculate quaternion with accelerometer and magnetometer. */
static void quaternion_acc_mag(const float *acc, const float *mag, const float *mag_inert, quaternion_t *q) {
    quaternion_t q_acc;
    float roll, pitch, yaw;
    float mag_acc[3], sy, cy;

    roll = atan2f(-acc[1], -acc[2]);
    pitch = atan2f(acc[0], NORM_2(acc[1], acc[2]));
    quaternion_from_euler(&q_acc, roll, pitch, 0.f);

    quaternion_rot_vec_inv(&q_acc, mag, mag_acc);
    sy = (mag_inert[1] * mag_acc[0]) - (mag_inert[0] * mag_acc[1]);
    cy = (mag_inert[0] * mag_acc[0]) + (mag_inert[1] * mag_acc[1]);
    yaw = atan2f(sy, cy);

    quaternion_from_euler(q, roll, pitch, yaw);
}
