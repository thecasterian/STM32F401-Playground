#ifndef AHRS_H
#define AHRS_H

#include <stdint.h>
#include "quaternion.h"

typedef enum {
    AHRS_OK = 0,
    AHRS_ERR = -1,
} ahrs_status_t;

typedef ahrs_status_t (*ahrs_read_t)(void *aux, float *data);
typedef struct {
    float dt;

    ahrs_read_t read_acc, read_gyro, read_mag;
    void *aux_acc, *aux_gyro, *aux_mag;

    float alpha_acc, alpha_gyro, alpha_mag;

    float gyro_bias[3];

    float acc_iir[3], gyro_iir[3], mag_iir[3];
    quaternion_t q;
} ahrs_t;

ahrs_status_t ahrs_init(ahrs_t *a, float dt, ahrs_read_t read_acc, void *aux_acc, ahrs_read_t read_gyro, void *aux_gyro,
                        ahrs_read_t read_mag, void *aux_mag);
ahrs_status_t ahrs_set_iir(ahrs_t *a, float alpha_acc, float alpha_gyro, float alpha_mag);

ahrs_status_t ahrs_calibrate_acc_gyro(ahrs_t *a, uint16_t num_sample);

ahrs_status_t ahrs_update(ahrs_t *a);

ahrs_status_t ahrs_get_quaternion(ahrs_t *a, quaternion_t *q);

#endif
