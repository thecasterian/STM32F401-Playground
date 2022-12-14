#ifndef AHRS_H
#define AHRS_H

#include <stdint.h>
#include "matrix.h"
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

    quaternion_t q;                             /* Quaternion from body frame to inertial frame. */
    matrix_t P;                                 /* Covariance matrix. */

    float mag_inert[3];                         /* Magnetic field in inertial frame. */

    float gyro_bias[3];
    float mag_max[3], mag_min[3];
    float mag_bias[3];                          /* Bias for hard iron calibration. */
    float mag_scale[3];                         /* Scale factor for soft iron calibration. */
} ahrs_t;

ahrs_status_t ahrs_init(ahrs_t *a, float dt, ahrs_read_t read_acc, void *aux_acc, ahrs_read_t read_gyro, void *aux_gyro,
                        ahrs_read_t read_mag, void *aux_mag);

ahrs_status_t ahrs_init_attitude(ahrs_t *a, uint16_t num_sample);

ahrs_status_t ahrs_update(ahrs_t *a);

#endif
