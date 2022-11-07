#include "math.h"
#include "quaternion.h"

void quaternion_init(quaternion_t *q, float x, float y, float z, float w) {
    q->x = x;
    q->y = y;
    q->z = z;
    q->w = w;
}

void quaternion_from_euler(quaternion_t *q, float roll, float pitch, float yaw)
{
    float cr, cp, cy, sr, sp, sy;

    cr = cosf(roll / 2.f);
    cp = cosf(pitch / 2.f);
    cy = cosf(yaw / 2.f);

    sr = sinf(roll / 2.f);
    sp = sinf(pitch / 2.f);
    sy = sinf(yaw / 2.f);

    q->x = cr * sp * sy - sr * cp * cy;
    q->y = -(cr * sp * cy + sr * cp * sy);
    q->z = sr * sp * cy - cr * cp * sy;
    q->w = cr * cp * cy + sr * sp * sy;
}

void quaternion_scale(const quaternion_t *q, float scale, quaternion_t *res) {
    quaternion_t tmp;

    tmp.x = q->x * scale;
    tmp.y = q->y * scale;
    tmp.z = q->z * scale;
    tmp.w = q->w * scale;

    *res = tmp;
}

void quaternion_normalize(const quaternion_t *q, quaternion_t *res) {
    float norm;

    norm = sqrtf((q->x * q->x) + (q->y * q->y) + (q->z * q->z) + (q->w * q->w));
    res->x = q->x / norm;
    res->y = q->y / norm;
    res->z = q->z / norm;
    res->w = q->w / norm;
}

void quaternion_add(const quaternion_t *q1, const quaternion_t *q2, quaternion_t *res) {
    res->x = q1->x + q2->x;
    res->y = q1->y + q2->y;
    res->z = q1->z + q2->z;
    res->w = q1->w + q2->w;
}

void quaternion_mul(const quaternion_t *q1, const quaternion_t *q2, quaternion_t *res) {
    quaternion_t tmp;

    tmp.x = (q1->x * q2->w) + (q1->w * q2->x) + (q1->y * q2->z) - (q1->z * q2->y);
    tmp.y = (q1->w * q2->y) - (q1->x * q2->z) + (q1->y * q2->w) + (q1->z * q2->y);
    tmp.z = (q1->w * q2->z) + (q1->x * q2->y) - (q1->y * q2->x) + (q1->z * q2->w);
    tmp.w = (q1->w * q2->w) - (q1->x * q2->x) - (q1->y * q2->y) - (q1->z * q2->z);

    *res = tmp;
}

void quaternion_rot_vec(const quaternion_t *q, const float *vec, float *res) {
    quaternion_t q_vec = {vec[0], vec[1], vec[2], 0.f}, q_conj = {-q->x, -q->y, -q->z, q->w}, q_res;

    quaternion_mul(q, &q_vec, &q_vec);
    quaternion_mul(&q_vec, &q_conj, &q_res);

    res[0] = q_res.x;
    res[1] = q_res.y;
    res[2] = q_res.z;
}

void quaternion_rot_vec_inv(const quaternion_t *q, const float *vec, float *res) {
    quaternion_t q_vec = {vec[0], vec[1], vec[2], 0.f}, q_conj = {-q->x, -q->y, -q->z, q->w}, q_res;

    quaternion_mul(&q_conj, &q_vec, &q_vec);
    quaternion_mul(&q_vec, q, &q_res);

    res[0] = q_res.x;
    res[1] = q_res.y;
    res[2] = q_res.z;
}

void quaternion_to_euler(const quaternion_t *q, float *roll, float *pitch, float *yaw) {
    *roll = atan2f(2.f * ((q->y * q->z) - (q->w * q->x)), 1.f - 2.f * ((q->x * q->x) + (q->y * q->y)));
    *pitch = asinf(-2.f * ((q->w * q->y) + (q->x * q->z)));
    *yaw = atan2f(2.f * ((q->x * q->y) - (q->w * q->z)), 1.f - 2.f * ((q->y * q->y) + (q->z * q->z)));
}
