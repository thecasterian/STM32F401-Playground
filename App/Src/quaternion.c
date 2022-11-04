#include "math.h"
#include "quaternion.h"

void quaternion_init(quaternion_t *q, float x, float y, float z, float w) {
    q->x = x;
    q->y = y;
    q->z = z;
    q->w = w;
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
