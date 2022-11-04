#ifndef QUATERNION_H
#define QUATERNION_H

typedef struct {
    float x;                                    /* i-term. */
    float y;                                    /* j-term. */
    float z;                                    /* k-term. */
    float w;                                    /* Real part. */
} quaternion_t;

void quaternion_init(quaternion_t *q, float x, float y, float z, float w);

void quaternion_scale(const quaternion_t *q, float scale, quaternion_t *res);
void quaternion_normalize(const quaternion_t *q, quaternion_t *res);

void quaternion_add(const quaternion_t *q1, const quaternion_t *q2, quaternion_t *res);
void quaternion_mul(const quaternion_t *q1, const quaternion_t *q2, quaternion_t *res);

#endif
