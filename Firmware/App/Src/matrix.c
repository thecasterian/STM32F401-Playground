#include <math.h>
#include "matrix.h"

#define ELEM(M, I, J) ((M)->e[((I) * (M)->c) + (J)])

matrix_status_t matrix_init(matrix_t *m, uint16_t r, uint16_t c, const float *elem) {
    if ((r * c) > MATRIX_ELEM_MAX_NUM) {
        return MATRIX_ERR;
    }

    m->r = r;
    m->c = c;
    for (uint16_t i = 0U; i < (r * c); i++) {
        m->e[i] = elem[i];
    }

    return MATRIX_OK;
}

matrix_status_t matrix_zero(matrix_t *m, uint16_t r, uint16_t c) {
    if ((r * c) > MATRIX_ELEM_MAX_NUM) {
        return MATRIX_ERR;
    }

    m->r = r;
    m->c = c;
    for (uint16_t i = 0U; i < (r * c); i++) {
        m->e[i] = 0.f;
    }

    return MATRIX_OK;
}

matrix_status_t matrix_one(matrix_t *m, uint16_t r, uint16_t c) {
    if ((r * c) > MATRIX_ELEM_MAX_NUM) {
        return MATRIX_ERR;
    }

    m->r = r;
    m->c = c;
    for (uint16_t i = 0U; i < (r * c); i++) {
        m->e[i] = 1.f;
    }

    return MATRIX_OK;
}

matrix_status_t matrix_eye(matrix_t *m, uint16_t r) {
    if ((r * r) > MATRIX_ELEM_MAX_NUM) {
        return MATRIX_ERR;
    }

    m->r = r;
    m->c = r;
    for (uint16_t i = 0U; i < (r * r); i++) {
        m->e[i] = 0.f;
    }
    for (uint16_t i = 0U; i < r; i++) {
        ELEM(m, i, i) = 1.f;
    }

    return MATRIX_OK;
}

matrix_status_t matrix_diag(matrix_t *m, uint16_t r, const float *elem) {
    if ((r * r) > MATRIX_ELEM_MAX_NUM) {
        return MATRIX_ERR;
    }

    m->r = r;
    m->c = r;
    for (uint16_t i = 0U; i < (r * r); i++) {
        m->e[i] = 0.f;
    }
    for (uint16_t i = 0U; i < r; i++) {
        ELEM(m, i, i) = elem[i];
    }

    return MATRIX_OK;
}

matrix_status_t matrix_transpose(const matrix_t *m, matrix_t *res) {
    matrix_t tmp;

    tmp.r = m->c;
    tmp.c = m->r;
    for (uint16_t i = 0U; i < m->r; i++) {
        for (uint16_t j = 0U; j < m->c; j++) {
            ELEM(&tmp, j, i) = ELEM(m, i, j);
        }
    }

    *res = tmp;
    return MATRIX_OK;
}

matrix_status_t matrix_cholesky(const matrix_t *m, matrix_t *res) {
    matrix_t tmp;
    float s;

    if (m->r != m->c) {
        return MATRIX_ERR;
    }

    matrix_zero(&tmp, m->r, m->c);
    for (uint16_t i = 0U; i < m->r; i++) {
        for (uint16_t j = 0U; j <= i; j++) {
            s = ELEM(m, i, j);
            for (uint16_t k = 0U; k < j; k++) {
                s -= ELEM(&tmp, i, k) * ELEM(&tmp, j, k);
            }

            if (i == j) {
                if (s < 0.f) {
                    return MATRIX_ERR;
                }
                ELEM(&tmp, i, j) = sqrtf(s);
            } else {
                ELEM(&tmp, i, j) = s / ELEM(&tmp, j, j);
            }
        }
    }

    *res = tmp;
    return MATRIX_OK;
}

matrix_status_t matrix_inv_sym(const matrix_t *m, matrix_t *res) {
    matrix_t L, L_inv;

    if (matrix_cholesky(m, &L) != MATRIX_OK) {
        return MATRIX_ERR;
    }

    matrix_zero(&L_inv, L.r, L.c);
    for (uint16_t i = 0U; i < L.r; i++) {
        ELEM(&L_inv, i, i) = 1.f / ELEM(&L, i, i);
        for (uint16_t j = 0U; j < i; j++) {
            for (uint16_t k = j; k < i; k++) {
                ELEM(&L_inv, i, j) += ELEM(&L, i, k) * ELEM(&L_inv, k, j);
            }
            ELEM(&L_inv, i, j) = -ELEM(&L_inv, i, j) / ELEM(&L, i, i);
        }
    }

    matrix_transpose(&L_inv, res);
    matrix_mul(res, &L_inv, res);

    return MATRIX_OK;
}

matrix_status_t matrix_add(const matrix_t *a, const matrix_t *b, matrix_t *res) {
    if ((a->r != b->r) || (a->c != b->c)) {
        return MATRIX_ERR;
    }

    res->r = a->r;
    res->c = a->c;
    for (uint16_t i = 0U; i < (a->r * a->c); i++) {
        res->e[i] = a->e[i] + b->e[i];
    }

    return MATRIX_OK;
}

matrix_status_t matrix_sub(const matrix_t *a, const matrix_t *b, matrix_t *res) {
    if ((a->r != b->r) || (a->c != b->c)) {
        return MATRIX_ERR;
    }

    res->r = a->r;
    res->c = a->c;
    for (uint16_t i = 0U; i < (a->r * a->c); i++) {
        res->e[i] = a->e[i] - b->e[i];
    }

    return MATRIX_OK;
}

matrix_status_t matrix_mul(const matrix_t *a, const matrix_t *b, matrix_t *res) {
    matrix_t tmp;

    if ((a->c != b->r) || ((a->r * b->c) > MATRIX_ELEM_MAX_NUM)) {
        return MATRIX_ERR;
    }

    tmp.r = a->r;
    tmp.c = b->c;
    for (uint16_t i = 0U; i < a->r; i++) {
        for (uint16_t j = 0U; j < b->c; j++) {
            ELEM(&tmp, i, j) = 0.f;
            for (uint16_t k = 0U; k < a->c; k++) {
                ELEM(&tmp, i, j) += ELEM(a, i, k) * ELEM(b, k, j);
            }
        }
    }

    *res = tmp;
    return MATRIX_OK;
}
