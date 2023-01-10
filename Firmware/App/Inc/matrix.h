#ifndef MATRIX_H
#define MATRIX_H

#include <stdint.h>

/* Maximum number of matrix elements. */
#define MATRIX_ELEM_MAX_NUM 16

typedef struct {
    uint16_t r, c;                               /* Number of rows and columns. */
    float e[MATRIX_ELEM_MAX_NUM];               /* Array of elements. */
} matrix_t;

typedef enum {
    MATRIX_OK = 0,
    MATRIX_ERR = -1,
} matrix_status_t;

matrix_status_t matrix_init(matrix_t *m, uint16_t r, uint16_t c, const float *elem);
matrix_status_t matrix_zero(matrix_t *m, uint16_t r, uint16_t c);
matrix_status_t matrix_one(matrix_t *m, uint16_t r, uint16_t c);
matrix_status_t matrix_eye(matrix_t *m, uint16_t r);
matrix_status_t matrix_diag(matrix_t *m, uint16_t r, const float *elem);

matrix_status_t matrix_transpose(const matrix_t *m, matrix_t *res);
matrix_status_t matrix_cholesky(const matrix_t *m, matrix_t *res);
matrix_status_t matrix_inv_sym(const matrix_t *m, matrix_t *res);
matrix_status_t matrix_add(const matrix_t *a, const matrix_t *b, matrix_t *res);
matrix_status_t matrix_sub(const matrix_t *a, const matrix_t *b, matrix_t *res);
matrix_status_t matrix_mul(const matrix_t *a, const matrix_t *b, matrix_t *res);

#endif
