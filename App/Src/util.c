#include <string.h>
#include "util.h"

int16_t to_signed_16(uint16_t x) {
    int16_t res;

    (void)memcpy(&res, &x, sizeof(res));

    return res;
}
