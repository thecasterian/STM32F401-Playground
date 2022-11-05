#ifndef UTIL_H
#define UTIL_H

#include <stdio.h>
#include <stdint.h>

#ifndef NO_DBG
 #define DBG_(FMT, ...) \
    printf("%s:%d:debug: " FMT "%s\n", __FILE__, __LINE__, __VA_ARGS__)
 #define DBG(...) \
    DBG_(__VA_ARGS__, "")
#else
 #define DBG(...)
#endif // NO_DBG

#define BYTE_0(x) ((x) & 0xFF)
#define BYTE_1(x) (((x) >> 8) & 0xFF)
#define BYTE_2(x) (((x) >> 16) & 0xFF)
#define BYTE_3(x) (((x) >> 24) & 0xFF)

#define PACK_2(b1, b0) ((b1) << 8 | (b0))
#define PACK_4(b3, b2, b1, b0) ((b3) << 24 | (b2) << 16 | (b1) << 8 | (b0))

#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))

/* Inaccurate microsecond delay function. */
void delay_us(uint32_t us);

int16_t to_signed_16(uint16_t x);

#endif
