#ifndef APPLICATION_H
#define APPLICATION_H

#include "w25q128jv.h"
#include "mpu9250.h"

extern w25q128jv_t w25q128jv;
extern mpu9250_t mpu9250;

void setup(void);
void loop(void);

#endif
