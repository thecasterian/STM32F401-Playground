#ifndef TIMER_H
#define TIMER_H

#include <stdbool.h>

typedef struct {
    volatile bool period_elapsed;
} Timer;

extern Timer timer;

void timer_start(void);

#endif