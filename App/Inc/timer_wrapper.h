#ifndef TIMER_H
#define TIMER_H

#include <stdbool.h>

typedef struct {
    bool period_elapsed;
} Timer;

extern Timer timer;

void timer_start(Timer *t);

#endif