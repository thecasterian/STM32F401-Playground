#include "tim.h"
#include "timer_wrapper.h"

Timer timer;

void timer_start(void) {
    HAL_TIM_Base_Start_IT(&htim5);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    timer.period_elapsed = true;
}
