#include "osal.h"
#include "stm32f4xx_hal.h"

// TIM2 is configured as a free-running 32-bit microsecond counter
// It runs at 1MHz (84MHz / 84 prescaler)
extern TIM_HandleTypeDef htim2;

static uint32_t _osal_gettick_us(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2);
}

ec_timet osal_current_time(void)
{
    ec_timet t;
    uint32_t tick = _osal_gettick_us();
    t.sec = tick / 1000000;
    t.usec = tick % 1000000;
    return t;
}

void osal_timer_start(osal_timert *self, uint32 timeout_usec)
{
    ec_timet now = osal_current_time();
    self->stop_time.sec = now.sec;
    self->stop_time.usec = now.usec;
    
    // Handle overflow properly
    self->stop_time.sec += timeout_usec / 1000000;
    self->stop_time.usec += timeout_usec % 1000000;
    
    while (self->stop_time.usec >= 1000000) {
        self->stop_time.sec++;
        self->stop_time.usec -= 1000000;
    }
}

boolean osal_timer_is_expired(osal_timert *self)
{
    ec_timet now = osal_current_time();
    if (now.sec > self->stop_time.sec)
        return TRUE;
    if (now.sec == self->stop_time.sec && now.usec >= self->stop_time.usec)
        return TRUE;
    return FALSE;
}

int osal_usleep(uint32 usec)
{
    uint32_t start = _osal_gettick_us();
    uint32_t current;
    int loop_count = 0;
    do {
        current = _osal_gettick_us();
        loop_count++;
        if (loop_count > 1000000) {
            /* Prevent infinite loop - break after 1M iterations */
            break;
        }
    } while ((current - start) < usec);
    return 0;
}

void osal_time_diff(ec_timet *start, ec_timet *end, ec_timet *diff)
{
    if (end->usec < start->usec) {
        diff->sec = end->sec - start->sec - 1;
        diff->usec = end->usec + 1000000 - start->usec;
    } else {
        diff->sec = end->sec - start->sec;
        diff->usec = end->usec - start->usec;
    }
}

int osal_thread_create(void *thandle, int stacksize, void *func, void *param)
{
    // No RTOS - threads not supported in bare-metal
    return 0;
}

int osal_thread_create_rt(void *thandle, int stacksize, void *func, void *param)
{
    return 0;
}
