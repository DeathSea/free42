#include <timer.h>

void main_timeout1(uint8_t key)
{
    end_timer();
    if (key != 0) {
        core_keytimeout1();
        cur_timer = TIMER_TIMEROUT2;
        start_timer2();
    }
}

void main_timeout2(uint8_t key)
{
    end_timer();
    if (key != 0) {
        core_keytimeout2();
    }
}

void main_timeout3(uint8_t key)
{
    end_timer3();
    core_timeout3(1);
}

void main_repeater(uint8_t key)
{
    end_timer();
    int repeat = core_repeat();
    if (repeat != 0) {
        if (repeat == 1) {
            cur_timer = TIMER_REPEATER;
            start_repeater(200);
        } else {
            cur_timer = TIMER_REPEATER;
            start_repeater(100);
        }
    } else {
        cur_timer = TIMER_TIMEROUT1;
        start_timer1();
    }
}

void end_timer()
{
    if (cur_timer != TIMER_END) {
        g_timer[cur_timer].enable = 0;
        g_timer[cur_timer].start_tick = 0;
        cur_timer = TIMER_END;
    }
}

void end_timer1()
{
    g_timer[TIMER_TIMEROUT1].enable = 0;
    g_timer[TIMER_TIMEROUT1].start_tick = 0;
}

void start_timer1()
{
    g_timer[TIMER_TIMEROUT1].enable = 1;
    g_timer[TIMER_TIMEROUT1].start_tick = HAL_GetTick();
}

void end_timer2()
{
    g_timer[TIMER_TIMEROUT2].enable = 0;
    g_timer[TIMER_TIMEROUT2].start_tick = 0;
}

void start_timer2()
{
    g_timer[TIMER_TIMEROUT2].enable = 1;
    g_timer[TIMER_TIMEROUT2].start_tick = HAL_GetTick();
}

void end_timer3()
{
    g_timer[TIMER_TIMEROUT3].enable = 0;
    g_timer[TIMER_TIMEROUT3].start_tick = 0;
}

void start_timer3()
{
    g_timer[TIMER_TIMEROUT3].enable = 1;
    g_timer[TIMER_TIMEROUT3].start_tick = HAL_GetTick();
}

void end_repeater()
{
    g_timer[TIMER_REPEATER].enable = 0;
    g_timer[TIMER_REPEATER].start_tick = 0;
}

void start_repeater(uint32_t delay)
{
    g_timer[TIMER_REPEATER].enable = 1;
    g_timer[TIMER_REPEATER].start_tick = HAL_GetTick();
    g_timer[TIMER_REPEATER].timeout = delay;
}
