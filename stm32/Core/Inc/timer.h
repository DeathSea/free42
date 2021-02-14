#ifndef __TIMER_H
#define __TIMER_H
#include "stm32l4xx_hal.h"
#define TIMER_TIMEROUT1 0
#define TIMER_TIMEROUT2 1
#define TIMER_TIMEROUT3 2
#define TIMER_REPEATER 3
#define TIMER_END 4
uint8_t cur_timer;

void main_timeout1(uint8_t);
void main_timeout2(uint8_t);
void main_timeout3(uint8_t);
void main_repeater(uint8_t);
void start_timer1();
void end_timer1();
void start_timer2();
void end_timer2();
void start_timer3();
void end_timer3();
void start_repeater(uint32_t delay);
void end_repeater();

struct timer {
    uint8_t enable;
    uint32_t start_tick;
    uint32_t timeout;
    void (* callback)(uint8_t);
    uint8_t timer_type;
} g_timer[TIMER_END]; 

#endif