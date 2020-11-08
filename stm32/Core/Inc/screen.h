#ifndef __SCREEN_HEAD
#define __SCREEN_HEAD
#include "stm32l4xx_hal.h"

static uint8_t LCD_CMD_CLEAR = 0x56; // 0b01010110
static uint8_t LCD_CMD_UPDATE = 0x93; // 0b10010011
static uint8_t LCD_CMD_NOP = 0x00;

extern uint8_t LCD_BUFF[240][50];

void ClearBuff(void);
void ClearLCD(void);
void OutputBuff(void);

#endif
