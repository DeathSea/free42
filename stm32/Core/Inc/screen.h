#ifndef __SCREEN_HEAD
#define __SCREEN_HEAD
#include "stm32l4xx_hal.h"

static uint8_t LCD_CMD_CLEAR = 0x56; // 0b01010110
static uint8_t LCD_CMD_UPDATE = 0x93; // 0b10010011
static uint8_t LCD_CMD_NOP = 0x00;

#define LCD_DISP_HEIGHT 240
#define LCD_DISP_WIDHT 400
#define LCD_DISP_BUFF_HEIGHT LCD_DISP_HEIGHT
#define LCD_DISP_BUFF_WIDTH 50
extern uint8_t LCD_BUFF[LCD_DISP_BUFF_HEIGHT][LCD_DISP_BUFF_WIDTH];
extern int LCD_CHANGE;

void ClearBuff(void);
void ClearLCD(void);
void OutputBuff(void);
void LCDSetBuff(const uint8_t* buff, uint8_t bytesPreLine, uint8_t x, uint8_t y, uint8_t height, uint8_t width, uint8_t reverbyte);
void LcdDispEnable();
void LcdDispDisable();
#endif
