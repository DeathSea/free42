#include "shell.h"
#include "stm32_handle.h"
#include "screen.h"

const char *shell_platform()
{
	return "STM_32 FREE42 ";
}

//	2 lines, 22 characters, 131×16 pixels
void shell_blitter(const char *bits, int bytesperline, int x, int y,
                             int width, int height)
{
	LCDSetBuff(bits, bytesperline, x, y, height, width, 1);
}

void shell_beeper(int frequency, int duration)
{
}

void shell_annunciators(int updn, int shf, int prt, int run, int g, int rad)
{
	return ;
}

int shell_wants_cpu()
{
	  return 0;
}

void shell_delay(int duration)
{
	HAL_Delay(duration);
}

void shell_request_timeout3(int delay) {}

uint4 shell_get_mem()
{
	return 9999;
}

int shell_low_battery()
{
	return 0;
}

void shell_powerdown()
{
}

int8 shell_random_seed()
{
	return 67434;
}

uint4 shell_milliseconds()
{
	return -1;
}

int shell_decimal_point()
{
	return 1;
}

void shell_print(const char *text, int length,
                 const char *bits, int bytesperline,
                 int x, int y, int width, int height)
{
}
	
// int shell_always_on(int always_on) { return 1; }
	
void shell_get_time_date(uint4 *time, uint4 *date, int *weekday)
{
    RTC_DateTypeDef rtcDataType = {0};
    RTC_TimeTypeDef rtcTimeType = {0};
	HAL_RTC_GetTime(&hrtc, &rtcTimeType, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &rtcDataType, RTC_FORMAT_BIN);

	*date = ((rtcDataType.Year + 2000) * 100 + rtcDataType.Month) * 100 + rtcDataType.Date;
	*time = rtcTimeType.Hours * 1000000 + rtcTimeType.Minutes * 10000 + rtcTimeType.Seconds * 100 + rtcTimeType.SubSeconds / 10;
	*weekday = ((rtcDataType.WeekDay == 7) ? 0 : rtcDataType.WeekDay);
}

void shell_message(const char *message)
{
}

void shell_log(const char *message) {}
