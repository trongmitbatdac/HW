#ifndef MAIN_H
#define MAIN_H

#include "stm32f1xx_hal.h"

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hour;
	uint8_t dayofweek;
	uint8_t dayofmonth;
	uint8_t month;
	uint8_t year;
}TIME;

extern TIME time;

uint8_t decToBcd(int val);
int bcdToDec(uint8_t val);
void Set_Time(uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year);
void Get_Time(void);

#endif
