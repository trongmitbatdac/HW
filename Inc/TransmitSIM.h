#ifndef __TransmitSIM__
#define __TransmitSIM__
#include "main.h"


void CmdSIM(float Avg_Temp, float Avg_Humi, float Avg_Density, int AQIh);
void aqi(int* AQI,float* c);
void AT_Command(char* cmd);
void Init_Sim800L(void);
void HTTP_POST(void);
void Clear_Buffer(char* Buffer);
void Clear_Full_Buffer(char* Buffer);
uint8_t Count_String(char* Buffer);
#endif
