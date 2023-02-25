#ifndef __Read_Val_Fc__
#define __Read_Val_Fc__

#include "stm32f1xx_hal_i2c.h"
		/*function*/
		
void Delay_us(uint16_t us);
void Read_AHT21(float* Humi, float* Temp);
void Read_DHT12(float *Humi,float *Temp);
void Read_Dust(float *voltage, uint16_t *adc, float *density);
//float lonti, float lati
void Process_SD(float Temp, float Humi, float Dens,double lonti, double lati,TIME time);
void kalman_filter(float* Temp, float* Humi, float* pm);

#endif
