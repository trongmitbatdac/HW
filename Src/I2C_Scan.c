#include <stdio.h>
#include "I2C_Scan.h"
#include "i2c-lcd.h"

extern I2C_HandleTypeDef hi2c1;

/*void I2C_Scan(void)
{
	char buffet[20];
	int abc[128];
	uint8_t total = 0;
	HAL_StatusTypeDef result;//status
	lcd_send_cmd(0x80|0x14);
	
	for(int i = 1; i < 128;i++)//scan i2c
	{
		result = HAL_I2C_IsDeviceReady(&hi2c1,(uint16_t)i<<1,2,2); //check address ready
		if(result == HAL_OK) //0x00U = HALOK
		{
			total++;
			abc[i] = i;
		}
		else
		{
			abc[i] = 0;
		}
	}
	for(int i = 1; i < 128;i++)
	{
		if(abc[i] != 0)
		{
			sprintf(buffet,"0x%X ",i);
			lcd_send_string(buffet);
		}
		else
		{
			//do nothing
		}
	}
}
*/

/*
	char buffet[20];
	int abc[128];
	uint8_t total = 0;
	int im = 0;
void I2C_Scan(void)
{

	HAL_StatusTypeDef result;//status
	lcd_send_cmd(0x80|0x14);
	
	for(int i = 1; i < 128;i++)//scan i2c
	{
		result = HAL_I2C_IsDeviceReady(&hi2c1,(uint16_t)i<<1,2,2); //check address ready
		if(result == HAL_OK) //0x00U = HALOK
		{
			total++;
			abc[i] = i;
			im = i;
		}
		else
		{
			abc[i] = 0;
		}
	}
	for(int i = 1; i < 128;i++)
	{
		if(abc[i] != 0)
		{
			sprintf(buffet,"0x%X ",i);
			lcd_send_string(buffet);
		}
		else
		{
			//do nothing
		}
	}
}
*/
