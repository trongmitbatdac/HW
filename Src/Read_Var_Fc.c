#include <stdio.h>
#include "main.h"
#include "fatfs_sd.h"
#include "ff.h"
#include "time.h"
#include "i2c-lcd.h"
#include "Read_Var_Fc.h"


extern I2C_HandleTypeDef hi2c1; 
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c2;

//#define DHT12_ADD 0x5C
#define AHT21_ADD (0x38<<1)

/*               SD commun                */
FATFS fs;  // file system
FIL fil; // File
FRESULT fresult;  // result
FILINFO fno;

/*****************************************************
*Function name: kalman_filter
 *Function function: ADC_filter
 *Entry parameter: ADC_Value
 *Export parameters: kalman_adc
*****************************************************/
/*
float kalman_filter(float ADC_Value)
{
    float x_k1_k1,x_k_k1;
    static float ADC_OLD_Value;
    float Z_k;
    static float P_k1_k1;

    static float Q = 0.0001;//Q: Regulation noise, Q increases, dynamic response becomes faster, and convergence stability becomes worse Q = 0.0001
    static float R = 0.005; //R: Test noise, R increases, dynamic response becomes slower, convergence stability becomes better R = 0.005;
    static float Kg = 0;
    static float P_k_k1 = 1000;

    float kalman_adc;
    static float kalman_adc_old=0;
    Z_k = ADC_Value;
    x_k1_k1 = kalman_adc_old;

    x_k_k1 = x_k1_k1;
    P_k_k1 = P_k1_k1 + Q;

    Kg = P_k_k1/(P_k_k1 + R);

    kalman_adc = x_k_k1 + Kg * (Z_k - kalman_adc_old);
    P_k1_k1 = (1 - Kg)*P_k_k1;
    P_k_k1 = P_k1_k1;

    ADC_OLD_Value = ADC_Value;
    kalman_adc_old = kalman_adc;

    return kalman_adc;
}
*/
void kalman_filter(float* Temp, float* Humi, float* pm)
{
    float x_k1_k1_t, x_k_k1_t;
    static float ADC_OLD_Value_t;
    float Z_k_t;
    static float P_k1_k1_t;

    static float Q_t = 0.0001;//Q: Regulation noise, Q increases, dynamic response becomes faster, and convergence stability becomes worse Q = 0.0001
    static float R_t = 0.005; //R: Test noise, R increases, dynamic response becomes slower, convergence stability becomes better R = 0.005;
    static float Kg_t = 0;
    static float P_k_k1_t = 1000;

    float kalman_adc_t;
    static float kalman_adc_old_t=0;
    Z_k_t = *Temp;
    x_k1_k1_t = kalman_adc_old_t;

    x_k_k1_t = x_k1_k1_t;
    P_k_k1_t = P_k1_k1_t + Q_t;

    Kg_t = P_k_k1_t/(P_k_k1_t + R_t);

    kalman_adc_t = x_k_k1_t + Kg_t * (Z_k_t - kalman_adc_old_t);
    P_k1_k1_t = (1 - Kg_t)*P_k_k1_t;
    P_k_k1_t = P_k1_k1_t;

    ADC_OLD_Value_t = *Temp;
    kalman_adc_old_t = kalman_adc_t;
		/************************************************************/
		
		float x_k1_k1_h, x_k_k1_h;
    static float ADC_OLD_Value_h;
    float Z_k_h;
    static float P_k1_k1_h;

    static float Q_h = 0.0001;//Q: Regulation noise, Q increases, dynamic response becomes faster, and convergence stability becomes worse Q = 0.0001
    static float R_h = 0.005; //R: Test noise, R increases, dynamic response becomes slower, convergence stability becomes better R = 0.005;
    static float Kg_h = 0;
    static float P_k_k1_h = 1000;

    float kalman_adc_h;
    static float kalman_adc_old_h=0;
    Z_k_h = *Humi;
    x_k1_k1_h = kalman_adc_old_h;

    x_k_k1_h = x_k1_k1_h;
    P_k_k1_h = P_k1_k1_h + Q_h;

    Kg_h = P_k_k1_h/(P_k_k1_h + R_h);

    kalman_adc_h = x_k_k1_h + Kg_h * (Z_k_h - kalman_adc_old_h);
    P_k1_k1_h = (1 - Kg_h)*P_k_k1_h;
    P_k_k1_h = P_k1_k1_h;

    ADC_OLD_Value_h = *Humi;
    kalman_adc_old_h = kalman_adc_h;
		/**************************************************************/
		
		float x_k1_k1_p, x_k_k1_p;
    static float ADC_OLD_Value_p;
    float Z_k_p;
    static float P_k1_k1_p;

    static float Q_p = 0.0001;//Q: Regulation noise, Q increases, dynamic response becomes faster, and convergence stability becomes worse Q = 0.0001
    static float R_p = 0.005; //R: Test noise, R increases, dynamic response becomes slower, convergence stability becomes better R = 0.005;
    static float Kg_p = 0;
    static float P_k_k1_p = 1000;

    float kalman_adc_p;
    static float kalman_adc_old_p=0;
    Z_k_p = *pm;
    x_k1_k1_p = kalman_adc_old_p;

    x_k_k1_p = x_k1_k1_p;
    P_k_k1_p = P_k1_k1_p + Q_p;

    Kg_p = P_k_k1_p/(P_k_k1_p + R_p);

    kalman_adc_p = x_k_k1_p + Kg_p * (Z_k_p - kalman_adc_old_p);
    P_k1_k1_p = (1 - Kg_p)*P_k_k1_p;
    P_k_k1_p = P_k1_k1_p;

    ADC_OLD_Value_p = *pm;
    kalman_adc_old_p = kalman_adc_p;
		/*****************************************************************/
}

void Delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0); // Timer setup counter = 0
	while(__HAL_TIM_GET_COUNTER(&htim1) < us); // waiting
}

void Read_AHT21(float* Humi, float* Temp)
{
	uint8_t AHT_Cmd[3] = {0xAC,0x33,0x00};
	uint8_t Rx_data[7];
	uint32_t ADC_Raw;
	
	HAL_Delay(40);
	HAL_I2C_Master_Transmit(&hi2c1, AHT21_ADD, (uint8_t*)AHT_Cmd, 3, 1000);
	HAL_Delay(80);
	HAL_I2C_Master_Receive(&hi2c1, AHT21_ADD, (uint8_t*)Rx_data, 7, 500);
	
	if(~Rx_data[0] & 0x80)
	{
		ADC_Raw = (((uint32_t)Rx_data[3]&15)<< 16 | (uint32_t)Rx_data[4]<<8 | Rx_data[5]); // 1000 0000 0000 0000 0000 0000
		*Temp = (float)(ADC_Raw * 200.00 / 1048576.00) - 50.00;
		
		ADC_Raw = ((uint32_t)Rx_data[1] << 12) | (uint32_t)Rx_data[2]<<4 | Rx_data[3]>>4;
		*Humi = (float)(ADC_Raw * 100.00 / 1048576.00);
		
		//CRC_data = (uint8_t)Rx_data[6];
	}
}
	
void Read_DHT12(float *Humi,float *Temp)
{
	uint8_t Temp_Humi[5];
	HAL_I2C_Mem_Read( &hi2c1 , AHT21_ADD << 1,0x00, I2C_MEMADD_SIZE_8BIT , Temp_Humi, 5, 2000); //read dht12
	//HAL_I2C_Mem_Read( &hi2c1 , DHT12_ADD << 1,0x00, I2C_MEMADD_SIZE_8BIT , Temp_Humi, 5, 2000); //read dht12
	if(Temp_Humi[4] == Temp_Humi[0] + Temp_Humi[1] + Temp_Humi[2] + Temp_Humi[3])
		{
			*Humi = (float)(Temp_Humi[0] + (float)Temp_Humi[1]/10);
			*Temp = (float)(Temp_Humi[2] + (float)(Temp_Humi[3]&0x7f)/10); //0x7f = 0111 1111
			
			if(Temp_Humi[3]&0x80) //0x80 = check bit ex: 1000 1111 & 1000 0000 = 1000 0000
			{
				*Temp = 0 - *Temp;
			}
			else
			{
				//do nothing
			}
		}
		else
		{
			/* do no thing */
		}	
}


void Read_Dust(float *voltage, uint16_t *adc, float *density)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_RESET); //set on led gp2y1010au0f
	Delay_us(280); //delay 280 us
	HAL_ADC_Start(&hadc1); // adc
	*adc = HAL_ADC_GetValue(&hadc1);//read adc value
	Delay_us(40);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET); // set off led
	Delay_us(9680);
	*voltage = ((float)(*adc)*3.3/4095)*(5/3.3); //voltage value
	//*density = 149.25*(*voltage)-44.78; // 3.65V and 0.3V
	//*density = 160*(*voltage)-68; // 3.55V and 0.425V
	//*density = ((*voltage - 0.45)/0.5)*100;
	//*density = 156.25*(*voltage)-46.875; // 3.5V and 0.3V
	//*density = 151.52*(*voltage)-30.3;  //3.5 and 0.2V
	//*density = 147.1*(*voltage)-14.71;  //3.5 and 0.1V
	//*density = 185.2*(*voltage)-55.6;  //3.3 and 0.3V
	//*density = 200*(*voltage)-100;  //3 and 0.5V
	//*density = 192.31*(*voltage)-76.9;  //3 and 0.4V
	//*density = (0.5/(3.4-0.5))*(*voltage - 0.5)*1000;  //3 and 0.5V
	*density = (0.5/(3.5-0.5))*(*voltage - 0.5)*1000;  //3.5V and 0.5V
	//*density = (0.5/(3.2-0.45))*(*voltage - 0.45)*1000;  //3.2V and 0.45V
	if(*density < 0)
	{
		*density = 0;
	}
	else if(*density > 500)
	{
		*density = 500;
	}
	else
	{
		//do nothing
	}

}
//,float lonti, float lati,
void Process_SD(float Temp, float Humi, float Dens,double lonti, double lati,TIME time)
{
	char Buffer_LCDer_SD[128];
	fresult = f_mount(&fs,"/",1);
	fresult = f_open(&fil, "data.txt", FA_OPEN_EXISTING | FA_WRITE);
	if(fresult == FR_NO_FILE)
	{
		f_open(&fil,"data.txt",FA_CREATE_ALWAYS);
	}
	f_lseek(&fil, f_size(&fil));
	
	sprintf(Buffer_LCDer_SD,"%2d-%2d-%2d %2d:%2d:%2d,",time.year,time.month,time.dayofmonth,time.hour,time.minutes,time.seconds);//Thoi gian: 
	f_puts(Buffer_LCDer_SD,&fil);
	Clear_Buffer_LCDer(Buffer_LCDer_SD);
	
	sprintf(Buffer_LCDer_SD,"%.2f,",Temp);//Nhiet do: 
	f_puts(Buffer_LCDer_SD,&fil);
	Clear_Buffer_LCDer(Buffer_LCDer_SD);

	sprintf(Buffer_LCDer_SD,"%.2f,",Humi);//Do am:
	f_puts(Buffer_LCDer_SD,&fil);
	Clear_Buffer_LCDer(Buffer_LCDer_SD);
	
	sprintf(Buffer_LCDer_SD,"%3.2f,",Dens);//Nong do bui: 
	f_puts(Buffer_LCDer_SD,&fil);
	Clear_Buffer_LCDer(Buffer_LCDer_SD);
	
	sprintf(Buffer_LCDer_SD,"%f,",lonti);//kinh do
	f_puts(Buffer_LCDer_SD,&fil);
	Clear_Buffer_LCDer(Buffer_LCDer_SD);
	
	sprintf(Buffer_LCDer_SD,"%f,/n",lati);//vi do
	f_puts(Buffer_LCDer_SD,&fil);
	Clear_Buffer_LCDer(Buffer_LCDer_SD);
	
	
	f_close(&fil);
}
