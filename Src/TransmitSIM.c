#include <stdio.h>
#include "string.h"
#include "TransmitSIM.h"
#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

char Buff[128] = {0};
char Value[128] = {0};
uint8_t test[] = "OK";
uint8_t CRLF[] = "\r\n";
uint8_t SUB[] = "x1A";

float T_Temperature = 0;
float T_Humidity = 0;
float T_Density = 0;
double T_Lonti = 0;
double T_Lati = 0;
uint16_t T_AQI = 0;


/* SIM 800 series*/
//uint8_t AT[] = "AT\r\n";
//uint8_t CipStart[] = "AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\"\r\n";
//uint8_t CipSend[] = "AT+CIPSEND\r\n";
//uint8_t GetAPI[] = "GET /update?api_key=O3G8MR3353CV4H2M&field1=";
//uint8_t CloseIP[] = "AT+CIPCLOSE\r\n";
//sprintf(BUFF,"GET /update?api_key=2VWY9IUPL1SJJ8LE&field1=%.2f&field2=%.2f&field3=%3.2f&field4=%d HTTP/1.1\r\nHost: api.thingspeak.com\r\nConnection\r\n\r\n\x1A",Avg_Temp,Avg_Humi,Avg_Density,AQIh);
//HAL_UART_Transmit_IT(&huart1,(uint8_t*)BUFF, 256);
//HAL_UART_Transmit_IT(&huart1,CloseIP,sizeof(CloseIP));
//uint8_t MinimumMode[] = "AT+CFUN=0\r\n";
//uint8_t CIPSHUT[] = "AT+CIPSHUT\r\n";
//uint8_t OnlineMode[] = "AT+CFUN=1\r\n";

//For 4G IOT A7670C LASS
/*
uint8_t AT[] = "AT\r\n";
uint8_t NetOpen[] ="AT+NETOPEN\r\n";
uint8_t CipOpen[] = "AT+CIPOPEN=0,\"TCP\",\"thingspeak.com\",80\r\n";
uint8_t CipSend[] = "AT+CIPSEND=0\r\n";
uint8_t APIKey[] = "O3G8MR3353CV4H2M";
uint8_t GetUpdate[] = "GET /update?api_key=";
uint8_t SubCmd[] = "\r\n\x1A\x1A";
uint8_t NetClose[] = "AT+NETCLOSE\r\n";
uint8_t MinimumMode[] = "AT+CFUN=0\r\n";
uint8_t OnlineMode[] = "AT+CFUN=1\r\n";
//Upcomming
*/

uint8_t Count_String(char* Buffer)
{
	uint8_t j = 0;
	
	while(Buffer[j] != '\0')
	{
		j++;
	}
	
	return j;
}	

void Clear_Buffer(char* Buffer)
{
	for(int i = 0; i < Count_String(Buffer); i++)
	{
		Buffer[i] = 0;
	}
}

void Clear_Full_Buffer(char* Buffer)
{
		for(int i = 0; i < 128; i++)
	{
		Buffer[i] = 0;
	}
}

void AT_Command(char* cmd)
{
	Clear_Buffer(Buff);
	HAL_UART_Transmit(&huart1, (uint8_t*)cmd, Count_String(cmd) , 500);
	HAL_UART_Receive(&huart1, (uint8_t*)Buff, 128, 1000);
	HAL_Delay(500);
	
}

// Compatible for all of Sim module
void Init_Sim800L(void)
{
	uint8_t ATisOK = 0;
	uint8_t CFUNisFULLMODE = 0;
	uint8_t FalseCount = 0;
	while(!ATisOK)
	{
		if(strstr((char*)Buff,"OK"))
		{
			ATisOK = 1;
			FalseCount = 0;
			Clear_Full_Buffer(Buff);
		}
		else
		{
			if(FalseCount == 20)
			{
				ATisOK = 1;
				FalseCount = 0;
			}
			else
			{
				AT_Command("AT\r\n");
				FalseCount++;
			}
		}
	}
	
	while(!CFUNisFULLMODE)
	{
		if(strstr((char*)Buff,"1"))
		{
			CFUNisFULLMODE = 1;
			AT_Command("AT+CFUN=0\r\n");
			FalseCount = 0;
			Clear_Full_Buffer(Buff);
		}
		else if(strstr((char*)Buff,"0"))
		{
			CFUNisFULLMODE = 1;
			FalseCount = 0;
			Clear_Full_Buffer(Buff);
		}
		else
		{
			if(FalseCount == 20)
			{
				CFUNisFULLMODE = 1;
				FalseCount = 0;
			}
			else
			{
				Clear_Full_Buffer(Buff);
				AT_Command("AT+CFUN?\r\n");
				FalseCount++;
			}
		}
	}
}

// for Sim7020E Nb-iot
// For 800L

void HTTP_POST(void)
{
	uint8_t ATisOK = 0;
	uint8_t CFUNisFullMode = 0;
	uint8_t CIPMUXisOK = 0;
  uint8_t CIPSTARTisOK = 0;
  uint8_t CIPSENDisOK = 0;
  
	uint8_t SENDisOK = 0;
	
	int FalseCount = 0; // if false = 20 -> error while send data
	
	while(!ATisOK)
	{
		if(strstr((char*)Buff,"OK"))
		{
			ATisOK = 1;
			Clear_Full_Buffer(Buff);
			FalseCount = 0;
		}
		else
		{
			AT_Command("AT\r\n");
			FalseCount++;
			if(FalseCount == 20)
			{
				ATisOK = 1;
				FalseCount = 0;
			}
			else
			{
				//do no thing
			}
		}
	}
	
	//AT_Command("AT+CFUN?\r\n");
	while(!CFUNisFullMode)
	{
		if(strstr((char*)Buff,"CFUN: 1"))
		{
			CFUNisFullMode = 1;
			Clear_Full_Buffer(Buff);
			FalseCount = 0;
		}
		else if(strstr((char*)Buff,"CFUN: 0"))
		{
			Clear_Full_Buffer(Buff);
			AT_Command("AT+CFUN=1\r\n");
			HAL_Delay(5000);
		}
		else
		{
			Clear_Full_Buffer(Buff);
			AT_Command("AT+CFUN?\r\n");
			FalseCount++;
			if(FalseCount == 20)
			{
				CFUNisFullMode = 1;
				FalseCount = 0;
			}
			else
			{
				//do nothing
			}
		}
	}
	
	AT_Command("AT+CIPMUX?\r\n");
	while(!CIPMUXisOK)
	{
		if(strstr((char*)Buff,"0"))
		{
			CIPMUXisOK = 1;
			Clear_Full_Buffer(Buff);
			FalseCount = 0;
		}
		else
		{
			Clear_Full_Buffer(Buff);
			AT_Command("AT+CIPMUX=0\r\n");
			FalseCount++;
			if(FalseCount == 20)
			{
				CIPMUXisOK = 1;
				FalseCount = 0;
			}
			else
			{
				//do nothing
			}
		}
	}
	
	AT_Command("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\"\r\n");
	while(!CIPSTARTisOK)
	{
		if(strstr((char*)Buff,"CONNECT OK") || strstr((char*)Buff,"ALREADY CONNECT") || strstr((char*)Buff,"OK"))
		{
			CIPSTARTisOK = 1;
			Clear_Full_Buffer(Buff);
			FalseCount = 0;
		}
		else
		{
			//AT_Command("AT+CIPSHUT\r\n");
			AT_Command("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\"\r\n");
			FalseCount++;
			if(FalseCount == 20)
			{
				CIPSTARTisOK = 1;
				FalseCount = 0;
			}
			else
			{
				//do nothing
			}
		}
	}
	
	while(!CIPSENDisOK)
	{
		if(strstr((char*)Buff,">"))
		{
			CIPSENDisOK = 1;
			Clear_Full_Buffer(Buff);
			FalseCount = 0;
		}
		else
		{
			//AT_Command("AT+CIPSHUT\r\n");
			//Clear_Full_Buffer(Buff);
			AT_Command("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\"\r\n");
			Clear_Full_Buffer(Buff);
			AT_Command("AT+CIPSEND\r\n");
			
			FalseCount++;
			if(FalseCount == 20)
			{
				CIPSENDisOK = 1;
				FalseCount = 0;
			}
			else
			{
				//do nothing
			}
		}
	}
	Clear_Full_Buffer(Value);
	sprintf((char*)Value,"&field1=%.2f&field2=%.2f&field3=%3.2f&field4=%d&field5=%f&field6=%f\r\n\x1A", T_Temperature, T_Humidity, T_Density,T_AQI,T_Lonti,T_Lati);
	AT_Command("GET /update?api_key=2VWY9IUPL1SJJ8LE");
	AT_Command(Value);
	while(!SENDisOK)
	{
		if(strstr((char*)Buff,"SEND OK"))
		{
			SENDisOK = 1;
			Clear_Full_Buffer(Buff);
			FalseCount = 0;
		}
		else
		{
			Clear_Full_Buffer(Buff);
			Clear_Full_Buffer(Value);
			sprintf((char*)Value,"&field1=%.2f&field2=%.2f&field3=%3.2f&field4=%d\r\n\x1A", T_Temperature, T_Humidity, T_Density,T_AQI);
			AT_Command("GET /update?api_key=2VWY9IUPL1SJJ8LE");
			AT_Command(Value);
			FalseCount++;
			if(FalseCount == 20)
			{
				SENDisOK = 1;
				FalseCount = 0;
			}
			else
			{
				//do nothing
			}
		}
	}
	
	AT_Command("AT+CIPCLOSE\r\n");
	
	AT_Command("AT+CIPSHUT\r\n");
	
	while(!CFUNisFullMode)
	{
		if(strstr((char*)Buff,"1"))
		{
			CFUNisFullMode = 1;
			AT_Command("AT+CFUN=0\r\n");
			FalseCount = 0;
			Clear_Full_Buffer(Buff);
		}
		else if(strstr((char*)Buff,"0"))
		{
			CFUNisFullMode = 1;
			FalseCount = 0;
			Clear_Full_Buffer(Buff);
		}
		else
		{
			if(FalseCount == 20)
			{
				CFUNisFullMode = 1;
				FalseCount = 0;
			}
			else
			{
				Clear_Full_Buffer(Buff);
				AT_Command("AT+CFUN?\r\n");
				FalseCount++;
			}
		}
	}
	Clear_Full_Buffer(Buff);
	
}


void CmdSIM(float Avg_Temp, float Avg_Humi, float Avg_Density, int AQIh)
{
	//char Data[100];
	//int CharCount = 0;
	//char BUFF[256]= {0};
	//sprintf(Data,"&field1=%.2f&field2=%.2f&field3=%.2f&field4=%d",Avg_Temp,Avg_Humi,Avg_Density,AQIh);
	//while(Data[CharCount] != '\0')
  //{
		//CharCount++;
  //}
	/*
		sprintf(BUFF,"GET /update?api_key=2VWY9IUPL1SJJ8LE&field1=%.2f&field2=%.2f&field3=%3.2f&field4=%d HTTP/1.1\r\nHost: api.thingspeak.com\r\nConnection\r\n\r\n\x1A",Avg_Temp,Avg_Humi,Avg_Density,AQIh);
		HAL_UART_Transmit_IT(&huart1,MinimumMode,sizeof(MinimumMode));
		HAL_Delay(1000);
		HAL_UART_Transmit_IT(&huart1,OnlineMode,sizeof(OnlineMode));
		HAL_Delay(3000);
		HAL_UART_Transmit_IT(&huart1,CIPSHUT,sizeof(CIPSHUT));
		HAL_Delay(2000);
		HAL_UART_Transmit_IT(&huart1,AT,sizeof(AT));
		HAL_Delay(500);
		HAL_UART_Transmit_IT(&huart1,CipStart,sizeof(CipStart));
		HAL_Delay(2000);
		HAL_UART_Transmit_IT(&huart1,CipSend, sizeof(CipSend));
		HAL_Delay(300);
		
		HAL_UART_Transmit_IT(&huart1,(uint8_t*)BUFF, 256);
		//HAL_UART_Transmit_IT(&huart1,GET, sizeof(GET));
		HAL_Delay(5000);
		HAL_UART_Transmit_IT(&huart1,CloseIP,sizeof(CloseIP));
		HAL_Delay(300);
		HAL_UART_Transmit_IT(&huart1,MinimumMode,sizeof(MinimumMode));
		*/
		/*
		HAL_UART_Transmit_IT(&huart1,MinimumMode,sizeof(MinimumMode));
		HAL_Delay(3000);
		HAL_UART_Transmit_IT(&huart1,OnlineMode,sizeof(OnlineMode));
		HAL_Delay(3000);
		HAL_UART_Transmit_IT(&huart1,AT,sizeof(AT));
		HAL_Delay(500);
		HAL_UART_Transmit_IT(&huart1,NetOpen,sizeof(NetOpen));
		HAL_Delay(500);
		HAL_UART_Transmit_IT(&huart1,CipOpen,sizeof(CipOpen));
		HAL_Delay(500);
		HAL_UART_Transmit_IT(&huart1,CipSend,sizeof(CipSend));
		HAL_Delay(500);
		HAL_UART_Transmit_IT(&huart1,GetUpdate,sizeof(GetUpdate));
		HAL_UART_Transmit_IT(&huart1,APIKey,sizeof(APIKey));
		HAL_UART_Transmit_IT(&huart1,(uint8_t*)Data,CharCount);
		HAL_UART_Transmit_IT(&huart1,SubCmd,sizeof(SubCmd));
		HAL_Delay(3000);
		HAL_UART_Transmit_IT(&huart1,NetClose,sizeof(NetClose));
		HAL_Delay(500);
		HAL_UART_Transmit_IT(&huart1,MinimumMode,sizeof(MinimumMode));
		*/
}

void aqi(int* AQI,float* c)
{
	float nowcast = 0;
	float w = 0;
	
	if(c[1] > c[0])
	{
		w = c[0]/(c[1]);
	}
	else
	{
		w = c[1]/(c[0]);
	}
	
	if(w <= 1/2)
	{
		w = 1/2;
		nowcast = (1/2)*(c[0])+(1/4)*(c[1]);
	}
	else
	{
		nowcast = ((c[0]) + (w)*(c[1]))/(1+(w));
	}
	
	*AQI = 2*(nowcast);
	
}
