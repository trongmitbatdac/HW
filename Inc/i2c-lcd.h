#ifndef __LCD_I2C__
#define __LCD_I2C__

#include "stm32f1xx_hal.h"

void lcd_init (void);   // initialize lcd
void lcd_clear(void);
void lcd_send_cmd (char cmd);  // send command to the lcd
void lcd_send_data (char data);  // send data to the lcd
void lcd_send_string (char *str);  // send string to the lcd
void lcd_Clr(void); // clr display cmd
void lcd_put_cur(int row, int col);
void Clear_Buffer_LCDer(char* Buffer_LCD); // clear Buffer lcd and micro sd

#endif
