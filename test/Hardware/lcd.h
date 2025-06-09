#ifndef __LCD_H
#define __LCD_H		
#include "stm32f4xx.h"

void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color);
void LCD_ShowPicture(uint16_t x,uint16_t y,uint16_t length,uint16_t width,const uint8_t pic[]);
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color);
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);

#define WHITE 	0xFFFF
#define BLACK		0x0000	  
#define BLUE		0x001F 
#define GRAY		0X8430

#endif