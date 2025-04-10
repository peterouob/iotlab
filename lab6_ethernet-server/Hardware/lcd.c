#include "lcd.h"
#include "lcd_init.h"
#include "lcdfont.h"

/****************************
purpose:fill in the color you want in a specific area 

xsta -> start x coordinate
ysta -> start y coordinate
xend -> end x coordinate
yend -> end y coordinate
color -> hex code
****************************/
void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{          
	//set area which you want to fill 
	LCD_Address_Set(xsta, ysta, xend-1, yend-1);
	for(uint16_t i = ysta;i < yend;i ++)
	{													   	 	
		for(uint16_t j = xsta;j < xend;j ++)
		{
			LCD_WR_DATA(color);
		}
	} 					  	    
}

/****************************
purpose:draw a point on the screen

x -> x coordinate
y -> y coordinate
color -> hex code
****************************/
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
	LCD_Address_Set(x,y,x,y);
	LCD_WR_DATA(color);
} 

/****************************
purpose:draw a single word

x -> start x coordinate
y -> start y coordinate
fc -> word color
bc -> word background color
sizey -> height of single word
mode = 1 -> overlay mode -> just refresh specific area

NOTICE:
word width = height /2
****************************/
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
	uint8_t temp,sizex,t,m=0;
	uint16_t i,TypefaceNum;
	uint16_t x0=x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';
	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);
	for(i=0;i<TypefaceNum;i++)
	{ 
		if(sizey==12)temp=ascii_1206[num][i];
		else if(sizey==16)temp=ascii_1608[num][i];
		else if(sizey==24)temp=ascii_2412[num][i]; 
		else if(sizey==32)temp=ascii_3216[num][i];
		else return;
		for(t=0;t<8;t++)
		{
			if(!mode)
			{
				if(temp&(0x01<<t))LCD_WR_DATA(fc);
				else LCD_WR_DATA(bc);
				m++;
				if(m%sizex==0)
				{
					m=0;
					break;
				}
			}
			else
			{
				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}   	 	  
}

/****************************
purpose:draw a string(sequence char)

x -> start x coordinate
y -> start y coordinate
*p -> string
fc -> word color
bc -> word background color
sizey -> height of single word
mode = 1 -> overlay mode -> just refresh specific area

NOTICE:
1. use while to call LCD_ShowChar() until reach null character('\0')
2. sizey only can use 12, 16, 24, 32
****************************/
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{         
	while(*p!='\0')
	{       
		LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
		x+=sizey/2;
		p++;
	}  
}

/****************************
purpose:draw a picture

x -> start x coordinate
y -> start y coordinate
length -> picture height
width -> picture width
pic -> picture array
NOTICE:
1. Image2Lcd setting
	output gray -> 16bits true color 
	max width & height -> 240*240
	V content header
	V MSB 
	R:5bits, G:6bits, B:5bits
****************************/
void LCD_ShowPicture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[])
{
	uint32_t k = 0;
	LCD_Address_Set(x, y, x+length-1, y+width-1);
	for(uint16_t i = 0;i < length;i ++)
	{
		for(uint16_t j = 0;j < width; j++)
		{
			LCD_WR_DATA8(pic[k*2]);
			LCD_WR_DATA8(pic[k*2+1]);
			k++;
		}
	}			
}