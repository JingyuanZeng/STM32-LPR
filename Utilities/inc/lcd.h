/* ��ֹ�ݹ������ͷ�ļ� ------------------------------------------------------*/
#ifndef __LCD_H
#define __LCD_H
	 
#include "stm32f10x.h"
#include "lcd.h"
	 
void Lcd_Gpio_Init(void);
void LCD_Writ_Bus(u16 bus_data);
void LCD_Write_COM(u16 bus_data);
void LCD_Write_DATA(u16 bus_data);
//void Image();//����ͼƬ1
void LCD_Fast_clear(u16 color);
void LCD_Init(void);
void Pant(int dcolor);
void LCD_Fill(unsigned short color);
void LCD_DrawPoint(u16 x,u16 y,u16 color);//��ָ��λ��д��һ�����ص�����
void LCD_ShowChar(u16 x,u16 y, u8 num,u8 mode);
void LCD_ShowNum(u16 x,u16 y,u32 num,u8 len);
void LCD_ShowNumPoint(u16 x,u16 y,u16 num);//��ʾ4λ��+2λС����
#ifdef __cplusplus
}
#endif


#endif /* __LTK_GPIO_H */