#include "ht1621b.h"
#include "delay.h"
#include "intrins.h"//nop


void ht1621_init_pin(void)
{
	P1M1 = 0XC1;//����Ϊ�������
	P1M0 = 0XC1;//����Ϊ�������
	P0M7 = 0XC1;//����Ϊ�������
	P0M6 = 0XC1;//����Ϊ�������

	LCD_CS = 1;
	LCD_RD = 1;
	LCD_WR = 1;
	LCD_DATA = 1;
}

//��λ��ǰ
void ht1621_sendbit_hight(u8 dat,u8 cnt)
{
	u8 i = 0;

	for (i = 0; i < cnt; ++i)
	{
		LCD_WR = 0;
		if((dat & 0x80) == 0)
			LCD_DATA = 0;
		else 
			LCD_DATA = 1;
		LCD_WR = 1;
		lcd_delay_us(5);
		dat <<= 1;
	}
}
//��λ��ǰ
void ht1621_sendbit_low(u8 dat,u8 cnt)
{
	u8 i = 0;

	for (i = 0; i < cnt; ++i)
	{
		LCD_WR = 0;
		if((dat & 0x01) == 0)
			LCD_DATA = 0;
		else	
			LCD_DATA = 1;
		LCD_WR = 1;
		lcd_delay_us(5);
		dat >>= 1;	
	}
}


void ht1621_send_cmd(u8 command)
{
	LCD_CS = 0;
	ht1621_sendbit_hight(0x80,4);  //д���־λ100  д����  û�и���ʱ���������command���λ��0
	ht1621_sendbit_hight(command,8);
	LCD_CS = 1;
}

void ht1621_write_byte(u8 addr, u8 dat)
{
	LCD_CS = 0;
	ht1621_sendbit_hight(0xA0,3);//101   д���ݱ�־λ
	ht1621_sendbit_hight(addr << 2,6);//ram  ��6λ  SEG���
	ht1621_sendbit_low(dat,4);//data ��4λ  COM
	LCD_CS = 1;
}

void release_ht1621b_pin(void)
{
	LCD_CS = 1;
	LCD_RD = 1;
	LCD_WR = 1;
	LCD_DATA = 1;
}





