#ifndef __LCD_DISPLAY_H__
#define __LCD_DISPLAY_H__
#include "HC89F303.h"
#include "ht1621b.h"
#include "flash.h"

#define BIAS		0X52 	//ƫ�õ�ѹ
#define	SYSDIS		0X00	//�ر�ϵͳʱ�Ӻ�ƫѹ������
#define SYSEN   	0X02	//��ϵͳʱ��
#define LCD_OFF		0X04	//�ر�ƫѹ������
#define LCD_ON		0X06	//��ƫѹ������
#define RC256 		0X30	//�ڲ�ʱ��256K
#define TONEON 		0X12 	//���������
#define TONEOFF 	0X10  	//�ر��������
#define WDTDIS 		0X0A 	//��ֹ���Ź�
#define XTAL_32K	0X28	//�ⲿ��ʱ��32k  
#define TMDIS		0x08

#define Error_STD   0xff

extern u8 cali_display_std;

void lcd_display_time(u8 time_type);
void lcd_display_gap(u8 gap);

void init_lcd_ht1621b(void);
void ht1621_all_display(void);
void ht1621_all_clear(void);
void lcd_turn_off_time(void);
void flick(void);

#endif
