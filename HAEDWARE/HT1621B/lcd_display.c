#include "lcd_display.h"
#include "ht1621b.h"
#include "delay.h"
#include "key.h"
#include "uart.h"

#define COM1 0X01
#define COM2 0X02
#define COM3 0X04
#define COM4 0X08
#define COM_OFF 0x00

u8 Blink_std = 0;
u8 cali_display_std = 0;
void lcd_ht1621b_on ( void )
{
	ht1621_send_cmd ( LCD_ON );
}

void lcd_ht1621b_off ( void )
{
	ht1621_send_cmd ( LCD_OFF );
}

void ht1621_all_display ( void )
{
	u8 i = 0;

	for ( i = 0; i < 32; ++i ) //max seg 31
	{
		ht1621_write_byte ( i,0x0F ); //��1Ϊ���� comȫ��
	}
	// release_ht1621b_pin();
}

void ht1621_all_clear ( void )
{
	u8 i = 0;

	for ( i = 0; i < 32; ++i ) //max seg 31
	{
		ht1621_write_byte ( i,0x00 ); //��1Ϊ���� comȫ��
	}
	 //release_ht1621b_pin();
}

void init_lcd_ht1621b ( void )
{
	ht1621_init_pin();
	
	ht1621_send_cmd ( BIAS );
	ht1621_send_cmd ( RC256 ); //RC256  XTAL_32K
	ht1621_send_cmd ( WDTDIS );
	ht1621_send_cmd ( TMDIS );
	ht1621_send_cmd ( SYSEN );
	ht1621_send_cmd ( LCD_ON );
	ht1621_all_clear();
	ht1621_all_display();
}

void lcd_turn_off_time ( void )
{
	ht1621_write_byte ( 28,COM_OFF );
	ht1621_write_byte ( 31,COM_OFF );
}

//void lcd_display_time(u8 time_type)
//{
//	ht1621_write_byte(31,COM_OFF);  //old LCD
//	ht1621_write_byte(28,COM_OFF);
//	switch(time_type)
// 	{
// 		case TIMER_ON: 		ht1621_write_byte(28,COM4);break;//ѡ��com ѡ��SEG����  28ΪSEG28 COM4 0X08 ��ʾON
// 		case TIMER_0_5H: 	ht1621_write_byte(31,COM2);break;
// 		case TIMER_1H: 		ht1621_write_byte(31,COM3);break;
// 		case TIMER_2H: 		ht1621_write_byte(31,COM4);break;
// 		case TIMER_4H: 		ht1621_write_byte(28,COM2);break;
// 		case TIMER_8H: 		ht1621_write_byte(28,COM3);break;
// 	}
//}


//void lcd_display_time ( u8 time_type )
//{
//	ht1621_write_byte ( 31,COM_OFF );
//	ht1621_write_byte ( 28,COM_OFF );
//	switch ( time_type )
//	{
//		case TIMER_ON:
//			ht1621_write_byte ( 28,COM4 );
//			break;//ѡ��com ѡ��SEG����  28ΪSEG28 COM4 0X08 ��ʾON
//		case TIMER_0_5H:
//			ht1621_write_byte ( 31,COM2 );
//			break;//0.5
//		case TIMER_1H:
//			ht1621_write_byte ( 28,COM2 );
//			break;//1h
//		case TIMER_2H:
//			ht1621_write_byte ( 31,COM3 );
//			break;//2h
//		case TIMER_4H:
//			ht1621_write_byte ( 28,COM3 );
//			break;//4h
//		case TIMER_8H:
//			ht1621_write_byte ( 31,COM4 );
//			break;//8h
//	}
//}

void lcd_display_time ( u8 time_type )
{
	ht1621_send_cmd ( LCD_ON );
	ht1621_write_byte ( 31,COM_OFF );
	ht1621_write_byte ( 28,COM_OFF );
	switch ( time_type )
	{
		case TIMER_ON:
			ht1621_write_byte ( 27,COM2|COM4 );
			ht1621_write_byte ( 28,COM2|COM4|COM3 ); //ON
		    ht1621_write_byte ( 31,COM2|COM4 );
			break;
		case TIMER_0_5H:
			ht1621_write_byte ( 27,COM2|COM4|COM1); //0.5H
			ht1621_write_byte ( 28,COM2|COM4 );
		    ht1621_write_byte ( 31,COM2|COM4 );
			break;
		case TIMER_1H:
			ht1621_write_byte ( 27,COM2|COM4|COM3 ); //1H
			ht1621_write_byte ( 28,COM2|COM4 );
		    ht1621_write_byte ( 31,COM2|COM4 );
			break;
		case TIMER_2H:
			ht1621_write_byte ( 27,COM2|COM4 );
			ht1621_write_byte ( 28,COM2|COM4 );
		    ht1621_write_byte ( 31,COM2|COM4|COM1 );//2H
			break;
		case TIMER_4H:
			ht1621_write_byte ( 27,COM2|COM4 );
			ht1621_write_byte ( 28,COM2|COM4|COM1 );//4h
		    ht1621_write_byte ( 31,COM2|COM4 );
			break;
		case TIMER_8H:
			ht1621_write_byte ( 27,COM2|COM4 );
			ht1621_write_byte ( 28,COM2|COM4 );
		    ht1621_write_byte ( 31,COM2|COM4|COM3 );//8H
			break;
	}
}

void lcd_display_gap ( u8 gap )
{
	ht1621_send_cmd ( LCD_ON );
	ht1621_write_byte ( 29,0x00 );
	ht1621_write_byte ( 30,0x00 );
	if (cali_display_std == 1)
	{	
	if (Blink_std == 0)	
	{
		ht1621_write_byte ( 29,COM2|COM3 );
		ht1621_write_byte ( 30,COM2|COM3|COM4 );
		Blink_std = 1;
	}
	else 
	{
	  	Blink_std = 0;
			ht1621_write_byte ( 28,COM_OFF );
			ht1621_write_byte ( 31,COM_OFF );
	}
 }
	else if ( gap == 1 )
	{
		ht1621_write_byte ( 29,COM2|COM3 );
	}
	else if ( gap == 2 )
	{
		ht1621_write_byte ( 29,COM1|COM2|COM4 );
		ht1621_write_byte ( 30,COM3|COM4 );
	}
	else if ( gap == 3 )
	{
		ht1621_write_byte ( 29,COM1|COM2|COM3|COM4 );
		ht1621_write_byte ( 30,COM3 );
	}
	else if ( gap == 4 )
	{
		ht1621_write_byte ( 29,COM2|COM3 );
		ht1621_write_byte ( 30,COM2|COM3 );
	}
	else if ( gap == 5 )
	{
		ht1621_write_byte ( 29,COM1|COM3|COM4 );
		ht1621_write_byte ( 30,COM2|COM3 );
	}
	else if ( gap == 6 )
	{
		ht1621_write_byte ( 29,COM1|COM3|COM4 );
		ht1621_write_byte ( 30,COM2|COM3|COM4 );
	}
	else if ( gap == 7 )
	{
		ht1621_write_byte ( 29,COM1|COM2|COM3 );
	}
	else if ( gap == 8 )
	{
		ht1621_write_byte ( 29,COM1|COM2|COM3|COM4 );
		ht1621_write_byte ( 30,COM2|COM3|COM4 );
	}
	else if ( gap == 9 )
	{
		ht1621_write_byte ( 29,COM1|COM2|COM3|COM4 );
		ht1621_write_byte ( 30,COM2|COM3 );
	}
	else if ( gap == Error_STD )
	{
		if ( Blink_std == 0 )
		{
			ht1621_write_byte ( 29,COM1 );
			ht1621_write_byte ( 30,COM2|COM3|COM4 );
			Blink_std = 1;
		}
		else
		{
			Blink_std = 0;
			ht1621_write_byte ( 28,COM_OFF );
			ht1621_write_byte ( 31,COM_OFF );
		}
	}
	
}

//
//void flick(void)
//{
//
//}