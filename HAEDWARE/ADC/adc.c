#include "adc.h"
#include "delay.h"
#include "uart.h"

void adc_init(void)
{
    P0M1 = 0x01;		//P01����Ϊģ������
    P0M3 = 0x01;        //P03����Ϊģ������
	ADCC0 = 0x80;		//��ADCת����Դ			00 VDD
//											01 �ڲ�4V
//											10 �ڲ�3V
//											11 �ڲ�2V
//1�� �ڲ��ο���ѹѡ��Ϊ 2V ʱ�� VDD ��ѹ����� 2.7V��
//    �ڲ��ο���ѹѡ��Ϊ 3V ʱ�� VDD ��ѹ����� 3.5V�� 
//    �ڲ��ο���ѹѡ��Ϊ 4V ʱ�� VDD ��ѹ�����4.5V��
//2�� ϵͳ�������ģʽǰ�����齫 ADC �ο���ѹѡ��� VDD�����Խ�һ������ϵͳ���ġ�
	delay_us(20);						//��ʱ20us��ȷ��ADCϵͳ�ȶ�

//	ADCC1 = 0x01;						//ѡ��ADCͨ��1
	ADCC2 = 0x49;						//ת�����12λ���ݣ������Ҷ��룬ADCʱ��4��Ƶ 4M/4 = 1MHz
}

u16 get_adc_val(void)
{
	u16 adc_val = 0;
	ADCC1 = 0x01;
	
	ADCC0 |= 0x40;					//����ADCת��
	while(!(ADCC0&0x20));			//�ȴ�ADCת������
	ADCC0 &=~ 0x20;					//�����־λ
	adc_val = ADCR;					//��ȡADC��ֵ
      

    // ADC_printf("P01_ADC = %d \r\n",adc_val);
	return adc_val;
}

u16 get_adc_val_ch(void)
{
	u16 adc_val_ch = 0;
	ADCC1 = 0x03;                     //ѡ��ADCͨ��7
	
	ADCC0 |= 0x40;					//����ADCת��
	while(!(ADCC0&0x20));			//�ȴ�ADCת������
	ADCC0 &=~ 0x20;					//�����־λ
	adc_val_ch = ADCR;					//��ȡADC��ֵ

//    ADC_printf("P23_ADC = %d \r\n",adc_val_ch);
	return adc_val_ch;
}



void get_voltage(u16 *u1_voltage,u16 *u3_voltage)
{
  u32 temp = 0;
  u8  i = 0;
  for(i=0;i<5;i++)
  	{
      temp+=get_adc_val();
    }
   temp/=5;
  *u1_voltage = temp*5000/4096;
  temp = 0;
  for (i=0;i<5;i++)
  	{
     temp+=get_adc_val_ch();
    }
  temp/=5;
  *u3_voltage = temp*5000/4096;
}

//u16 get_voltage_u1(void)
//{
//    u32 tep = 0;
//	u8 i = 0;
//
//	for(i = 0;i < 5;i++)
//	{
//		tep += get_adc_val();
//	}
//    tep /= 5;
//    tep = tep*4000/4096;	//���0�3�0�7��1000���0�9 ���0�5�0�7�0�3�0�4�0�3�0�8�0�5�0�8�0�0�0�2���0�6�0�5�0�2�0�3 4096
//    return tep;
//}
//
//u16 get_voltage_u3(void)
//{
//    u32 tep = 0;
//	u8 i = 0;
//
//	for(i = 0;i < 5;i++)
//	{
//		tep += get_adc_val_ch();
//	}
//    tep /= 5;
//    tep = tep*4000/4096;	//���0�3�0�7��1000���0�9 ���0�5�0�7�0�3�0�4�0�3�0�8�0�5�0�8�0�0�0�2���0�6�0�5�0�2�0�3 4096
//    return tep;
//}



