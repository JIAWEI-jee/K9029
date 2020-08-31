#include "pwm.h"
u8 pwm_period = 0;

void set_pwm(u8 duty)
{
	u32 tep = (u32)duty*pwm_period/100;
	PWM3D = tep;
}

void pwm_init(u16 hz)
{
	P0M0 = 0xC1;                        //P00����Ϊ�������
	PWM3_MAP = 0x00;					//PWM3ӳ��P00��


	//16MHZ
	//���ڼ��� 	= 0xFF / (Fosc / PWM��Ƶϵ��)	 PWM3C	
	//			= 0xFF /(16000000 / 128)			
	// 			= 1/255 /125000
	// 			= 490HZ
	pwm_period = (31250/hz)-1;			//31250 = 4000000/128
	PWM3P = pwm_period;					//PWM����  
	//set_pwm(10);
	PWM3C = 0x97; 						//ʹ��PWM3���ر��жϣ����������ʱ��128��Ƶ 111
}

