#ifndef __TIMER_H__
#define __TIMER_H__
#include "HC89F303.h"

#define overrang_time   (60*60*2)
#define Heat_Value      (60*4)
#define Length          8
#define Temperature_Value         91.0624
#define H_0_5           (60*30)
#define H_1_0           (60*60)
#define H_2_0           (60*60*2)
#define H_3_0           (60*60*3)
#define H_4_0           (60*60*4)
#define H_5_0           (60*60*5)
#define H_6_0           (60*60*6)
#define H_7_0           (60*60*7)
#define H_8_0           (60*60*8)


extern u8 over_rang_time_std;
extern u8 Gap_protect_std;
extern u8 On_stay;
extern u8 Heat_start_std;
extern u8  Open_Heat_Value;
extern u8 one_heat;

void time0_init(void);
void set_time_sec_val(u16 sec);
void set_time_sec(void);
void set_correct_time(u8 gap);

#endif

