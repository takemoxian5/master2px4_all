#ifndef _MISSIONAPI_H_
#define _MISSIONAPI_H_

#include "stm32f4xx.h"


void Loop_check(void);

void Duty_Loop(void);

void Inner_Loop(float);

void Outer_Loop(float);





typedef struct
{
	u8 check_flag;
	u8 err_flag;
	s16 cnt_1ms;
	s16 cnt_2ms;
	s16 cnt_5ms;
	s16 cnt_10ms;
	s16 cnt_20ms;
	s16 cnt_50ms;
	u16 time;
}loop_t;






 void waypoint_test();
// void polygon_set_AB(coord_t coord_A, coord_t coord_B,bool direction);















#endif


