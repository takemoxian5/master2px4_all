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


 //float 有效精度就7位，113.5782881为例至少10位精度，必须double 15~16位
 typedef struct __coord_t
 {
	 double latitude;
	 double longitude;
	 double altitude;
 } coord_t;
 extern coord_t coord_gloableA;
  extern coord_t coord_gloableB;
 typedef struct __coordNed_t
 {
	 float y;	 //latitude   纬度 ，南北向为y
	 float x;	   //longitude 东西向为x
	 float z;	 // 高度
	 float dist;	//绝对距离
 } coordNed_t;




 void waypoint_test();
void polygon_set_AB(coord_t coord_A, coord_t coord_B,u8 direction);
coord_t coord_set(double latitude,double longitude);















#endif


