#include "usart.h"	
#include "MissionAPI.h"	
s16 loop_cnt;


loop_t loop;

void Loop_check()  //TIME INTTERRUPT
{
	loop.time++; //u16
	loop.cnt_2ms++;
	loop.cnt_5ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;

	if( loop.check_flag == 1)
	{
		loop.err_flag ++;     //每累加一次，证明代码在预定周期内没有跑完。
	}
	else
	{	
		loop.check_flag = 1;	//该标志位在循环的最后被清零
	}
}

void Duty_1ms()
{
	Get_Cycle_T(1);
//	LED_Display( LED_Brightness );								//20级led渐变显示
//	ANO_DT_Data_Exchange();												//数传通信定时调用
}

float test[5];
void Duty_2ms()
{
	float inner_loop_time;
	
	inner_loop_time = Get_Cycle_T(0); 						//获取内环准确的执行周期
	
	test[0] = GetSysTime_us()/1000000.0f;
	
//	MPU6050_Read(); 															//读取mpu6轴传感器

//	MPU6050_Data_Prepare( inner_loop_time );			//mpu6轴传感器数据处理

//	CTRL_1( inner_loop_time ); 										//内环角速度控制。输入：执行周期，期望角速度，测量角速度，角度前馈；输出：电机PWM占空比。<函数未封装>
	
	RC_Duty( inner_loop_time , Rc_Pwm_In );				// 遥控器通道数据处理 ，输入：执行周期，接收机pwm捕获的数据。
	
	
	
	test[1] = GetSysTime_us()/1000000.0f;
}

void Duty_5ms()
{
	float outer_loop_time;
	
	outer_loop_time = Get_Cycle_T(2);								//获取外环准确的执行周期
	
	test[2] = GetSysTime_us()/1000000.0f;
	
	/*IMU更新姿态。输入：半个执行周期，三轴陀螺仪数据（转换到度每秒），三轴加速度计数据（4096--1G）；输出：ROLPITYAW姿态角*/
 	IMUupdate(0.5f *outer_loop_time,mpu6050.Gyro_deg.x, mpu6050.Gyro_deg.y, mpu6050.Gyro_deg.z, mpu6050.Acc.x, mpu6050.Acc.y, mpu6050.Acc.z,&Roll,&Pitch,&Yaw);

 	CTRL_2( outer_loop_time ); 											// 外环角度控制。输入：执行周期，期望角度（摇杆量），姿态角度；输出：期望角速度。<函数未封装>
	
	test[3] = GetSysTime_us()/1000000.0f;
}

void Duty_10ms()
{
// 		if( MS5611_Update() ) 				//更新ms5611气压计数据
//		{	
//			baro_ctrl_start = 1;  //20ms
//		}
		
//	  ANO_AK8975_Read();			//获取电子罗盘数据	
}

void Duty_20ms()
{
//	Parameter_Save();
}

void Duty_50ms()
{
//	Mode();
//	LED_Duty();								//LED任务
//	Ultra_Duty();
}


void Duty_Loop()   					//最短任务周期为1ms，总的代码执行时间需要小于1ms。
{

	if( loop.check_flag == 1 )
	{
		loop_cnt = time_1ms;
		
		Duty_1ms();							//周期1ms的任务
		
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();						//周期2ms的任务
		}
		if( loop.cnt_5ms >= 5 )
		{
			loop.cnt_5ms = 0;
			Duty_5ms();						//周期5ms的任务
		}
		if( loop.cnt_10ms >= 10 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//周期10ms的任务
		}
		if( loop.cnt_20ms >= 20 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//周期20ms的任务
		}
		if( loop.cnt_50ms >= 50 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//周期50ms的任务
		}
		
		loop.check_flag = 0;		//循环运行完毕标志
	}
}




void remote_update(void)
{
    if (myReceiver.cmdReadyFlag == 1)
    {
        myReceiver.cmdReadyFlag = 0;
    }
    else
    {
        // No full command has been received yet.
        return;
    }
    cnt++;
    if ((myReceiver.cmdIn[0] != 0xFA) || (myReceiver.cmdIn[1] != 0xFB))
    {
        if(cnt>1)
        {
            cnt=0;
        }
    }
    switch(myReceiver.cmdIn[2])
    {
        case 0x05:           //飞控状态传送
//			  if(api->getBroadcastData().status==2)
//			  {
//				  if(startState==0)
//				  {
//					  if(api->getBroadcastData().pos.health>3)
//					  {
//						  startState  = 1;
//						  startlatitude = api->getBroadcastData().pos.latitude;
//						  startlongitude = api->getBroadcastData().pos.longitude;
//					  }
//				  }
//			  }else{
//				  startState  = 0;
//			  }
//			  f32Distance=f64Distance = get_distance(startlatitude,startlongitude,\
//									   api->getBroadcastData().pos.latitude,\
//									   api->getBroadcastData().pos.longitude);
//			  
//			f32Height = api->getBroadcastData().pos.height;
//			  planeSpeed.x = api->getBroadcastData().v.x;
//			  planeSpeed.y = api->getBroadcastData().v.y;
//			  planeSpeed.z = api->getBroadcastData().v.z;
//			  Health = api->getBroadcastData().pos.health;
//			
//			  arm_sqrt_f32(planeSpeed.x*planeSpeed.x+planeSpeed.y*planeSpeed.y , &f32Dspeed);		  
//			  f32Hspeed = planeSpeed.z;
//			  if(f32Height<0)f32Height = -f32Height;
//			  u16Height = (uint16_t)(f32Height*100);//mm
//			  u32Distance =(uint32_t)(f32Distance*10);//dm
//			  u16Distance = (uint16_t)u32Distance;
//			  
//			  u16Dspeed = (uint16_t)(f32Dspeed*10);   //m/s
//			  s8Hspeed	= (int8_t)(f32Hspeed*10);	  //m/s
            receiverPutChar(0xfa);
            size=10;
            sendBuf[0] = size;
            sendBuf[1] = type;
            memcpy(&sendBuf[2],&u16Height,2);
            memcpy(&sendBuf[4],&u16Distance,2);
            memcpy(&sendBuf[6],&u16Dspeed,2);
            memcpy(&sendBuf[8],&s8Hspeed,1);
            memcpy(&sendBuf[9],&Health,1);
            receiverSend(sendBuf,10);
            receiverPutChar(0xfe);
            break;
        case 0x06:
            break;
        case 0x07:
            break;
        case 0x08:
            break;
        case 0x09:
            break;
        default:
            break;
    }
#if 1//def Add_remote_debug
    u8 i=0;
    {
        printf("Test_data ===");

        for (i = 1; i < myReceiver.rxLength-1; i++)
        {
            printf("%02x",myReceiver.cmdIn[i]);
        }
        printf("\r\n");
    }
#endif
}

