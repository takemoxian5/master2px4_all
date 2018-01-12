#include "sys.h"
#include "delay.h"
#include "usart.h"
//#include "led.h"
//#include "key.h"
//#include "lcd.h"
//#include "ov7725.h"
#include "mytimer.h"
//#include "malloc.h"
#include "sdio_sdcard.h"
//#include "usmart.h"
//#include "w25qxx.h"
#include "ff.h"
//#include "exfuns.h"
//#include "bmp.h"
#include "stdio.h"
//#include <string.h>
//#include "mavlink.h"
#include "OpenTel_Mavlink.h"
#include "testsuite.h"
#include "MissionAPI.h"	

//#include "cppforstm32.h"
#ifdef __cplusplus
extern "C" {
#endif
//#include "testsuite.h"
unsigned int black_num[9];
unsigned int frame_num=0;
uint32_t sample_time=0;


mavlink_system_t mavlink_system;
mavlink_message_t* msg ;
mavlink_message_t* last_msg;

uint8_t testTxBuf[10] = {1,2,3,4,5,6,7,8,9,10};


u8 Init_Finish = 0;
//u8 fly_ready = 0;







FRESULT open_append (
    FIL* fp,            /* [OUT] File object to create */
    const char* path    /* [IN]  File name to be opened */
)
{
    FRESULT fr;

    /* Opens an existing file. If not exist, creates a new file. */
    fr = f_open(fp, path, FA_WRITE | FA_OPEN_ALWAYS);
    if (fr == FR_OK)
    {
        /* Seek to end of the file to append data */
        fr = f_lseek(fp, f_size(fp));
        if (fr != FR_OK)
            f_close(fp);
    }
    return fr;
}
static void SD_CARD_INIT(void)
{
    SDIO_Interrupts_Config();   /* 配置SDIO中断�?此函数在bsp_sdio_sd.c */
//SD卡检�?
    while(SD_Init())//检测不到SD�?
    {
        printf("\r\nNO SD Card\r\n");
        delay_ms(500);

    }
    show_sdcard_info(); //打印SD卡相关信�?
    printf("\r\n SD Card OK\r\n");//检测SD卡成�?
}
void delay(void)
{
    uint16_t count = 30000;
    while(count--);
}
#ifdef DEBUG_SEND_MSG
char  test_cntxx[20];
#endif


/*
 *
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断优先级组别设置
	//SysTick_Config(SystemCoreClock / 1000); //滴答时钟
	SysTick_Configuration(); 	//滴答时钟
    delay_init(168);      // 延时
    serial_open(0, 0);
    Para_Init();							//参数初始化
    TIM3_Int_Init(0xFFFF,8400-1);   //��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ��
    
    Cycle_Time_Init();
    
    printf("STM32F4Discovery Board initialization finished!\r\n");

    mavlink_system.sysid =6;//MAV_TYPE_GCS;// MAV_TYPE_GCS=6地面站角色 MAV_TYPE_FIXED_WING;//MAV_TYPE_GENERIC;
    mavlink_system.compid =0;//MAV_AUTOPILOT_GENERIC;//=0
	mavlink_servo_output_raw_t* servo_output_raw;
	servo_output_raw->time_usec = 20000000;
	servo_output_raw->servo1_raw = 950;
	servo_output_raw->servo2_raw = 950;
	servo_output_raw->servo3_raw = 950;
	servo_output_raw->servo4_raw = 950;
	servo_output_raw->servo5_raw = 950;
	servo_output_raw->servo6_raw = 950;
	servo_output_raw->servo7_raw = 950;
	servo_output_raw->servo8_raw = 1550;
	servo_output_raw->port = 1;

	
	mavlink_command_long_t com = { 0 };
	mavlink_set_mode_t set_mode={0};
	mavlink_command_long_t take_off_local={0};
	mavlink_command_long_t take_off={0};
	mavlink_command_long_t land={0};
	mavlink_command_long_t ROI={0};
	mavlink_set_position_target_local_ned_t set_position={0};
	/*mavlink_control_system_state_t control={0};
	control.airspeed=-1;
	control.x_acc=1;
	control.x_vel=2;
	control.x_pos=3;
	control.y_acc=1;
	control.y_vel=2;
	control.y_pos=3;
	control.z_acc=1;
	control.z_vel=2;
	control.z_pos=3;*/
	
	set_position.target_system=1;
	set_position.target_component=0;
	set_position.coordinate_frame=MAV_FRAME_LOCAL_NED;
	set_position.type_mask=4039;
	set_position.time_boot_ms=0;
	set_position.afx=0;
	set_position.vx=8;
	set_position.x=0;
	set_position.afy=0;
	set_position.vy=8;
	set_position.y=0;
	set_position.afz=0;
	set_position.vz=8;
	set_position.z=0;
	
	com.target_system    = 1;
	com.target_component = 190;
	com.command          = MAV_CMD_DO_SET_SERVO;//MAV_CMD_COMPONENT_ARM_DISARM;
	com.confirmation     = 0; //first
	com.param1           = 4; // flag >0.5 => start, <0.5 => stop
	com.param2           = 1314; // flag >0.5 => start, <0.5 => stop
	
//	set_mode.target_system=1;
//	set_mode.base_mode=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
//	set_mode.custom_mode=4;
		set_mode.custom_mode=4;
		set_mode.target_system=1;
		set_mode.base_mode=157;//MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
		

	take_off_local.target_system=1;
	take_off_local.target_component=0;
	take_off_local.confirmation=true;
	take_off_local.command=MAV_CMD_NAV_TAKEOFF_LOCAL;
	take_off_local.param1=  0;
	take_off_local.param3=30;
	take_off_local.param4=0;
	take_off_local.param5=0;
	take_off_local.param6=0;
	take_off_local.param7=5;
	
	ROI.target_system=1;
	ROI.target_component=0;
	ROI.confirmation=true;
	ROI.command=MAV_CMD_DO_SET_SERVO;
	ROI.param1=1;
	ROI.param2=1010;
	ROI.param3=0;
	ROI.param4=0;
	ROI.param5=0;
	ROI.param6=0;
	ROI.param7=0;
	
	take_off.target_system=1;
	take_off.target_component=0;
	take_off.confirmation=true;
	take_off.command=MAV_CMD_NAV_TAKEOFF;
	take_off.param1= 4000;
	take_off.param4=0;
	take_off.param5=0;
	take_off.param6=0;
	take_off.param7=1;
	
	land.target_system=1;
	land.command=MAV_CMD_NAV_LAND;
	land.confirmation=true;
	land.target_component=0;
	land.param5=0;
	land.param6=0;
	land.param7=0;
	int a=0;
	int b=0;
	int c=0;
//	global_position_int->vx=128;

    while(1)
    {
//        mavlink_send_message(0, MSG_HEARTBEAT, 0);
//        mavlink_send_message(0, MSG_LOCATION, 0);
        while(1)
        {
            if(sysTickUptime % 200==1)
            {
//			  sysTickUptime = 0;
//			  mavlink_send_message(0, MSG_LOCATION, 0);
			  mavlink_send_message(0, MSG_HEARTBEAT, 0);
//			mavlink_msg_command_long_send_struct(MAVLINK_COMM_0,&com);
//			mavlink_msg_command_long_send_struct(MAVLINK_COMM_0,&land);
		mavlink_msg_set_mode_send_struct(MAVLINK_COMM_0,&set_mode);

		//USART_SendData(USART2,a);
		//while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//等待发送结束
#if 0//def Add_remote
		if(a==1)
		{
			USART_SendData(USART2,b);
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//等待发送结束
			if(b!=1&&c==1)
			{
				
			}
			delay_ms(500);
			mavlink_msg_command_long_send_struct(MAVLINK_COMM_0,&take_off);
			if(c==1)
			{
				USART_SendData(USART2,8);
				while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//等待发送结束
				TIM3->CR1|=0x01;    //使能定时器3 使能TIMx	
			}
			if(d>=10)
			{
				mavlink_msg_set_mode_send_struct(MAVLINK_COMM_0,&set_mode);
				USART_SendData(USART2,9);
				while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//等待发送结束
			}
			if(d>=20&&b==1)
			{
				mavlink_msg_command_long_send_struct(MAVLINK_COMM_0,&ROI);
				//mavlink_msg_set_position_target_local_ned_send_struct(MAVLINK_COMM_0,&set_position);
				USART_SendData(USART2,10);
				while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//等待发送结束
				//
			}
		}
#endif  //end of Add_remote
               
//                  serial_write_buf(testTxBuf, 9);
//      delay();
//      u8 i;
//      for(i=0; i<10; i++) testTxBuf[i]++;
//    delay();
//                mavlink_send_message(0, MSG_HEARTBEAT, 0);
//                mavlink_send_message(0, MSG_ATTITUDE, 0);
//                mavlink_send_message(0, MSG_AHRS, 0);
//                mavlink_test_minimal(15, 7, msg);
            }
			else if(sysTickUptime % 200==5)
            {
uint8_t            system_id=6,component_id= 0;
//uint8_t            system_id=mavlink_msg_mission_count_get_target_system(msg);
//uint8_t			 component_id=  mavlink_msg_mission_count_get_target_component(msg);

//mavlink_test_set_mode(system_id, component_id, last_msg);

//			mavlink_test_mission_count(system_id, component_id, last_msg);
//			mavlink_test_mission_item(system_id, component_id, last_msg);


//waypoint_test();



//	mavlink_test_mission_request_partial_list(system_id, component_id, last_msg);
//    mavlink_test_mission_write_partial_list(system_id, component_id, last_msg);
//    mavlink_test_mission_item(system_id, component_id, last_msg);
//    mavlink_test_mission_request(system_id, component_id, last_msg);
//    mavlink_test_mission_set_current(system_id, component_id, last_msg);
//    mavlink_test_mission_current(system_id, component_id, last_msg);
//    mavlink_test_mission_request_list(system_id, component_id, last_msg);
//    mavlink_test_mission_count(system_id, component_id, last_msg);
//    mavlink_test_mission_clear_all(system_id, component_id, last_msg);
//    mavlink_test_mission_item_reached(system_id, component_id, last_msg);
//    mavlink_test_mission_ack(system_id, component_id, last_msg);
			}
            update();
//			remote_update();
        }
#ifdef TEST_UART
        serial_write_buf(testTxBuf, 10);
        delay();
        for(i=0; i<10; i++) testTxBuf[i]++;
        delay();
        while(1);
#endif
    }
}
#ifdef __cplusplus
}
#endif // __cplusplus

