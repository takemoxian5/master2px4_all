#include "usart.h"
#include "MissionAPI.h"
#include "height_ctrl.h"
#include "ctrl.h"
#include "OpenTel_Mavlink.h"
#include "include.h"

#include "cppforstm32.h"


#include<time.h>

s16 loop_cnt;


loop_t loop;
u8  gc_target_system=1;
u8  gc_target_component=190;
coord_t coord_gloableA,coord_gloableB,coord_gloableLast,coord_gloableHome;
float grid_angle;

float gfAltitude=6.0000000;//全局变量，待移至 include
#define  WAYPOINT_SIZE  10
#define  ABWAYPOINT_SIZE  50
#define  ABWAYPOINT_PLUS  100
float fight_angle;

//作业参数 ，待移到SD卡
u8 grid_pwm=70;
u8 gubGridSpace=6; //喷洒间距
u8 grid_speed=5;
u8 gubMissionTypeCnt=0;
u8 gubDirectionAB=0;



//作业标志
//typedef enum MAV_DO_REPOSITION_FLAGS
//{
//   WATER_COMTAINER,
//   MAV_DO_REPOSITION_FLAGS_CHANGE_MODE=1, /* The aircraft should immediately transition into guided. This should not be set for follow me applications | */
//   MAV_DO_REPOSITION_FLAGS_ENUM_END=2, /*  | */
//} MAV_DO_REPOSITION_FLAGS;
typedef struct ___ab_flag_s
{
   bool water_remanining;    // 水
   bool battery_remaining;   //
//   bool;
//   bool battery_remaining;
//   bool battery_remaining;
} ab_flag_s;

ab_flag_s ab_flag;

typedef enum AB_MODE_FLAG
{
    AB_MODE_FLAG_N=0,
    AB_MODE_FLAG_A=1,
    AB_MODE_FLAG_B=2,
} AB_MODE_FLAG;


u8 ABcheck_need_flag=0;  //

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
        loop.check_flag = 1;    //该标志位在循环的最后被清零
    }
}

void Duty_1ms()
{
    Get_Cycle_T(1);
//  LED_Display( LED_Brightness );                              //20级led渐变显示
//  ANO_DT_Data_Exchange();                                             //数传通信定时调用
}

float test[5];
void Duty_2ms()
{
    float inner_loop_time;

    inner_loop_time = Get_Cycle_T(0);                       //获取内环准确的执行周期

    test[0] = GetSysTime_us()/1000000.0f;

    test[1] = GetSysTime_us()/1000000.0f;
}

void Duty_5ms()
{
    float outer_loop_time;

    outer_loop_time = Get_Cycle_T(2);                               //获取外环准确的执行周期

    test[2] = GetSysTime_us()/1000000.0f;

    /*IMU更新姿态。输入：半个执行周期，三轴陀螺仪数据（转换到度每秒），三轴加速度计数据（4096--1G）；输出：ROLPITYAW姿态角*/
// IMUupdate(0.5f *outer_loop_time,mpu6050.Gyro_deg.x, mpu6050.Gyro_deg.y, mpu6050.Gyro_deg.z, mpu6050.Acc.x, mpu6050.Acc.y, mpu6050.Acc.z,&Roll,&Pitch,&Yaw);

//  CTRL_2( outer_loop_time );                                          // 外环角度控制。输入：执行周期，期望角度（摇杆量），姿态角度；输出：期望角速度。<函数未封装>

    test[3] = GetSysTime_us()/1000000.0f;
}

void Duty_10ms()
{
//      if( MS5611_Update() )               //更新ms5611气压计数据
//      {
//          baro_ctrl_start = 1;  //20ms
//      }

//    ANO_AK8975_Read();            //获取电子罗盘数据
}

void Duty_20ms()
{
//  Parameter_Save();
}

void Duty_50ms()
{
//  Mode();
//  LED_Duty();                             //LED任务
//  Ultra_Duty();
}


void Duty_Loop()                    //最短任务周期为1ms，总的代码执行时间需要小于1ms。
{

    if( loop.check_flag == 1 )
    {
        loop_cnt = time_1ms;

        Duty_1ms();                         //周期1ms的任务

        if( loop.cnt_2ms >= 2 )
        {
            loop.cnt_2ms = 0;
            Duty_2ms();                     //周期2ms的任务
        }
        if( loop.cnt_5ms >= 5 )
        {
            loop.cnt_5ms = 0;
            Duty_5ms();                     //周期5ms的任务
        }
        if( loop.cnt_10ms >= 10 )
        {
            loop.cnt_10ms = 0;
            Duty_10ms();                    //周期10ms的任务
        }
        if( loop.cnt_20ms >= 20 )
        {
            loop.cnt_20ms = 0;
            Duty_20ms();                    //周期20ms的任务
        }
        if( loop.cnt_50ms >= 50 )
        {
            loop.cnt_50ms = 0;
            Duty_50ms();                    //周期50ms的任务
        }

        loop.check_flag = 0;        //循环运行完毕标志
    }
}



static void item_count(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
    if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MISSION_COUNT >= 256)
    {
        return;
    }
#endif
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t i;
    mavlink_mission_count_t packet_in =
    {
        2,1,190,0
    };
    mavlink_mission_count_t packet1, packet2;
    memset(&packet1, 0, sizeof(packet1));
    packet1.count = packet_in.count;
    packet1.target_system = packet_in.target_system;
    packet1.target_component = packet_in.target_component;
    packet1.mission_type = packet_in.mission_type;


#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
    {
        // cope with extensions
        memset(MAVLINK_MSG_ID_MISSION_COUNT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_COUNT_MIN_LEN);
    }
#endif
    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_count_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mission_count_decode(&msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
//下面两种 效果 一致，
#if 0
    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_count_pack(system_id, component_id, &msg, packet1.target_system, packet1.target_component, packet1.count, packet1.mission_type );
    mavlink_msg_mission_count_decode(&msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_count_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg, packet1.target_system, packet1.target_component, packet1.count, packet1.mission_type );
    mavlink_msg_mission_count_decode(&msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
#endif
    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_to_send_buffer(buffer, &msg);
    for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++)
    {
        comm_send_ch(MAVLINK_COMM_0, buffer[i]);
    }
    mavlink_msg_mission_count_decode(last_msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_count_send(MAVLINK_COMM_1, packet1.target_system, packet1.target_component, packet1.count, packet1.mission_type );
    mavlink_msg_mission_count_decode(last_msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

//mavlink_message_t msg_temp[100];//建议做局部变量
void item_pack(mavlink_mission_item_t* msg,
               uint16_t seq, uint16_t command, float param1, float param2, float param3, float param4, float x, float y, float z)
{
    mavlink_mission_item_t packet;
    msg->param1 = param1;
    msg->param2 = param2;
    msg->param3 = param3;
    msg->param4 = param4;
    msg->x = x;
    msg->y = y;
    msg->z = z;
    msg->seq = seq;
    msg->command = command;
    msg->target_system = gc_target_system;
    msg->target_component = gc_target_component;
    msg->frame = 3;//command ==MAV_CMD_NAV_WAYPOINT ? 3:2;  //global_relative=3 ,mission=2
    msg->current = seq ==0? 1:0;  //set current at first point              //G201801111281 ChenYang
    msg->autocontinue = true;
    msg->mission_type = gubMissionTypeCnt;
}

static void waypoint_send(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
    if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MISSION_ITEM >= 256)
    {
        return;
    }
#endif
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t i;
    mavlink_mission_item_t packet_in =
    {
//        17.0,45.0,73.0,101.0,
//          129.0,157.0,185.0,
//          18691,18795,101,168,235,46,113,180
        0,0,0,0,
        22.5731620,113.5783691,25.0000000,
        0,16,1,190,3,0,1,0
    };
    mavlink_mission_item_t packet1, packet2;
    memset(&packet1, 0, sizeof(packet1));
    packet1.param1 = packet_in.param1;
    packet1.param2 = packet_in.param2;
    packet1.param3 = packet_in.param3;
    packet1.param4 = packet_in.param4;
    packet1.x = packet_in.x;
    packet1.y = packet_in.y;
    packet1.z = packet_in.z;
    packet1.seq = packet_in.seq;
    packet1.command = packet_in.command;
    packet1.target_system = packet_in.target_system;
    packet1.target_component = packet_in.target_component;
    packet1.frame = packet_in.frame;
    packet1.current = packet_in.current;
    packet1.autocontinue = packet_in.autocontinue;
    packet1.mission_type = packet_in.mission_type;


#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
    {
        // cope with extensions
        memset(MAVLINK_MSG_ID_MISSION_ITEM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_ITEM_MIN_LEN);
    }
#endif
    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_item_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mission_item_decode(&msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_item_pack(system_id, component_id, &msg, packet1.target_system, packet1.target_component, packet1.seq, packet1.frame, packet1.command, packet1.current, packet1.autocontinue, packet1.param1, packet1.param2, packet1.param3, packet1.param4, packet1.x, packet1.y, packet1.z, packet1.mission_type );
    mavlink_msg_mission_item_decode(&msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_item_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg, packet1.target_system, packet1.target_component, packet1.seq, packet1.frame, packet1.command, packet1.current, packet1.autocontinue, packet1.param1, packet1.param2, packet1.param3, packet1.param4, packet1.x, packet1.y, packet1.z, packet1.mission_type );
    mavlink_msg_mission_item_decode(&msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_to_send_buffer(buffer, &msg);
    for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++)
    {
        comm_send_ch(MAVLINK_COMM_0, buffer[i]);
    }
    mavlink_msg_mission_item_decode(last_msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mission_item_send(MAVLINK_COMM_1, packet1.target_system, packet1.target_component, packet1.seq, packet1.frame, packet1.command, packet1.current, packet1.autocontinue, packet1.param1, packet1.param2, packet1.param3, packet1.param4, packet1.x, packet1.y, packet1.z, packet1.mission_type );


    mavlink_msg_mission_item_decode(last_msg, &packet2);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

    //最简方式
    mavlink_mission_item_t packet3 =
    {
        0,0,0,0,
        22.5730420,113.57822241,25.0000000,
//              2,16,0,1,190,1,1,1
        1,16,1,190,3,1,1,0
    };
    memset(&packet1, 0, sizeof(packet1));
    packet1 =packet3;
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
    {
        // cope with extensions
        memset(MAVLINK_MSG_ID_MISSION_ITEM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_ITEM_MIN_LEN);
    }
#endif
    mavlink_msg_mission_item_send(MAVLINK_COMM_0, packet1.target_system, packet1.target_component, packet1.seq, packet1.frame, packet1.command, packet1.current, packet1.autocontinue, packet1.param1, packet1.param2, packet1.param3, packet1.param4, packet1.x, packet1.y, packet1.z, packet1.mission_type );

}


//u8 gubGridSpace=6; //喷洒间距



coord_t convertNedToGeo( coord_t origin,coordNed_t distance
// , double* x, double* y, double* z
                       )
{
    coord_t coord_temp;
    double x_rad = distance.y / CONSTANTS_RADIUS_OF_EARTH;
    double y_rad = distance.x / CONSTANTS_RADIUS_OF_EARTH;
    double c = sqrtf(x_rad * x_rad + y_rad * y_rad);
    double sin_c = sin(c);
    double cos_c = cos(c);

    double ref_lon_rad = origin.longitude * M_DEG_TO_RAD;
    double ref_lat_rad = origin.latitude * M_DEG_TO_RAD;

    double ref_sin_lat = sin(ref_lat_rad);
    double ref_cos_lat = cos(ref_lat_rad);

    double lat_rad;
    double lon_rad;
    if (fabs(c) > epsilon)
    {
        lat_rad = asin(cos_c * ref_sin_lat + (x_rad * sin_c * ref_cos_lat) / c);
        lon_rad = (ref_lon_rad + atan2(y_rad * sin_c, c * ref_cos_lat * cos_c - x_rad * ref_sin_lat * sin_c));

    }
    else
    {
        lat_rad = ref_lat_rad;
        lon_rad = ref_lon_rad;
    }

    coord_temp.latitude = lat_rad * M_RAD_TO_DEG;
    coord_temp.longitude =lon_rad * M_RAD_TO_DEG;
    coord_temp.altitude= origin.altitude+distance.z  ;
    return coord_temp;
}

coordNed_t convertGeoToNed(coord_t origin,coord_t coord  )   //左边参考起点，右边当前点
// , double* x, double* y, double* z
                          
{
    coordNed_t coord_temp;
    double lat_rad = coord.latitude * M_DEG_TO_RAD;
    double lon_rad = coord.longitude * M_DEG_TO_RAD;

    double ref_lon_rad = origin.longitude * M_DEG_TO_RAD;
    double ref_lat_rad = origin.latitude * M_DEG_TO_RAD;

    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double cos_d_lon = cos(lon_rad - ref_lon_rad);

    double ref_sin_lat = sin(ref_lat_rad);
    double ref_cos_lat = cos(ref_lat_rad);

    double c = acos(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon);
    double k = (fabs(c) <epsilon) ? 1.0 : (c / sin(c));

    coord_temp.y = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
    coord_temp.x = k * cos_lat * sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH;
    coord_temp.dist= sqrt(coord_temp.x*coord_temp.x+coord_temp.y*coord_temp.y);
    coord_temp.z=(coord.altitude - origin.altitude);
    return coord_temp;
}

//G2018011113141 CY128 坐标转换  Convert polygon to NED
#if 0
void polygon_to_NED()
{
    for (int i=0; i<_mapPolygon.count(); i++)
    {
        double y, x, down;
        coord_t vertex = _mapPolygon.pathModel().value<QGCcoord_t*>(i)->coordinate();
        if (i == 0)
        {
            // This avoids a nan calculation that comes out of convertGeoToNed
            x = y = 0;
        }
        else
        {
            convertGeoToNed(vertex, tangentOrigin, &y, &x, &down);
        }
        polygonPoints += QPointF(x, y);
    }

}
#endif  //end of MAV_LOG_TSET




/***************************************************************************
  Function:          coord_set
  Description:       功能函数，简化代码
  Input:             float altitude       //send count
                     float longitude      //send count
  Output:            void
  Return:            coord_t    //send count
  Others:            none
****************************************************************************/
coord_t coord_set(double latitude,double longitude,double altitude)
{
    coord_t coord_temp;
    coord_temp.latitude  =      latitude;               //              float x,
    coord_temp.longitude =      longitude;              //              float y,
    coord_temp.altitude  =      altitude;             //              float z
    return coord_temp;
}

//发送一些控制 和状态切换命令，  举例如下
#if 0
MAV_CMD_NAV_TAKEOFF=22

#endif




	void send_one_cmd_long( 
	uint16_t command,  
	uint8_t confirmation,  // 0 first tansmission of this
	float param1, float param2, float param3, float param4, float param5, float param6, float param7)
	{
		mavlink_command_long_t packet1;
		memset(&packet1, 0, sizeof(packet1));
		packet1.param1 =  param1;
		packet1.param2 =  param2;
		packet1.param3 =  param3;
		packet1.param4 =  param4;
		packet1.param5 =  param5;
		packet1.param6 =  param6;
		packet1.param7 =  param7;
		packet1.command = command;
		packet1.target_system = gc_target_system;
		packet1.target_component = gc_target_component;
		packet1.confirmation = confirmation;
		_mav_finalize_message_chan_send(MAVLINK_COMM_0, MAVLINK_MSG_ID_COMMAND_LONG, (const char *)&packet1, MAVLINK_MSG_ID_COMMAND_LONG_MIN_LEN, MAVLINK_MSG_ID_COMMAND_LONG_LEN, MAVLINK_MSG_ID_COMMAND_LONG_CRC);

	}

	
	void send_one_miss(u8 seq_cnt,coord_t coord_temp,uint16_t command,
		float param1, float param2, float param3, float param4)
	{
		mavlink_mission_item_t packet_item;
				   item_pack( &packet_item,
						   seq_cnt,
						   command,				   //			   uint16_t command,
						   param1,				   //			   float param1,
						   param2,				   //			   float param2,
						   param3,				   //			   float param3,
						   param4,				   //			   float param4,   angle,只设置第一点即可
						   coord_temp.latitude, 				   //			   float x,
						   coord_temp.longitude,				   //			   float y,
						   coord_temp.altitude					   //			   float z
						 );
		_mav_finalize_message_chan_send(MAVLINK_COMM_0, MAVLINK_MSG_ID_MISSION_ITEM, (const char *)&packet_item, MAVLINK_MSG_ID_MISSION_ITEM_MIN_LEN, MAVLINK_MSG_ID_MISSION_ITEM_LEN, MAVLINK_MSG_ID_MISSION_ITEM_CRC);
	}

										/***************************************************************************
										  Function: 		 功能函数区
										  Description:		 flight mode
										****************************************************************************/

/***************************************************************************
  Function:          pauseVehicle
  Description:       hold mode
  Input:             void
  Output:            void
  Return:            void
  Others:            none
****************************************************************************/
 void holdMiss()
		{
		send_one_cmd_long( 
                            MAV_CMD_DO_REPOSITION,
                            1,   // 
                            -1.0f,
                            MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
                            0.0f,
                            NAN,
                            NAN,
                            NAN,
                            NAN);
		}
  void GotoLocation(coord_t gotoCoord)
		{
		send_one_cmd_long( 
                            MAV_CMD_DO_REPOSITION,
                            1,   // 
                            -1.0f,
                            MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
                            0.0f,
                            NAN,
			gotoCoord.latitude,
			gotoCoord.longitude,
			gotoCoord.altitude);

		}
   void ChangeAltitude(double altitudeChange)
		{
#ifdef next_func   //下一步开发
//	  if (!vehicle->homePosition().isValid()) {
//		  qgcApp()->showMessage(tr("Unable to change altitude, home position unknown."));
//		  return;
//	  }
//	  if (qIsNaN(vehicle->homePosition().altitude())) {
//		  qgcApp()->showMessage(tr("Unable to change altitude, home position altitude unknown."));
//		  return;
//	  }
//	  double currentAltRel = altitudeRelative()->rawValue().toDouble();
#endif
	  
	  double newAltRel = altitudeChange;


		
		send_one_cmd_long( 
                            MAV_CMD_DO_REPOSITION,
                            1,   // 
                            -1.0f,
                            MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
                            0.0f,
                            NAN,
                            NAN,
                            NAN,
                            coord_gloableHome.altitude+newAltRel);
		}
 void startMiss(u8 first_item,u8 last_item)
		{
		send_one_cmd_long( 
                            MAV_CMD_MISSION_START,
                            first_item,   // 
                            last_item,
                            NAN,
                            NAN,
                            NAN,
                            NAN,
                            NAN,
                            NAN);
		}
 void SetServo(u8 servo,u8 pwm)
		{
		send_one_cmd_long( 
                            MAV_CMD_DO_SET_SERVO,
                            servo,   // 
                            pwm,
                            NAN,
                            NAN,
                            NAN,
                            NAN,
                            NAN,
                            NAN);
		}


void send_one_Waypoint(u8 seq_cnt,coord_t coord_temp)
{
	mavlink_mission_item_t packet_item;
			   item_pack( &packet_item,
					   seq_cnt,
					   MAV_CMD_NAV_WAYPOINT,				   //			   uint16_t command,
					   0,				   //			   float param1,
					   0,				   //			   float param2,
					   0,				   //			   float param3,
					   fight_angle,// grid_angle,				   //			   float param4,   angle,只设置第一点即可
					   coord_temp.latitude,					   //			   float x,
					   coord_temp.longitude, 				   //			   float y,
					   coord_temp.altitude					   //			   float z
					 );
	_mav_finalize_message_chan_send(MAVLINK_COMM_0, MAVLINK_MSG_ID_MISSION_ITEM, (const char *)&packet_item, MAVLINK_MSG_ID_MISSION_ITEM_MIN_LEN, MAVLINK_MSG_ID_MISSION_ITEM_LEN, MAVLINK_MSG_ID_MISSION_ITEM_CRC);
}
//#define outmsg3(str,x,y,z) printf("%s==%f,==%f ,==%f\r\n",x,y,z)
 /***************************************************************************
   Function:		  polygon_set_AB
   Description: 	  send count
   Input:			  coord_t coord_A	   //send count
					  coord_t coord_B	   //send count
					  bool direction	   //0 left  1	right
   Output:			  void
   Return:			  void
   Others:			  none
 ****************************************************************************/
 void polygon_set_AB(coord_t coord_A, coord_t coord_B,u8 direction)
 {
 	  gubMissionTypeCnt=rand()%200;
	  gc_target_component=20;
	  gubDirectionAB=direction;
	  ABcheck_need_flag=1;    //发射校验
	 mavlink_mission_item_t msg_temp[4];//建议做局部变量
	 u8 seq_cnt=0;
	 coord_t coord_temp[4];
	 coordNed_t grid_distance,grid_dist_vert,grid_distance_op;
	 u8 i;

		 grid_distance=convertGeoToNed(coord_A, coord_B);
//Start G2018011813141 CY128  $AB 点误操作，判断
		 if(grid_distance.dist<8||grid_distance.dist>200)
{
			printf("grid_distance==%d \r\n",grid_distance.dist);
		 	return;
}
//End G2018011813141 CY128 
u32 grid_angle_temp;
		 grid_angle = (atan2(grid_distance.y, grid_distance.x) * M_RAD_TO_DEG);

		 grid_dist_vert.x=gubGridSpace*cos((grid_angle-90)*M_DEG_TO_RAD);
		 grid_dist_vert.y=gubGridSpace*sin((grid_angle-90)*M_DEG_TO_RAD);
		 		 grid_angle_temp=(int)(90-grid_angle+360)%360;
				 if (grid_angle < 0.0)  
				  		  grid_angle += 360.0;
//				  	  grid_angle=90-grid_angle_temp;
//				  	  if (grid_angle > 90.0) {
//				  		  grid_angle -= 180.0;
//				  	  } else if (grid_angle < -90.0) {
//				  		  grid_angle += 180;
//				  	  }
				 printf("grid_angle==%f fight_angle==%f \r\n",grid_angle);


 if(direction==0)  //左边,方向只改变 垂直向量 方向
 {
		 grid_dist_vert.x=-grid_dist_vert.x;
		 grid_dist_vert.y=-grid_dist_vert.y;
 }
		 grid_distance_op.x=-grid_distance.x;
		 grid_distance_op.y=-grid_distance.y;
		 grid_distance_op.z=-grid_distance.z;
		 printf("grid_dist_vertx==%f,grid_dist_verty==%f ,grid_dist_verty==%f\r\n",grid_dist_vert.x,grid_dist_vert.y,grid_dist_vert.z);
//Start G2018011213141 CY128  send count
		 mavlink_mission_count_t packet;
		  packet.count = ABWAYPOINT_PLUS;
		  packet.target_system = gc_target_system;
		  packet.target_component = gc_target_component;
		  packet.mission_type = gubMissionTypeCnt;
		  _mav_finalize_message_chan_send(MAVLINK_COMM_0, MAVLINK_MSG_ID_MISSION_COUNT, (const char *)&packet, MAVLINK_MSG_ID_MISSION_COUNT_MIN_LEN, MAVLINK_MSG_ID_MISSION_COUNT_LEN, MAVLINK_MSG_ID_MISSION_COUNT_CRC);
//End G2018011213141 CY128 
		  // transfer geo
			  coord_temp[0]=coord_A;
			  coord_temp[1]=coord_B;
//		  send_one_Waypoint(seq_cnt++,coord_A);
//		  send_one_Waypoint(seq_cnt++,coord_B);
	  for (	i = 2 ; i <ABWAYPOINT_PLUS+2 ; i++ )
	 {
			  switch (i%4)
		 {
		   case 0: //v+
		   //奇数点
				 {
			     coord_temp[i%4]=convertNedToGeo(coord_temp[(i+3)%4],grid_dist_vert);
					   break;
				 }
		   case 1: //d+
				 {
				 coord_temp[i%4]=convertNedToGeo(coord_temp[(i+3)%4],grid_distance);
					   break;
				 }
		   case 2:	// v+
				 {
				 coord_temp[i%4]=convertNedToGeo(coord_temp[(i+3)%4],grid_dist_vert);
					   break;
				 }
		   case 3:	//d-
				 {
				 coord_temp[i%4]=convertNedToGeo(coord_temp[(i+3)%4],grid_distance_op);
				  break;
				 }
		 }
 //   printf("coord_tempx==%f,coord_tempy==%f ,coord_tempz==%f\r\n",coord_temp[i].latitude,coord_temp[i].longitude,coord_temp[i].altitude);
	 send_one_Waypoint(seq_cnt++,coord_temp[i%4]);
	 }

 }
 void waypoint_test(void)
{
	coord_t coord_A,coord_B;
	coord_A=coord_set(22.5730988,113.5782881,gfAltitude); //A
	coord_B=coord_set(22.5733928,113.5779696,gfAltitude); //B
	polygon_set_AB(coord_A, coord_B, 0);
	printf(" gubMissionTypeCnt== %d\r\n",gubMissionTypeCnt);
//	gubMissionTypeCnt++;
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
		 case 0x05: 		  //飞控状态传送
 // 		   if(api->getBroadcastData().status==2)
 // 		   {
 // 			   if(startState==0)
 // 			   {
 // 				   if(api->getBroadcastData().pos.health>3)
 // 				   {
 // 					   startState  = 1;
 // 					   startlatitude = api->getBroadcastData().pos.latitude;
 // 					   startlongitude = api->getBroadcastData().pos.longitude;
 // 				   }
 // 			   }
 // 		   }else{
 // 			   startState  = 0;
 // 		   }
 // 		   f32Distance=f64Distance = get_distance(startlatitude,startlongitude,\
 // 									api->getBroadcastData().pos.latitude,\
 // 									api->getBroadcastData().pos.longitude);
 //
 // 		 f32Height = api->getBroadcastData().pos.height;
 // 		   planeSpeed.x = api->getBroadcastData().v.x;
 // 		   planeSpeed.y = api->getBroadcastData().v.y;
 // 		   planeSpeed.z = api->getBroadcastData().v.z;
 // 		   Health = api->getBroadcastData().pos.health;
 //
 // 		   arm_sqrt_f32(planeSpeed.x*planeSpeed.x+planeSpeed.y*planeSpeed.y , &f32Dspeed);
 // 		   f32Hspeed = planeSpeed.z;
 // 		   if(f32Height<0)f32Height = -f32Height;
 // 		   u16Height = (uint16_t)(f32Height*100);//mm
 // 		   u32Distance =(uint32_t)(f32Distance*10);//dm
 // 		   u16Distance = (uint16_t)u32Distance;
 //
 // 		   u16Dspeed = (uint16_t)(f32Dspeed*10);   //m/s
 // 		   s8Hspeed  = (int8_t)(f32Hspeed*10);	   //m/s
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


#ifdef next_func


 int SumSruct(void)
 {
	 int i=0,Sum=0;
	 for(i=0;i<60;i++)
	 {
		 if(Student[i].Label==99)  //99是我设置的X
		 {
			 Sum++;
		 }
	 }
	 return Sum;		 //返回长度
 } 
 //#define outmsg3(str,x,y,z) printf("%s==%f,==%f ,==%f\r\n",x,y,z)
  /***************************************************************************
	Function:		   polygon_set_AB
	Description:	   send count
	Input:			   coord_t coord_A		//send count
					   coord_t coord_B		//send count
					   bool direction		//0 left  1  right
	Output: 		   void
	Return: 		   void
	Others: 		   none
  ****************************************************************************/
  void polygon_set_ABC(void)
  {

  	 // Convert polygon to NED
	 coordNed_t out_coor;
	 coordNed_t *polygonPoints;
	 coordNed_t _mapPolygon[5];
	 coord_t tangentOrigin =_mapPolygon[0] ;//_mapPolygon.pathModel().value<QGCcoord_t*>(0)->coordinate();
	 for (int i=0; i<_mapPolygon_count; i++) {
		 coord_t vertex =_mapPolygon[i] ;
		 if (i == 0) {
			 // This avoids a nan calculation that comes out of convertGeoToNed
			 polygonPoints[i]= {0,0,0,0};
		 } else {
			 polygonPoints[i]=convertGeoToNed(tangentOrigin,vertex);
		 }
	 }
 //Start G2018031913148 CY128  面积计算
	 polygonPoints = _convexPolygon(polygonPoints);
	 double coveredArea = 0.0;
	 for (int i=0; i<polygonPoints.count(); i++) {
		 if (i != 0) {
			 coveredArea += polygonPoints[i - 1].x* polygonPoints[i].y - polygonPoints[i].x* polygonPoints[i -1].y;
		 } else {
			 coveredArea += polygonPoints.last().x* polygonPoints[i].y - polygonPoints[i].x* polygonPoints.last().y;
		 }
	 }
 
	 _setCoveredArea(0.5 * fabs(coveredArea));
 //End G2018031913148 CY128 
	   gubMissionTypeCnt=rand()%200;
	   gc_target_component=20;
	   gubDirectionAB=direction;
	   ABcheck_need_flag=1;    //发射校验
		  mavlink_mission_item_t msg_temp[4];//建议做局部变量
		  u8 seq_cnt=0;
		  coord_t coord_temp[4];
		  coordNed_t grid_distance,grid_dist_vert,grid_distance_op;
		  u8 i;
 
		  grid_distance=convertGeoToNed(coord_A, coord_B);
 //Start G2018011813141 CY128  $AB 点误操作，判断
		  if(grid_distance.dist<8||grid_distance.dist>200)
 {
		 printf("grid_distance==%d \r\n",grid_distance.dist);
		 return;
 }
 //End G2018011813141 CY128 
 u32 grid_angle_temp;
		  grid_angle = (atan2(grid_distance.y, grid_distance.x) * M_RAD_TO_DEG);
 
		  grid_dist_vert.x=gubGridSpace*cos((grid_angle-90)*M_DEG_TO_RAD);
		  grid_dist_vert.y=gubGridSpace*sin((grid_angle-90)*M_DEG_TO_RAD);
				  grid_angle_temp=(int)(90-grid_angle+360)%360;
				  if (grid_angle < 0.0)  
						   grid_angle += 360.0;
 // 				   grid_angle=90-grid_angle_temp;
 // 				   if (grid_angle > 90.0) {
 // 					   grid_angle -= 180.0;
 // 				   } else if (grid_angle < -90.0) {
 // 					   grid_angle += 180;
 // 				   }
				  printf("grid_angle==%f fight_angle==%f \r\n",grid_angle);
 
 
  if(direction==0)	//左边,方向只改变 垂直向量 方向
  {
		  grid_dist_vert.x=-grid_dist_vert.x;
		  grid_dist_vert.y=-grid_dist_vert.y;
  }
		  grid_distance_op.x=-grid_distance.x;
		  grid_distance_op.y=-grid_distance.y;
		  grid_distance_op.z=-grid_distance.z;
		  printf("grid_dist_vertx==%f,grid_dist_verty==%f ,grid_dist_verty==%f\r\n",grid_dist_vert.x,grid_dist_vert.y,grid_dist_vert.z);
 //Start G2018011213141 CY128  send count
		  mavlink_mission_count_t packet;
		   packet.count = ABWAYPOINT_PLUS;
		   packet.target_system = gc_target_system;
		   packet.target_component = gc_target_component;
		   packet.mission_type = gubMissionTypeCnt;
		   _mav_finalize_message_chan_send(MAVLINK_COMM_0, MAVLINK_MSG_ID_MISSION_COUNT, (const char *)&packet, MAVLINK_MSG_ID_MISSION_COUNT_MIN_LEN, MAVLINK_MSG_ID_MISSION_COUNT_LEN, MAVLINK_MSG_ID_MISSION_COUNT_CRC);
 //End G2018011213141 CY128 
		   // transfer geo
			   coord_temp[0]=coord_A;
			   coord_temp[1]=coord_B;
 // 	   send_one_Waypoint(seq_cnt++,coord_A);
 // 	   send_one_Waypoint(seq_cnt++,coord_B);
	   for ( i = 2 ; i <ABWAYPOINT_PLUS+2 ; i++ )
	  {
			   switch (i%4)
		  {
			case 0: //v+
			//奇数点
				  {
				  coord_temp[i%4]=convertNedToGeo(coord_temp[(i+3)%4],grid_dist_vert);
						break;
				  }
			case 1: //d+
				  {
				  coord_temp[i%4]=convertNedToGeo(coord_temp[(i+3)%4],grid_distance);
						break;
				  }
			case 2:  // v+
				  {
				  coord_temp[i%4]=convertNedToGeo(coord_temp[(i+3)%4],grid_dist_vert);
						break;
				  }
			case 3:  //d-
				  {
				  coord_temp[i%4]=convertNedToGeo(coord_temp[(i+3)%4],grid_distance_op);
				   break;
				  }
		  }
  //   printf("coord_tempx==%f,coord_tempy==%f ,coord_tempz==%f\r\n",coord_temp[i].latitude,coord_temp[i].longitude,coord_temp[i].altitude);
	  send_one_Waypoint(seq_cnt++,coord_temp[i%4]);
	  }
 
  }

 void  _generateGrid(void)
 {
#ifdef next_func
	 if (_ignoreRecalc) {
		 return;
	 }
 //AB 点模式
	 if (_mapPolygon.count() < 3 || _gridSpacingFact.rawValue().toDouble() <= 0) {
		 _clearInternal();
		 return;
	 }
#endif
	 _simpleGridPoints.clear();
	 _transectSegments.clear();
	 _reflyTransectSegments.clear();
	 _additionalFlightDelaySeconds = 0;
	 QList<QList<QPointF>>	 transectSegments;
	 // Convert polygon to NED
	 coordNed_t out_coor;
	 coordNed_t *polygonPoints;
	 coordNed_t _mapPolygon[5];
	 coord_t tangentOrigin =_mapPolygon[0] ;//_mapPolygon.pathModel().value<QGCcoord_t*>(0)->coordinate();
	 for (int i=0; i<_mapPolygon_count; i++) {
		 coord_t vertex =_mapPolygon[i] ;
		 if (i == 0) {
			 // This avoids a nan calculation that comes out of convertGeoToNed
			 polygonPoints[i]= {0,0,0,0};
		 } else {
			 polygonPoints[i]=convertGeoToNed(tangentOrigin,vertex);
		 }
	 }
 //Start G2018031913148 CY128  面积计算
	 polygonPoints = _convexPolygon(polygonPoints);
	 double coveredArea = 0.0;
	 for (int i=0; i<polygonPoints.count(); i++) {
		 if (i != 0) {
			 coveredArea += polygonPoints[i - 1].x* polygonPoints[i].y - polygonPoints[i].x* polygonPoints[i -1].y;
		 } else {
			 coveredArea += polygonPoints.last().x* polygonPoints[i].y - polygonPoints[i].x* polygonPoints.last().y;
		 }
	 }
 
	 _setCoveredArea(0.5 * fabs(coveredArea));
 //End G2018031913148 CY128 
 
	 // Generate grid
	 int cameraShots = 0;
	 cameraShots += _gridGenerator(polygonPoints, transectSegments, false /* refly */);
	 _convertTransectToGeo(transectSegments, tangentOrigin, _transectSegments);
	 _adjustTransectsToEntryPointLocation(_transectSegments);
	 _appendGridPointsFromTransects(_transectSegments);
	 if (_refly90Degrees) {
		 QVariantList reflyPointsGeo;
 
		 transectSegments.clear();
		 cameraShots += _gridGenerator(polygonPoints, transectSegments, true /* refly */);
		 _convertTransectToGeo(transectSegments, tangentOrigin, _reflyTransectSegments);
		 _optimizeTransectsForShortestDistance(_transectSegments.last().last(), _reflyTransectSegments);
		 _appendGridPointsFromTransects(_reflyTransectSegments);
	 }
 
 
	 // Calc survey distance
	 double surveyDistance = 0.0;
	 for (int i=1; i<_simpleGridPoints.count(); i++) {
		 coord_t coord1 = _simpleGridPoints[i-1].value<coord_t>();
		 coord_t coord2 = _simpleGridPoints[i].value<coord_t>();
		 surveyDistance += coord1.distanceTo(coord2);
	 }
	 _setSurveyDistance(surveyDistance);
 
	 if (cameraShots == 0 && _triggerCamera()) {
		 cameraShots = (int)floor(surveyDistance / _triggerDistance());
		 // Take into account immediate camera trigger at waypoint entry
		 cameraShots++;
	 }
	 _setCameraShots(cameraShots);
 
	 if (_hoverAndCaptureEnabled()) {
		 _additionalFlightDelaySeconds = cameraShots * _hoverAndCaptureDelaySeconds;
	 }
	 // Determine command count for lastSequenceNumber
 
	 _missionCommandCount= 0;
	 for (int i=0; i<_transectSegments.count(); i++) {
		 const QList<coord_t>& transectSegment = _transectSegments[i];
 
		 _missionCommandCount += transectSegment.count();	 // This accounts for all waypoints
		 if (_hoverAndCaptureEnabled()) {
			 // Internal camera trigger points are entry point, plus all points before exit point
			 _missionCommandCount += transectSegment.count() - (_hasTurnaround() ? 2 : 0) - 1;
		 } else if (_triggerCamera()) {
			 _missionCommandCount += 2; 						 // Camera on/off at entry/exit
		 }
	 }
 
	 // Set exit coordinate
	 if (_simpleGridPoints.count()) {
		 coord_t coordinate = _simpleGridPoints.first().value<coord_t>();
		 coordinate.setAltitude(_gridAltitudeFact.rawValue().toDouble());
		 setCoordinate(coordinate);
		 coord_t exitCoordinate = _simpleGridPoints.last().value<coord_t>();
		 exitCoordinate.setAltitude(_gridAltitudeFact.rawValue().toDouble());
		 _setExitCoordinate(exitCoordinate);
	 }
	 setDirty(true);
 }





 //生成任务点
 int _gridGenerator(const QList<QPointF>& polygonPoints,  QList<QList<QPointF>>& transectSegments, bool refly)
 {
	 int cameraShots = 0;
	 double gridAngle = _gridAngleFact.rawValue().toDouble();
	 double gridSpacing = _gridSpacingFact.rawValue().toDouble();
	 gridAngle = _clampGridAngle90(gridAngle);
	 gridAngle += refly ? 90 : 0;
	 transectSegments.clear();
	 // Convert polygon to bounding rect
	 QPolygonF polygon;
	 for (int i=0; i<polygonPoints.count(); i++) {
		 polygon << polygonPoints[i];
	 }
	 polygon << polygonPoints[0];
	 QRectF smallBoundRect = polygon.boundingRect();
	 QPointF boundingCenter = smallBoundRect.center();
 
#ifdef next_func
	 // Rotate the bounding rect around it's center to generate the larger bounding rect
	 QPolygonF boundPolygon;
	 boundPolygon << _rotatePoint(smallBoundRect.topLeft(), 	 boundingCenter, gridAngle);
	 boundPolygon << _rotatePoint(smallBoundRect.topRight(),	 boundingCenter, gridAngle);
	 boundPolygon << _rotatePoint(smallBoundRect.bottomRight(),  boundingCenter, gridAngle);
	 boundPolygon << _rotatePoint(smallBoundRect.bottomLeft(),	 boundingCenter, gridAngle);
	 boundPolygon << boundPolygon[0];
	 QRectF largeBoundRect = boundPolygon.boundingRect();
#endif
 
	 // Create set of rotated parallel lines within the expanded bounding rect. Make the lines larger than the
	 // bounding box to guarantee intersection.
 
	 QList<QLineF> lineList;
	 bool northSouthTransects = _gridAngleIsNorthSouthTransects();
	 int entryLocation = _gridEntryLocationFact.rawValue().toInt();
 
	 if (northSouthTransects) {
		 if (entryLocation == EntryLocationTopLeft || entryLocation == EntryLocationBottomLeft) {
			 // Generate transects from left to right
			 float x = largeBoundRect.topLeft().x- (gridSpacing / 2);
			 while (x < largeBoundRect.bottomRight().x()) {
				 float yTop =	 largeBoundRect.topLeft().y - 10000.0;
				 float yBottom = largeBoundRect.bottomRight().y + 10000.0;
 
				 lineList += QLineF(_rotatePoint(QPointF(x, yTop), boundingCenter, gridAngle), _rotatePoint(QPointF(x, yBottom), boundingCenter, gridAngle));
 
				 x += gridSpacing;
			 }
		 } else {
			 // Generate transects from right to left
			 float x = largeBoundRect.topRight().x+ (gridSpacing / 2);
			 while (x > largeBoundRect.bottomLeft().x()) {
				 float yTop =	 largeBoundRect.topRight().y - 10000.0;
				 float yBottom = largeBoundRect.bottomLeft().y + 10000.0;
				 lineList += QLineF(_rotatePoint(QPointF(x, yTop), boundingCenter, gridAngle), _rotatePoint(QPointF(x, yBottom), boundingCenter, gridAngle));
 
				 x -= gridSpacing;
			 }
		 }
	 } else {
		 gridAngle = _clampGridAngle90(gridAngle - 90.0);
		 if (entryLocation == EntryLocationTopLeft || entryLocation == EntryLocationTopRight) {
			 // Generate transects from top to bottom
			 float y = largeBoundRect.bottomLeft().y + (gridSpacing / 2);
			 while (y > largeBoundRect.topRight().y) {
				 float xLeft =	 largeBoundRect.bottomLeft().x- 10000.0;
				 float xRight =  largeBoundRect.topRight().x+ 10000.0;
 
				 lineList += QLineF(_rotatePoint(QPointF(xLeft, y), boundingCenter, gridAngle), _rotatePoint(QPointF(xRight, y), boundingCenter, gridAngle));
 
				 y -= gridSpacing;
			 }
		 } else {
			 // Generate transects from bottom to top
			 float y = largeBoundRect.topLeft().y - (gridSpacing / 2);
			 while (y < largeBoundRect.bottomRight().y) {
				 float xLeft =	 largeBoundRect.topLeft().x- 10000.0;
				 float xRight =  largeBoundRect.bottomRight().x+ 10000.0;
 
				 lineList += QLineF(_rotatePoint(QPointF(xLeft, y), boundingCenter, gridAngle), _rotatePoint(QPointF(xRight, y), boundingCenter, gridAngle));
 
				 y += gridSpacing;
			 }
		 }
	 }
 
	 // Now intersect the lines with the polygon
	 QList<QLineF> intersectLines;
#if 1
	 _intersectLinesWithPolygon(lineList, polygon, intersectLines);
#else
	 // This is handy for debugging grid problems, not for release
	 intersectLines = lineList;
#endif
 
	 // Less than two transects intersected with the polygon:
	 // 	 Create a single transect which goes through the center of the polygon
	 // 	 Intersect it with the polygon
	 if (intersectLines.count() < 2) {
		 _mapPolygon.center();
		 QLineF firstLine = lineList.first();
		 QPointF lineCenter = firstLine.pointAt(0.5);
		 QPointF centerOffset = boundingCenter - lineCenter;
		 firstLine.translate(centerOffset);
		 lineList.clear();
		 lineList.append(firstLine);
		 intersectLines = lineList;
		 _intersectLinesWithPolygon(lineList, polygon, intersectLines);
	 }
 
	 // Make sure all lines are going to same direction. Polygon intersection leads to line which
	 // can be in varied directions depending on the order of the intesecting sides.
	 QList<QLineF> resultLines;
	 _adjustLineDirection(intersectLines, resultLines);
 
	 // Calc camera shots here if there are no images in turnaround
	 if (_triggerCamera() && !_imagesEverywhere()) {
		 for (int i=0; i<resultLines.count(); i++) {
			 cameraShots += (int)floor(resultLines[i].length() / _triggerDistance());
			 // Take into account immediate camera trigger at waypoint entry
			 cameraShots++;
		 }
	 }
 
	 // Turn into a path
	 for (int i=0; i<resultLines.count(); i++) {
		 QLineF 		 transectLine;
		 QList<QPointF>  transectPoints;
		 const QLineF&	 line = resultLines[i];
 
		 float turnaroundPosition = _turnaroundDistance() / line.length();
 
		 if (i & 1) {
			 transectLine = QLineF(line.p2(), line.p1());
		 } else {
			 transectLine = QLineF(line.p1(), line.p2());
		 }
 
		 // Build the points along the transect
 
		 if (_hasTurnaround()) {
			 transectPoints.append(transectLine.pointAt(-turnaroundPosition));
		 }
 
		 // Polygon entry point
		 transectPoints.append(transectLine.p1());
 
		 // For hover and capture we need points for each camera location
		 if (_triggerCamera() && _hoverAndCaptureEnabled()) {
			 if (_triggerDistance() < transectLine.length()) {
				 int innerPoints = floor(transectLine.length() / _triggerDistance());
				 float transectPositionIncrement = _triggerDistance() / transectLine.length();
				 for (int i=0; i<innerPoints; i++) {
					 transectPoints.append(transectLine.pointAt(transectPositionIncrement * (i + 1)));
				 }
			 }
		 }
 
		 // Polygon exit point
		 transectPoints.append(transectLine.p2());
 
		 if (_hasTurnaround()) {
			 transectPoints.append(transectLine.pointAt(1 + turnaroundPosition));
		 }
 
		 transectSegments.append(transectPoints);
	 }
 
	 return cameraShots;
 }

#endif  //end of next_func










