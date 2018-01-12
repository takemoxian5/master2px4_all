#include "usart.h"
#include "MissionAPI.h"
#include "height_ctrl.h"
#include "ctrl.h"
#include "OpenTel_Mavlink.h"
#include "include.h"

#include "cppforstm32.h"
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

//  MPU6050_Read();                                                             //读取mpu6轴传感器

//  MPU6050_Data_Prepare( inner_loop_time );            //mpu6轴传感器数据处理

//  CTRL_1( inner_loop_time );                                      //内环角速度控制。输入：执行周期，期望角速度，测量角速度，角度前馈；输出：电机PWM占空比。<函数未封装>

//  RC_Duty( inner_loop_time , Rc_Pwm_In );             // 遥控器通道数据处理 ，输入：执行周期，接收机pwm捕获的数据。



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
//            if(api->getBroadcastData().status==2)
//            {
//                if(startState==0)
//                {
//                    if(api->getBroadcastData().pos.health>3)
//                    {
//                        startState  = 1;
//                        startlatitude = api->getBroadcastData().pos.latitude;
//                        startlongitude = api->getBroadcastData().pos.longitude;
//                    }
//                }
//            }else{
//                startState  = 0;
//            }
//            f32Distance=f64Distance = get_distance(startlatitude,startlongitude,\
//                                     api->getBroadcastData().pos.latitude,\
//                                     api->getBroadcastData().pos.longitude);
//
//          f32Height = api->getBroadcastData().pos.height;
//            planeSpeed.x = api->getBroadcastData().v.x;
//            planeSpeed.y = api->getBroadcastData().v.y;
//            planeSpeed.z = api->getBroadcastData().v.z;
//            Health = api->getBroadcastData().pos.health;
//
//            arm_sqrt_f32(planeSpeed.x*planeSpeed.x+planeSpeed.y*planeSpeed.y , &f32Dspeed);
//            f32Hspeed = planeSpeed.z;
//            if(f32Height<0)f32Height = -f32Height;
//            u16Height = (uint16_t)(f32Height*100);//mm
//            u32Distance =(uint32_t)(f32Distance*10);//dm
//            u16Distance = (uint16_t)u32Distance;
//
//            u16Dspeed = (uint16_t)(f32Dspeed*10);   //m/s
//            s8Hspeed  = (int8_t)(f32Hspeed*10);     //m/s
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
    msg->target_system = 1;
    msg->target_component = 190;
    msg->frame = command ==MAV_CMD_NAV_WAYPOINT ? 3:2;  //global_relative=3 ,mission=2
    msg->current = seq ==0? 1:0;  //set current at first point              //G201801111281 ChenYang
    msg->autocontinue = true;
    msg->mission_type = 0;

//    packet->param1 = param1;
//    packet.param2 = param2;
//    packet.param3 = param3;
//    packet.param4 = param4;
//    packet.x = x;
//    packet.y = y;
//    packet.z = z;
//    packet.seq = seq;
//    packet.command = command;
//    packet.target_system = 1;
//    packet.target_component = 190;
//    packet.frame = command ==MAV_CMD_NAV_WAYPOINT ? 3:2;  //global_relative=3 ,mission=2
//  packet.current = seq ==0? 1:0;  //set current at first point              //G201801111281 ChenYang
//    packet.autocontinue = true;
//    packet.mission_type = 0;
////    msg =packet;
//    memcpy(msg, &packet, MAVLINK_MSG_ID_MISSION_ITEM_MIN_LEN);
//  msg->msgid = MAVLINK_MSG_ID_MISSION_ITEM;

//    return mavlink_finalize_message(msg, mavlink_system.sysid, mavlink_system.compid, MAVLINK_MSG_ID_MISSION_ITEM_MIN_LEN, MAVLINK_MSG_ID_MISSION_ITEM_LEN, MAVLINK_MSG_ID_MISSION_ITEM_CRC);
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

//float 有效精度就7位，113.5782881为例至少10位精度，必须double 15~16位
typedef struct __coord_t
{
    double latitude;
    double longitude;
    double altitude;
} coord_t;
coord_t coord_gloable;
typedef struct __coord2_t
{
    float latitude;
    float longitude;
    float altitude;
} coord2_t;

typedef struct __coordNed_t
{
    float y;    //latitude   纬度 ，南北向为y
    float x;      //longitude 东西向为x
    float z;    // 高度
    float dist;    //绝对距离
} coordNed_t;



// These defines are private
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif
#define M_DEG_TO_RAD (M_PI / 180.0)
#define M_RAD_TO_DEG (180.0 / M_PI)
#define CONSTANTS_ONE_G                 9.80665f        /* m/s^2        */
#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C     1.225f          /* kg/m^3       */
#define CONSTANTS_AIR_GAS_CONST             287.1f          /* J/(kg * K)       */
#define CONSTANTS_ABSOLUTE_NULL_CELSIUS         -273.15f        /* °C          */
#define CONSTANTS_RADIUS_OF_EARTH           6371000         /* meters (m)       */


#define epsilon 0.00000001   //精度

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
    coord_temp.altitude= distance.z + origin.altitude;
    return coord_temp;
}

coordNed_t convertGeoToNed(coord_t origin,coord_t coord     //左边参考起点，右边当前点
// , double* x, double* y, double* z
                          )
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
        QGeoCoordinate vertex = _mapPolygon.pathModel().value<QGCQGeoCoordinate*>(i)->coordinate();
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
//       qCDebug(SurveyMissionItemLog) << "vertex:x:y" << vertex << polygonPoints.last().x() << polygonPoints.last().y();
    }

}
#endif  //end of MAV_LOG_TSET
float gfAltitude=25.0000000;//全局变量，待移至 include
#define  WAYPOINT_SIZE  10
#define  ABWAYPOINT_SIZE  10



/***************************************************************************
  Function:          coord_set
  Description:       功能函数，简化代码
  Input:             float altitude       //$
                     float longitude      //$
  Output:            void
  Return:            coord_t    //$
  Others:            none
****************************************************************************/
#if 1
coord_t coord_set(double latitude,double longitude)
#else

coord_t coord_set(float latitude,float longitude)
#endif
{
    coord_t coord_temp;
    coord_temp.latitude  =      latitude;               //              float x,
    coord_temp.longitude =      longitude;              //              float y,
    coord_temp.altitude  =      gfAltitude;             //              float z
    return coord_temp;
}
/***************************************************************************
  Function:          polygon_set_AB
  Description:       $
  Input:             coord_t coord_A      //$
                     coord_t coord_B      //$
                     bool direction       //0 left  1  right
  Output:            void
  Return:            void
  Others:            none
****************************************************************************/
void polygon_set_AB(coord_t coord_A, coord_t coord_B,bool direction)
{
	{
		mavlink_mission_item_t msg_temp[ABWAYPOINT_SIZE];//建议做局部变量
		u8 seq_cnt=0;
		coord_t coord_temp[ABWAYPOINT_SIZE];
		coordNed_t grid_distance,grid_dist_vert,grid_distance_op;
//		coordNed_t grid_points[ABWAYPOINT_SIZE];
		u8 i;
		float grid_angle;
		u8 grid_space=6; //喷洒间距
	//	  coord_temp[0]=coord_set(22.5730988,113.5782881); //A
	//	  coord_temp[1]=coord_set(22.5733928,113.5779696); //B
	//	  coord_temp[2]=coord_set(22.5734273,113.5780150); //B1
	//	grid_distance=convertGeoToNed(coord_temp[0], coord_temp[1]);
//		coord_A=coord_set(22.5730988,113.5782881); //A
//		coord_B=coord_set(22.5733928,113.5779696); //B
		grid_distance=convertGeoToNed(coord_A, coord_B);
		grid_angle = (atan2(grid_distance.y, grid_distance.x) * M_RAD_TO_DEG);
		printf("grid_angle==%f \r\n",grid_angle);
		grid_dist_vert.x=grid_space*cos((grid_angle-90)*M_DEG_TO_RAD);
		grid_dist_vert.y=grid_space*sin((grid_angle-90)*M_DEG_TO_RAD);
//		grid_dist_vert_op.x=-grid_dist_vert.x;
//		grid_dist_vert_op.y=-grid_dist_vert.y;
if(direction==0)  //左边
{
		grid_dist_vert.x=-grid_dist_vert.x;
		grid_dist_vert.y=-grid_dist_vert.y;
}
		grid_distance_op.x=-grid_distance.x;
		grid_distance_op.y=-grid_distance.y;
		printf("grid_dist_vertx==%f,grid_dist_verty==%f ,grid_dist_verty==%f\r\n",grid_dist_vert.x,grid_dist_vert.y,grid_dist_vert.z);
#if 0
		grid_points[1]=grid_distance; //B 点，以A 为起点的绝对坐标
		for (  i = 2 ; i <ABWAYPOINT_SIZE ; i++ )
	{
			 switch (i%4)
		{
		  case 0: //v+
		  //奇数点
				{
				grid_points[i].x=grid_points[i-1].x+grid_dist_vert.x;
				grid_points[i].y=grid_points[i-1].y+grid_dist_vert.y;
					  break;
				}
		  case 1: //d+
				{
				grid_points[i].x=grid_points[i-1].x+grid_distance.x;	
				grid_points[i].y=grid_points[i-1].y+grid_distance.y;
					  break;
				}
		  case 2:  // v+
				{
				grid_points[i].x=grid_points[i-1].x+grid_dist_vert.x;
				grid_points[i].y=grid_points[i-1].y+grid_dist_vert.y;
					  break;
				}
		  case 3:  //d-
				{
				grid_points[i].x=grid_points[i-1].x-grid_distance.x;	
				grid_points[i].y=grid_points[i-1].y-grid_distance.y;
					  break;
				}
		}
	 printf("grid_pointx==%f,grid_pointy==%f ,grid_pointz==%f\r\n",grid_points[i].x,grid_points[i].y,grid_points[i].z);
			 
	}
#endif  //end of MAV_LOG_TSET
	// transfer geo
		coord_temp[0]=coord_A;
		coord_temp[1]=coord_B;
			for (  i = 2 ; i <ABWAYPOINT_SIZE ; i++ )
	{
			 switch (i%4)
		{
		  case 0: //v+
		  //奇数点
				{
			  coord_temp[i]=convertNedToGeo(coord_temp[i-1],grid_dist_vert);
					  break;
				}
		  case 1: //d+
				{
				coord_temp[i]=convertNedToGeo(coord_temp[i-1],grid_distance);
					  break;
				}
		  case 2:  // v+
				{
				coord_temp[i]=convertNedToGeo(coord_temp[i-1],grid_dist_vert);
	//			grid_points[i].x=grid_points[i-1].x+grid_dist_vert.x;
	//			grid_points[i].y=grid_points[i-1].y+grid_dist_vert.y;
					  break;
				}
		  case 3:  //d-
				{
				coord_temp[i]=convertNedToGeo(coord_temp[i-1],grid_distance_op);
				 break;
				}
		}
	 printf("coord_tempx==%f,coord_tempy==%f ,coord_tempz==%f\r\n",coord_temp[i].latitude,coord_temp[i].longitude,coord_temp[i].altitude);
	
	}
	
	
			for (  i = 0; i <ABWAYPOINT_SIZE ; i++ )
	
	{
			item_pack( &msg_temp[seq_cnt],
					   seq_cnt,
					   MAV_CMD_NAV_WAYPOINT,				   //			   uint16_t command,
					   0,				   //			   float param1,
					   0,				   //			   float param2,
					   0,				   //			   float param3,
					   0,				   //			   float param4,   angle,只设置第一点即可
					   coord_temp[i].latitude,					   //			   float x,
					   coord_temp[i].longitude, 				   //			   float y,
					   coord_temp[i].altitude					   //			   float z
					 );
			seq_cnt++;	 
	}
	
	//Start G2018011113141 CY128  $send item
		mavlink_mission_count_t packet;
		packet.count = seq_cnt;
		packet.target_system = 1;
		packet.target_component = 190;
		packet.mission_type = 0;
		_mav_finalize_message_chan_send(MAVLINK_COMM_0, MAVLINK_MSG_ID_MISSION_COUNT, (const char *)&packet, MAVLINK_MSG_ID_MISSION_COUNT_MIN_LEN, MAVLINK_MSG_ID_MISSION_COUNT_LEN, MAVLINK_MSG_ID_MISSION_COUNT_CRC);
		for (  i = 0 ; i <seq_cnt ; i++ )
		{
			_mav_finalize_message_chan_send(MAVLINK_COMM_0, MAVLINK_MSG_ID_MISSION_ITEM, (const char *)&msg_temp[i], MAVLINK_MSG_ID_MISSION_ITEM_MIN_LEN, MAVLINK_MSG_ID_MISSION_ITEM_LEN, MAVLINK_MSG_ID_MISSION_ITEM_CRC);
		}
	//End G2018011113141 CY128
	
	
	}



}

//#define outmsg3(str,x,y,z) printf("%s==%f,==%f ,==%f\r\n",x,y,z)

 void waypoint_test()
{
	coord_t coord_A,coord_B;
	coord_A=coord_set(22.5730988,113.5782881); //A
	coord_B=coord_set(22.5733928,113.5779696); //B
	polygon_set_AB(  coord_A,  coord_B, 0);

}
 #if 0
{
    mavlink_mission_item_t msg_temp[WAYPOINT_SIZE];//建议做局部变量
    u8 seq_cnt=0;
    coord_t coord_temp[WAYPOINT_SIZE];
    coordNed_t grid_distance,grid_dist_vert,grid_dist_vert_op,grid_distance_op;
    coordNed_t grid_points[WAYPOINT_SIZE];
    coord_t coord_distance2;
    u8 i,j;
    u8 WayPointCnt=0;
    float grid_angle;
	coord_t coord_A,coord_B;
	u8 grid_space=6; //喷洒间距
	bool direction=1; //右侧
//    coord_temp[0]=coord_set(22.5730988,113.5782881); //A
//    coord_temp[1]=coord_set(22.5733928,113.5779696); //B
//    coord_temp[2]=coord_set(22.5734273,113.5780150); //B1
//	grid_distance=convertGeoToNed(coord_temp[0], coord_temp[1]);

	coord_A=coord_set(22.5730988,113.5782881); //A
	coord_B=coord_set(22.5733928,113.5779696); //B
    grid_distance=convertGeoToNed(coord_A, coord_B);
    grid_angle = (atan2(grid_distance.y, grid_distance.x) * M_RAD_TO_DEG);
    printf("grid_angle==%f \r\n",grid_angle);
	grid_dist_vert.x=grid_space*cos((grid_angle-90)*M_DEG_TO_RAD);
	grid_dist_vert.y=grid_space*sin((grid_angle-90)*M_DEG_TO_RAD);
	grid_dist_vert_op.x=-grid_dist_vert.x;
	grid_dist_vert_op.y=-grid_dist_vert.y;
	grid_distance_op.x=-grid_distance.x;
	grid_distance_op.y=-grid_distance.y;
	printf("grid_dist_vertx==%f,grid_dist_verty==%f ,grid_dist_verty==%f\r\n",grid_dist_vert.x,grid_dist_vert.y,grid_dist_vert.z);

	grid_points[1]=grid_distance; //B 点，以A 为起点的绝对坐标

	for (  i = 2 ; i <10 ; i++ )
{
		 switch (i%4)
	{
	  case 0: //v+
	  //奇数点
			{
			grid_points[i].x=grid_points[i-1].x+grid_dist_vert.x;
			grid_points[i].y=grid_points[i-1].y+grid_dist_vert.y;
				  break;
			}
	  case 1: //d+
			{
			grid_points[i].x=grid_points[i-1].x+grid_distance.x;	
			grid_points[i].y=grid_points[i-1].y+grid_distance.y;
				  break;
			}
	  case 2:  // v+
			{
			grid_points[i].x=grid_points[i-1].x+grid_dist_vert.x;
			grid_points[i].y=grid_points[i-1].y+grid_dist_vert.y;
				  break;
			}
	  case 3:  //d-
			{
			grid_points[i].x=grid_points[i-1].x-grid_distance.x;	
			grid_points[i].y=grid_points[i-1].y-grid_distance.y;
				  break;
			}
	}
 printf("grid_pointx==%f,grid_pointy==%f ,grid_pointz==%f\r\n",grid_points[i].x,grid_points[i].y,grid_points[i].z);
		 
}
// transfer geo
	coord_temp[0]=coord_A;
	coord_temp[1]=coord_B;
		for (  i = 2 ; i <10 ; i++ )
{
		 switch (i%4)
	{
	  case 0: //v+
	  //奇数点
			{
		  coord_temp[i]=convertNedToGeo(coord_temp[i-1],grid_dist_vert);
				  break;
			}
	  case 1: //d+
			{
			coord_temp[i]=convertNedToGeo(coord_temp[i-1],grid_distance);
				  break;
			}
	  case 2:  // v+
			{
			coord_temp[i]=convertNedToGeo(coord_temp[i-1],grid_dist_vert);
//			grid_points[i].x=grid_points[i-1].x+grid_dist_vert.x;
//			grid_points[i].y=grid_points[i-1].y+grid_dist_vert.y;
				  break;
			}
	  case 3:  //d-
			{
			coord_temp[i]=convertNedToGeo(coord_temp[i-1],grid_distance_op);
			 break;
			}
	}
 printf("coord_tempx==%f,coord_tempy==%f ,coord_tempz==%f\r\n",coord_temp[i].latitude,coord_temp[i].longitude,coord_temp[i].altitude);

}



		for (  i = 0; i <10 ; i++ )

{
		item_pack( &msg_temp[seq_cnt],
				   seq_cnt,
				   MAV_CMD_NAV_WAYPOINT,				   //			   uint16_t command,
				   0,				   //			   float param1,
				   0,				   //			   float param2,
				   0,				   //			   float param3,
				   0,				   //			   float param4,   angle,只设置第一点即可
				   coord_temp[i].latitude,					   //			   float x,
				   coord_temp[i].longitude, 				   //			   float y,
				   coord_temp[i].altitude					   //			   float z
				 );
		seq_cnt++;	 
}




//    item_pack( &msg_temp[seq_cnt],
//               seq_cnt,
//               MAV_CMD_NAV_WAYPOINT,                   //              uint16_t command,
//               0,                  //              float param1,
//               0,                  //              float param2,
//               0,                  //              float param3,
//               0,                  //              float param4,   angle,只设置第一点即可
//               coord_gloable.altitude,                     //              float x,
//               coord_gloable.longitude,                    //              float y,
//               coord_gloable.altitude                      //              float z
//             );
//    seq_cnt++;


//Start G2018011113141 CY128  $send item
    mavlink_mission_count_t packet;
    packet.count = seq_cnt;
    packet.target_system = 1;
    packet.target_component = 190;
    packet.mission_type = 0;
    _mav_finalize_message_chan_send(MAVLINK_COMM_0, MAVLINK_MSG_ID_MISSION_COUNT, (const char *)&packet, MAVLINK_MSG_ID_MISSION_COUNT_MIN_LEN, MAVLINK_MSG_ID_MISSION_COUNT_LEN, MAVLINK_MSG_ID_MISSION_COUNT_CRC);
    for (  i = 0 ; i <seq_cnt ; i++ )
    {
        _mav_finalize_message_chan_send(MAVLINK_COMM_0, MAVLINK_MSG_ID_MISSION_ITEM, (const char *)&msg_temp[i], MAVLINK_MSG_ID_MISSION_ITEM_MIN_LEN, MAVLINK_MSG_ID_MISSION_ITEM_LEN, MAVLINK_MSG_ID_MISSION_ITEM_CRC);
    }
//End G2018011113141 CY128


}
#endif  //end of MAV_LOG_TSET

