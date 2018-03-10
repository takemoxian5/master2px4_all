

#include "OpenTel_Mavlink.h"
#include "fifo.h"
#include "usart.h"
#include "MissionAPI.h"
#include "ff.h"
#include "pwm_out.h"
//Add By BigW
//typedef uint8_t bool;
//#define bool char
//#define true 1
//#define false 0
//typedef int bool;
#ifdef __cplusplus
extern "C" {
#endif
typedef struct
{
    char c;
} prog_char_t;

// This is the state of the flight control system
// There are multiple states defined such as STABILIZE, ACRO,
static int8_t control_mode = STABILIZE;
mavlink_channel_t           chan;
uint16_t                    packet_drops;

mavlink_heartbeat_t         heartbeat;
mavlink_attitude_t          attitude;
mavlink_global_position_int_t position;
	mavlink_ahrs_t              ahrs;



mavlink_vfr_hud_t           	vfr_hud;
mavlink_manual_control_t    	manual_control;
mavlink_sys_status_t        	sys_status;
mavlink_local_position_ned_t 	local_position_ned;
mavlink_mission_count_t 		mission_count;
mavlink_mission_set_current_t 	mission_set_current;
mavlink_mission_current_t 		mission_current;

mavlink_gps_raw_int_t gps_raw_int;
mavlink_rc_channels_t rc_channels;
mavlink_mission_item_int_t mission_item_int;
mavlink_mission_request_t mission_request;
mavlink_mission_ack_t  mission_ack;
mavlink_set_position_target_global_int_t set_position_target_global_int;
mavlink_altitude_t  altitude;

uint8_t buf[100];
//End Add By BigW


// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// this costs us 51 bytes, but means that low priority
// messages don't block the CPU
static mavlink_statustext_t pending_status;

// true when we have received at least 1 MAVLink packet
static char mavlink_active;

// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (payload_space < MAVLINK_MSG_ID_ ## id ## _LEN) return false

void handleMessage(mavlink_message_t* msg);

/*
 *  !!NOTE!!
 *
 *  the use of NOINLINE separate functions for each message type avoids
 *  a compiler bug in gcc that would cause it to use far more stack
 *  space than is needed. Without the NOINLINE we use the sum of the
 *  stack needed for each message type. Please be careful to follow the
 *  pattern below when adding any new messages
 */

static NOINLINE void send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = MAV_STATE_ACTIVE;
    uint32_t custom_mode = control_mode;

    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
    switch (control_mode)
    {
        case AUTO:
        case RTL:
        case LOITER:
        case GUIDED:
        case CIRCLE:
        base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    }

		// all modes except INITIALISING have some form of manual
		// override if stick mixing is enabled
		base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

#if HIL_MODE != HIL_MODE_DISABLED
    base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif

    // we are armed if we are not initialising
    if (0) //motors.armed()) {
    {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    mavlink_msg_heartbeat_send(
        chan,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status);
}

static NOINLINE void send_attitude(mavlink_channel_t chan)
{
    mavlink_msg_attitude_send(
    chan,
    ++buf[1],//millis(),
    ++buf[2],//ahrs.roll,
    ++buf[3],//ahrs.pitch,
    ++buf[4],//ahrs.yaw,
    ++buf[5],//omega.x,
    ++buf[6],//omega.y,
    ++buf[7]);//omega.z);
}

static void NOINLINE send_location(mavlink_channel_t chan)
{
    //Matrix3f rot = ahrs.get_dcm_matrix(); // neglecting angle of attack for now
    mavlink_msg_global_position_int_send(
        chan,
        1,//millis(),
        2,//current_loc.lat,                // in 1E7 degrees
        3,//current_loc.lng,                // in 1E7 degrees
        4,//g_gps->altitude * 10,             // millimeters above sea level
        5,//(current_loc.alt - home.alt) * 10,           // millimeters above ground
        6,//g_gps->ground_speed * rot.a.x,  // X speed cm/s
        7,//g_gps->ground_speed * rot.b.x,  // Y speed cm/s
        8,//g_gps->ground_speed * rot.c.x,
        9);//g_gps->ground_course);          // course in 1/100 degree
}

static void NOINLINE send_ahrs(mavlink_channel_t chan)
{
    //Vector3f omega_I = ahrs.get_gyro_drift();
    mavlink_msg_ahrs_send(
        chan,
        ++buf[8],//omega_I.x,
        ++buf[9],//omega_I.y,
        ++buf[10],//omega_I.z,
        1,
        0,
        ++buf[11],//ahrs.get_error_rp(),
        ++buf[12]);//ahrs.get_error_yaw());
}


static void NOINLINE send_statustext(mavlink_channel_t chan)
{
}

// are we still delaying telemetry to try to avoid Xbee bricking?
static char telemetry_delayed(mavlink_channel_t chan)
{
    return false;
}


// try to send a message, return false if it won't fit in the serial tx buffer
static char mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    int16_t payload_space = serial_free();

    if (telemetry_delayed(chan))
    {
        return false;
    }

    switch(id)
    {
        case MSG_HEARTBEAT:
            CHECK_PAYLOAD_SIZE(HEARTBEAT);
            send_heartbeat(chan);
            break;

        case MSG_ATTITUDE:
            CHECK_PAYLOAD_SIZE(ATTITUDE);
            send_attitude(chan);
            break;

        case MSG_LOCATION:
            CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
            send_location(chan);
            break;

        case MSG_AHRS:
            CHECK_PAYLOAD_SIZE(AHRS);
            send_ahrs(chan);
            break;

        case MSG_STATUSTEXT:
            CHECK_PAYLOAD_SIZE(STATUSTEXT);
            send_statustext(chan);
            break;

        default:
            break;
    }
    return true;
}


#define MAX_DEFERRED_MESSAGES MSG_RETRY_DEFERRED
static struct mavlink_queue
{
    enum ap_message deferred_messages[MAX_DEFERRED_MESSAGES];
    uint8_t next_deferred_message;
    uint8_t num_deferred_messages;
} mavlink_queue[2];

// send a message using mavlink
void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    uint8_t i, nextid;
    struct mavlink_queue *q = &mavlink_queue[(uint8_t)chan];

    // see if we can send the deferred messages, if any
    while (q->num_deferred_messages != 0)
    {
        if (!mavlink_try_send_message(chan,
                                      q->deferred_messages[q->next_deferred_message],
                                      packet_drops))
        {
            break;
        }
        q->next_deferred_message++;
        if (q->next_deferred_message == MAX_DEFERRED_MESSAGES)
        {
            q->next_deferred_message = 0;
        }
        q->num_deferred_messages--;
    }

    if (id == MSG_RETRY_DEFERRED)
    {
        return;
    }

    // this message id might already be deferred
    for (i=0, nextid = q->next_deferred_message; i < q->num_deferred_messages; i++)
    {
        if (q->deferred_messages[nextid] == id)
        {
            // its already deferred, discard
            return;
        }
        nextid++;
        if (nextid == MAX_DEFERRED_MESSAGES)
        {
            nextid = 0;
        }
    }

    if (q->num_deferred_messages != 0 ||
        !mavlink_try_send_message(chan, id, packet_drops))
    {
        // can't send it now, so defer it
        if (q->num_deferred_messages == MAX_DEFERRED_MESSAGES)
        {
            // the defer buffer is full, discard
            return;
        }
        nextid = q->next_deferred_message + q->num_deferred_messages;
        if (nextid >= MAX_DEFERRED_MESSAGES)
        {
            nextid -= MAX_DEFERRED_MESSAGES;
        }
        q->deferred_messages[nextid] = id;
        q->num_deferred_messages++;
    }
}

void mavlink_send_text(mavlink_channel_t chan, enum gcs_severity severity, char *str)
{
    if (telemetry_delayed(chan))
    {
        return;
    }

    if (severity == SEVERITY_LOW)
    {
        // send via the deferred queuing system
        pending_status.severity = (uint8_t)severity;
        mav_array_memcpy((char *)pending_status.text, str, sizeof(pending_status.text));
        mavlink_send_message(chan, MSG_STATUSTEXT, 0);
    }
    else
    {
        // send immediately
        mavlink_msg_statustext_send(
            chan,
            severity,
            str);
    }
}
void pwm_check_AB(u16 seq, u8 grid_pwm)
{
if(seq%2==0)   //偶数点不喷
SetPwm(0);
else if(seq%2==1)                                                                                                                                      
{
SetPwm(grid_pwm);
}
SetPwm(80);

}
u8 retrun_check_AB_flag=0;
void retrun_check_AB(u16 seq, u16 voltage_battery)
{
    mavlink_command_long_t mission_start= {0};

    mission_start.target_system=1;
    mission_start.target_component=0;
    mission_start.confirmation=true;
    mission_start.command=MAV_CMD_MISSION_START;
    mission_start.param1=0;
    mission_start.param2=5;
if(voltage_battery>42500)return;
	if(seq%4==2&&retrun_check_AB_flag==0)// A 点附近  2. 6. 10
	{
    retrun_check_AB_flag=1;
    mavlink_msg_command_long_send_struct(MAVLINK_COMM_0,&mission_start);
	}
}

void update(void)
{
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
    status.packet_rx_drop_count = 0;
	pwm_check_AB(mission_current.seq,  grid_pwm);

    // process received bytes
    while(serial_available())
    {
        uint8_t c = serial_read_ch();

        // Try to get a new message
        if (mavlink_parse_char(chan, c, &msg, &status))
        {
            mavlink_active = true;
            handleMessage(&msg);
//            if(
//				msg.msgid!=MAVLINK_MSG_ID_HEARTBEAT
//               &&msg.msgid!=MAVLINK_MSG_ID_SYS_STATUS
//               &&msg.msgid!=MAVLINK_MSG_ID_ATTITUDE
//               &&msg.msgid!=MAVLINK_MSG_ID_VFR_HUD //BATTERY
//               &&msg.msgid!=MAVLINK_MSG_ID_SERVO_OUTPUT_RAW
//               &&msg.msgid!=147 //BATTERY
//               &&msg.msgid!=2 //sys time
////               &&msg.msgid!=65 //rc
//               &&msg.msgid!=70 //rc
//               &&msg.msgid!=245 //extended
//               &&msg.msgid!=231 //wind
//               &&msg.msgid!=241 //vibration
//               &&msg.msgid!=230 //estimator
//               &&msg.msgid!=141 //altitude
//               &&msg.msgid!=24 //gps_raw_int
//               &&msg.msgid!=42 //mission CURRENT
//               &&msg.msgid!=77 //COMMAND_ACK
//              )
                printf("new msg msgid======+========================%d====get!\r\n",msg.msgid);

        }
		else
			{
//			printf("error!%d\r\n",msg.msgid);
			}
    }
}
char  sd_data[20];//char  textFileBuffer2[40];
char  file_name_path[40];
char* file_name="item.bin";
char* file_path="/YX128";
FRESULT result;
FATFS fs;
DIR DirInf;
//FILINFO FileInf;
FIL file;
uint32_t bw;
u8 save_data()
{
#if 1
#if  0
    /* 挂载文件系统 */
    result = f_mount(0, &fs);           /* Mount a logical drive */
    if (result != FR_OK)
    {
        /* 如果挂载不成功，进行格式化 */
        result = f_mkfs("0:",0,0);
        if (result != FR_OK)
        {
            return -1;     //flash有问题，无法格式化
        }
        else
        {
            /* 重新进行挂载 */
            result = f_mount(0, &fs);           /* Mount a logical drive */
            if (result != FR_OK)
            {
                /* 卸载文件系统 */
                f_mount(0, NULL);
                return -2 ;
            }
        }
    }

    /* 打开根文件夹 */
    result = f_opendir(&DirInf, "/");
    if (result != FR_OK)
    {
        /* 卸载文件系统 */
        f_mount(0, NULL);
        return -3;
    }

    /* 打开文件 */
    result = f_open(&file, file_name, FA_CREATE_ALWAYS | FA_WRITE);
    if (result !=  FR_OK)
    {
        /* 卸载文件系统 */
        f_mount(0, NULL);
        return -4;
    }

    /* 写入Sensor配置文件 */
    result = f_write(&file, &sd_data, 4, &bw);
    if (result == FR_OK)
    {
        /* 关闭文件*/
        f_close(&file);
        /* 打开文件 */
        result = f_open(&file, file_name, FA_CREATE_ALWAYS | FA_WRITE);
        if (result !=  FR_OK)
        {
            /* 卸载文件系统 */
            f_mount(0, NULL);
            return -4;
        }

    }
#endif  //end of MAV_LOG_TSET
    /* 挂载文件系统 */
    result = f_mount(0, &fs);           /* Mount a logical drive */
    if (result != FR_OK)
    {
        printf("挂载文件系统失败 (%d)\r\n", result);
    }
    /* 创建目录/  */
//            result = f_mkdir(file_path);
    result=f_mount(0, &fs);
    /* 打开根文件夹 */
    result = f_opendir(&DirInf, "/"); /* 如果不带参数，则从当前目录开始 */
//            result = f_opendir(&DirInf, file_path); /* 如果不带参数，则从当前目录开始 */
    if (result != FR_OK)
    {
        printf("打开根目录失败 (%d)\r\n", result);
    }
//          sprintf( file_name_path,    "%s%s",file_path,file_name);
//            printf("打开目录 (%s)\r\n", file_name_path);
    result = f_open(&file, file_name, FA_OPEN_ALWAYS | FA_WRITE);
    /* 写一串数据 */
//          result = f_lseek (&fil, fil.fsize);  ////指针指向文件末尾

    result = f_write(&file, &sd_data, 4, &bw);


    /* 关闭文件*/
    f_close(&file);
    /* 卸载文件系统 */
    f_mount(0, NULL);
    return 1 ;
#endif  //end of MAV_LOG_TSET
}



u8 read_data()
{
    /* 挂载文件系统 */
    result = f_mount(0, &fs);           /* Mount a logical drive */
    if (result != FR_OK)
    {
        printf("挂载文件系统失败 (%d)\r\n", result);
    }
    /* 创建目录/  */
    //            result = f_mkdir(file_path);
    result=f_mount(0, &fs);
    /* 打开根文件夹 */
    result = f_opendir(&DirInf, "/"); /* 如果不带参数，则从当前目录开始 */
    //            result = f_opendir(&DirInf, file_path); /* 如果不带参数，则从当前目录开始 */
    if (result != FR_OK)
    {
        printf("打开根目录失败 (%d)\r\n", result);
    }
    //          sprintf( file_name_path,    "%s%s",file_path,file_name);
    //            printf("打开目录 (%s)\r\n", file_name_path);
    result = f_open(&file, file_name, FA_OPEN_EXISTING | FA_READ);
    result = f_read(&file, sd_data, 4,&bw);
    /* 关闭文件*/
    f_close(&file);
    /* 卸载文件系统 */
    f_mount(0, NULL);
    return 1 ;

#if 0
    /* 挂载文件系统 */
    result = f_mount(0, &fs);           /* Mount a logical drive */
    if (result != FR_OK)
    {
        /* 如果挂载不成功，进行格式化 */
        result = f_mkfs("0:",0,0);
        if (result != FR_OK)
        {
            return -1;     //flash有问题，无法格式化
        }
        else
        {
            /* 重新进行挂载 */
            result = f_mount(0, &fs);           /* Mount a logical drive */
            if (result != FR_OK)
            {
                /* 卸载文件系统 */
                f_mount(0, NULL);
                return -2 ;
            }
        }
    }

    /* 打开根文件夹 */
    result = f_opendir(&DirInf, "/");
    if (result != FR_OK)
    {
        /* 卸载文件系统 */
        f_mount(0, NULL);
        return -3;
    }

    /* 打开文件 */
    result = f_open(&file, file_name, FA_OPEN_EXISTING | FA_READ);
    if (result !=  FR_OK)
    {
        /* 卸载文件系统 */
        f_mount(0, NULL);
        /* 文件不存在 */
        return -4;
    }
    /* 读取文件 */

    if(bw > 0)
    {
        result = f_read(&file, sd_data, 4,&bw);
        /* 关闭文件*/
        f_close(&file);
        /* 卸载文件系统 */
        f_mount(0, NULL);
        return 1;
    }
    else
    {
        /* 关闭文件*/
        f_close(&file);
        /* 卸载文件系统 */
        f_mount(0, NULL);
        return -4;
    }
    /* 关闭文件*/
    f_close(&file);
    /* 卸载文件系统 */
    f_mount(0, NULL);
    return -5;
#endif  //end of MAV_LOG_TSET


}
///for rc
#define PPM_ZERO_CENTRE 1514  //平衡零点
#define PPM_ZERO 982
#define PPM_RANGE 851 //820
#define PPM_MIN 173
#define PPM_MAX 2046
#define PPM_RANGE_2 PPM_RANGE+PPM_RANGE //820
uint16_t current_seq=0;
u8 test_flag_chan9=0;
u8 test_flag_chan10=0;

u8 test_flag_chan11=0;
u8 test_flag_chan12=0;
u8 test_flag_chan16=0;

u16 key_safe_last=0;
u16 key_again=0;
typedef struct __grid_config_s
{
  u8 gubGridSpace; //喷洒间距
  u8 grid_pwm;
  u8 grid_speed;
  float grid_angle;

} grid_config_s;

grid_config_s grid_config;

typedef struct __smart_item_s
{
    u8 last;
    u8 current;

} smart_item_s;

smart_item_s smart_item;
u8 file_result;
typedef struct ___ab_mode_s
{
    u8 last;
    u8 current;
} ab_mode_s;

ab_mode_s ab_mode;
typedef enum AB_MODE_FLAG
{
//   MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1, /* 0b00000001 Reserved for future use. | */
AB_MODE_FLAG_N=0,
AB_MODE_FLAG_A=1,
AB_MODE_FLAG_B=2,
} AB_MODE_FLAG;

void handleMessage(mavlink_message_t* msg)
{
    mavlink_command_long_t take_off_local= {0};
    take_off_local.target_system=1;
    take_off_local.target_component=0;
    take_off_local.confirmation=true;
    take_off_local.command=MAV_CMD_NAV_TAKEOFF_LOCAL;
    take_off_local.param1=0;
    take_off_local.param3=0;
    take_off_local.param4=0;
    take_off_local.param5=0;
    take_off_local.param6=0;
    take_off_local.param7=5;
    mavlink_command_long_t mission_start= {0};

    mission_start.target_system=1;
    mission_start.target_component=0;
    mission_start.confirmation=true;
    mission_start.command=MAV_CMD_MISSION_START;
    mission_start.param1=0;
    mission_start.param2=5;

    //struct Location tell_command = {};                                  // command for telemetry
    switch (msg->msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            mavlink_msg_heartbeat_decode(msg, &heartbeat);
       if(coord_gloableHome.altitude==0)
        coord_gloableHome = coord_set((((double)gps_raw_int.lat )/10000000),(((double)gps_raw_int.lon )/10000000),(((double)gps_raw_int.alt )/1000));
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE://机体运动姿态，俯仰 横滚大小 和速度
        {
            mavlink_msg_attitude_decode(msg, &attitude);
            break;
        }

        case MAVLINK_MSG_ID_SYS_STATUS:
        {
            mavlink_msg_sys_status_decode( msg, &sys_status);
            break;
        }
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        {
            mavlink_msg_local_position_ned_decode( msg, &local_position_ned);
            printf("local_position_ned===%f\r\n",local_position_ned.z);
            break;
        }
        case MAVLINK_MSG_ID_RC_CHANNELS:
        {
            mavlink_msg_rc_channels_decode( msg, &rc_channels);


            if(rc_channels.chan5_raw==PPM_ZERO)  //A  982
            {
                coord_gloableA = coord_set((((double)gps_raw_int.lat )/10000000),(((double)gps_raw_int.lon )/10000000),(((double)gps_raw_int.alt )/1000));
				if(altitude.altitude_relative>1)
				coord_gloableA.altitude=altitude.altitude_relative;//altitude_terrain
				else
	                coord_gloableA.altitude-=coord_gloableHome.altitude;//换为相对高度
                if(coord_gloableA.altitude>35)coord_gloableA.altitude=35;
                if(coord_gloableA.altitude<4)coord_gloableA.altitude=4;

//coord_gloableA.altitude=local_position_ned.z;
                fight_angle=attitude.yaw*M_RAD_TO_DEG;
            }
            else if(rc_channels.chan5_raw==PPM_MAX)//B  2046
            {
                coord_gloableB= coord_set((((double)gps_raw_int.lat )/10000000),(((double)gps_raw_int.lon )/10000000),(((double)gps_raw_int.alt )/1000));

				if(altitude.altitude_relative>1)
				coord_gloableB.altitude=altitude.altitude_relative;//altitude_terrain
				else
	            coord_gloableB.altitude-=coord_gloableHome.altitude;//换为相对高度
                if(coord_gloableB.altitude>35)coord_gloableB.altitude=35;
                if(coord_gloableB.altitude<4)coord_gloableB.altitude=4;
//coord_gloableB.altitude=local_position_ned.z;
            }

            if(rc_channels.chan7_raw  !=key_safe_last)  //A  982
            {
                if(coord_gloableHome.altitude==0)
                    coord_gloableHome = coord_set((((double)gps_raw_int.lat )/10000000),(((double)gps_raw_int.lon )/10000000),(((double)gps_raw_int.alt )/1000));
            }
            key_safe_last=rc_channels.chan7_raw;
            if(rc_channels.chan11_raw==PPM_MAX)  //c1  2046
            {
                test_flag_chan11=1;
            }
            else if(test_flag_chan11==1)
            {
                test_flag_chan11=0;
                polygon_set_AB(  coord_gloableA,  coord_gloableB, 0);
				
                printf(" c1 get\r\n");
            }
            if(rc_channels.chan12_raw==PPM_MAX)    //c2 2046
            {
                test_flag_chan12=1;
            }
            else if(test_flag_chan12==1)
            {
                test_flag_chan12=0;
                polygon_set_AB(  coord_gloableA,  coord_gloableB, 1);
                printf(" c2 get\r\n");
            }
            switch (rc_channels.chan8_raw)
            {
                case PPM_ZERO_CENTRE:
                    if(key_again==0)
                    {
                        key_again++;
                        smart_item.last=mission_current.seq;
//                  smart_item.current++;
                        sd_data[0]= smart_item.last ;
                        coord_gloableLast= coord_set((((double)gps_raw_int.lat )/10000000),(((double)gps_raw_int.lon )/10000000),(((double)gps_raw_int.alt )/1000));
                        coord_gloableLast.altitude-=coord_gloableHome.altitude;//换为相对高度
                        file_result= save_data();
                        printf(" file_result=%d  smart_item get\r\n",file_result);
                    }
                    break;
                default:
                    //if(key_again)
                    key_again=0;
                    break;
            }

            if(rc_channels.chan10_raw==1622)  //流量  982 1622 1747 1862 2006
            {
//                printf("OUT %d \r\n",rc_channels.chan10_raw);
//          mavlink_msg_set_mode_send(MAVLINK_COMM_0, 1, 89, 89);
				grid_pwm=30;

                if(test_flag_chan10==0)
                {
                    test_flag_chan10=1;
//              take_off_local.param5=gps_raw_int.lat;
//  			take_off_local.param6=gps_raw_int.lon;
//      		mavlink_msg_command_long_send_struct(MAVLINK_COMM_0,&take_off_local);
                }

//          current_seq= (rc_channels.chan10_raw-PPM_ZERO)/200;
//          mavlink_msg_mission_set_current_send(MAVLINK_COMM_0,  1, 190, current_seq);
            }
            else if(rc_channels.chan10_raw==1747)  //流量  982 1622 1747 1862 2006
            {
            grid_pwm=60;
//          	  mavlink_msg_mission_set_current_send(MAVLINK_COMM_0,  1, 190, 2);
//                mission_start.command=MAV_CMD_COMPONENT_ARM_DISARM;   //解锁  无法强制进行
//                mavlink_msg_command_long_send_struct(MAVLINK_COMM_0,&mission_start);
//                mission_start.command=MAV_CMD_MISSION_START;
#ifdef open_position
			set_position_target_global_int.target_system=1;
			set_position_target_global_int.target_component=190;
			set_position_target_global_int.coordinate_frame=MAV_FRAME_GLOBAL_INT;//MAV_FRAME_LOCAL_NED;
			set_position_target_global_int.lat_int=coord_gloableLast.latitude;
			set_position_target_global_int.lon_int=coord_gloableLast.longitude;
			set_position_target_global_int.alt=coord_gloableLast.altitude;

			mavlink_msg_set_position_target_global_int_send_struct(MAVLINK_COMM_0,&set_position_target_global_int);
#endif

            }
            else if(rc_channels.chan10_raw==1862)  //流量  982 1622 1747 1862 2006
            {
            grid_pwm=90;
            //    mavlink_msg_command_long_send_struct(MAVLINK_COMM_0,&mission_start);
//          mavlink_msg_set_mode_send(MAVLINK_COMM_0, 1, 89, 89);
            }
            else if(rc_channels.chan10_raw==982)  //流量  982 1622 1747 1862 2006
            {
            grid_pwm=0;
//          mavlink_msg_mission_set_current_send(MAVLINK_COMM_0,  1, 190, 0);
                test_flag_chan10=0;
            }

			
            if(rc_channels.chan9_raw==PPM_MAX)  //test   1键开始任务
            {
                test_flag_chan9=1;
//                waypoint_test();
            }
            else if(test_flag_chan9==1)  //按两次
            {
                test_flag_chan9=0;
                //  mavlink_msg_set_mode_send(MAVLINK_COMM_0, 1, 157, 157);//开始任务 9D
                if(heartbeat.system_status==MAV_STATE_ACTIVE)//起飞状态才执行
                    mavlink_msg_command_long_send_struct(MAVLINK_COMM_0,&mission_start);
            }
            if(rc_channels.chan16_raw==PPM_MAX)  //test   1键开始任务
            {

                if(test_flag_chan16==0)
                {
                    sd_data[0]=0;
                    file_result=read_data();
                    printf(" file_result=%d     \r\n",file_result);
                    smart_item.last=sd_data[0];
                    printf("last %d \r\n",smart_item.last);
//                    mission_start.param1=smart_item.last;
					mission_start.command=MAV_CMD_MISSION_START;
                    //  mavlink_msg_set_mode_send(MAVLINK_COMM_0, 1, 157, 157);//开始任务 9D=
                    if(heartbeat.system_status==MAV_STATE_ACTIVE)//起飞状态才执行
{
						mission_start.command=45;  //clear
						mavlink_msg_command_long_send_struct(MAVLINK_COMM_0,&mission_start);
						mission_start.command=MAV_CMD_MISSION_START;
						mission_start.param1=smart_item.last;
						mission_start.param2=smart_item.last+5;
                        mavlink_msg_command_long_send_struct(MAVLINK_COMM_0,&mission_start);
                        mission_start.param1=0;
}
                }
                test_flag_chan16++;
            }
            else
            {
                test_flag_chan16=0;
            }
            break;
        }
        case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
        {
            mavlink_msg_mission_set_current_decode( msg, &mission_set_current);
            break;
        }
        case MAVLINK_MSG_ID_MISSION_CURRENT:
        {
            mavlink_msg_mission_current_decode( msg, &mission_current);
//          printf("mission_current===%d\r\n",mission_current.seq);
            break;
        }
        case MAVLINK_MSG_ID_MISSION_COUNT:
        {
            mavlink_msg_mission_count_decode( msg, &mission_count);
            break;
        }
        case MAVLINK_MSG_ID_GPS_RAW_INT:
        {
            mavlink_msg_gps_raw_int_decode( msg, &gps_raw_int);
            break;
        }
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        {
            mavlink_msg_mission_item_int_decode( msg, &mission_item_int);
            break;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST:
        {
            mavlink_msg_mission_request_decode( msg, &mission_request);
		if(check_need_flag)
			{
			check_need_flag=0;
			if(gubMissionTypeCnt!=mission_request.mission_type)//校验不对
				set

		}
//            printf("mission_request===%d	%d	  %d\r\n",mission_request.seq,mission_request.target_component,mission_request.mission_type);
            break;
        }
        case    MAVLINK_MSG_ID_MISSION_ACK:
        {
            mavlink_msg_mission_ack_decode(msg, &mission_ack);
//    mission_ack->target_system = mavlink_msg_mission_ack_get_target_system(msg);
//    mission_ack->target_component = mavlink_msg_mission_ack_get_target_component(msg);
//    mission_ack->type = mavlink_msg_mission_ack_get_type(msg);
//    mission_ack->mission_type = mavlink_msg_mission_ack_get_mission_type(msg);
//    mission_ack.type   MAV_MISSION_ACCEPTED=0,MAV_MISSION_ERROR=1
            printf("target_system===%d %d %d\r\n",mission_ack.mission_type,mission_ack.target_component,mission_ack.type);
            break;
        }
		case MAVLINK_MSG_ID_ALTITUDE:
		{
				mavlink_msg_altitude_decode(msg, &altitude);
				break;
		}
#if 0
				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					mavlink_msg_global_position_int_decode(msg, &position);
					break;
				}
				case MAVLINK_MSG_ID_AHRS:
				{
					mavlink_msg_ahrs_decode(msg, &ahrs);
					break;
				}
				case MAVLINK_MSG_ID_VFR_HUD:   //机身速度和角度
				{
					mavlink_msg_vfr_hud_decode(msg,&vfr_hud);
						if(vfr_hud.groundspeed)
						{
						}
					break;
				}
				case MAVLINK_MSG_ID_MANUAL_CONTROL:
				{
					mavlink_msg_manual_control_decode(msg, &manual_control);
					break;
				}
#endif  //end of next_func

        default:
            break;
    }     // end switch
	
	retrun_check_AB(mission_current.seq,  sys_status.voltage_battery);

} // end handle mavlink

#ifdef __cplusplus
}
#endif // __cplusplus
