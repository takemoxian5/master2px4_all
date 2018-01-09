
#ifndef __OPENTEL_MAVLINK_H
#define __OPENTEL_MAVLINK_H
#include "usart.h"	
#include "../mavlink_2/common/mavlink.h"
#include "../mavlink_2/common/common.h"

#include "mavlink_types.h"

#include "define.h"

#ifdef __cplusplus
extern "C" {
#endif

void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops);
void update();
#ifdef __cplusplus
}
#endif // __cplusplus
#endif /*__OPENTEL_MAVLINK_H*/   


