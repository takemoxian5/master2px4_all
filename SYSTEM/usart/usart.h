#ifndef __USART_H
#define __USART_H
//#include "stm32f4xx_conf.h"
#include "stm32f4xx.h" 
#include "cppforstm32.h"
//#include "stm32f0xx.h"
#include <stdio.h>
#include "mavlink_types.h"
#ifdef __cplusplus
extern "C" {
#endif
#define UART_TX_BUFFER_SIZE        255
#define UART_RX_BUFFER_SIZE        255
void serial_open(uint8_t port, uint32_t baud);
u8 serial_write_buf(uint8_t* buf, uint16_t length);
u8 serial_read_ch(void);
u16 serial_free(void);
u16 serial_available(void);
void UART_send_byte(uint8_t byte);
void comm_send_ch(mavlink_channel_t chan, uint8_t ch);

#define DEBUG_SEND_MSG
#ifdef DEBUG_SEND_MSG
extern char  test_cntxx[20];
#define  Auto_PRINTLOG(x)  {{printf("====================%d=finished!\r\n",x);\
	test_cntxx[x]=5;}}
#endif
#ifdef __cplusplus
}
#endif // __cplusplus
#endif /* __USART_H */



