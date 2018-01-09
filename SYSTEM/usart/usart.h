#ifndef __USART_H
#define __USART_H
//#include "stm32f4xx_conf.h"
#include "stm32f4xx.h" 
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
#define  Auto_PRINTLOG(x)  {if(test_cntxx[x]!=0xa5){printf("====================%d=finished!\r\n",x);\
	test_cntxx[x]=0;}}
#endif
    
    
    
    
    
void receiverPutChar(uint8_t ch);
void receiverSend(uint8_t *dat,uint8_t size);
    
typedef struct __ReceiverRemote_t
{
    uint32_t cmdReadyFlag; //Rx_Handle_Flag
    uint8_t  cmdIn[10]; //Rx_buff
    int32_t  rxIndex;  //Rx_adr
    int32_t  rxLength; //Rx_length
} ReceiverRemote_t;

extern ReceiverRemote_t myReceiver;
extern  uint8_t size;
extern  uint8_t type;
extern  uint8_t sendBuf[30];
extern uint8_t   startState;
extern uint16_t u16Height;
extern uint16_t u16Distance;
extern uint32_t u32Distance;

extern uint16_t u16Dspeed;
extern int8_t   s8Hspeed;
extern uint8_t  Health;
extern uint16_t  cnt;    
    
    
    
    
    
    
    

//void remote_update(void);

#ifdef __cplusplus
}
#endif // __cplusplus
#endif /* __USART_H */



