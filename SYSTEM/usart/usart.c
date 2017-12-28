#include "usart.h"	

#include "fifo.h"
#include "stdio.h"
fifo_t uart_rx_fifo, uart_tx_fifo;
uint8_t uart_tx_buf[UART_TX_BUFFER_SIZE], uart_rx_buf[UART_RX_BUFFER_SIZE];

///* Private function prototypes -----------------------------------------------*/
//#ifdef __GNUC__
//  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
//     set to 'Yes') calls __io_putchar() */
//  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif /* __GNUC__ */
////////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  

#ifdef __cplusplus
extern "C" {
#endif
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
    USART_SendData(USART1, (unsigned char) ch);

while (!(USART1->SR & USART_FLAG_TXE));

return (ch);
//	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
//	USART1->DR = (u8) ch;      
//	return ch;
}
#else
//#ifdef __GNUC__
//  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
//     set to 'Yes') calls __io_putchar() */
//  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif /* __GNUC__ */


//PUTCHAR_PROTOTYPE 
//{
//  /* 将Printf内容发往串口 */
//  USART_SendData(USART1, (uint8_t)ch);
//  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
// 
//  return (ch);
//}
#endif
  
/* Private functions ---------------------------------------------------------*/

void USART1_Gpio_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);     //tx
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);     //rx

}

void USART2_Gpio_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);     //tx
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);     //rx

}

void USART3_Gpio_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);        //tx
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);        //rx
}

/*
 * USART1 is used for receiving commands from PC and
 * printing debug information to PC
 */
void USART1_Config(void)
{
  USART1_Gpio_Config();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART1, &USART_InitStructure);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  USART_Cmd(USART1, ENABLE);

  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) != SET)
    ;
}

/*
 * USART2 is used for receiving commands from PC and
 * printing debug information to PC
 */
void USART2_Config(void)
{
  USART2_Gpio_Config();

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = 921600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART2, &USART_InitStructure);
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

  USART_Cmd(USART2, ENABLE);

  while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) != SET)
    ;
}

/*
 * USART3 is used for communicating with the DJI flight controller
 * The Baud rate needs to match the Baud rate used by the flight controller
 */
void USART3_Config(void)
{
  USART3_Gpio_Config();

  USART_InitTypeDef USART_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    USART_InitStructure.USART_BaudRate = 57600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART3, &USART_InitStructure);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

  USART_Cmd(USART3, ENABLE);
  while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) != SET)
    ;

}

void USARTxNVIC_Config()
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  NVIC_InitTypeDef NVIC_InitStructure_USART3;
  NVIC_InitStructure_USART3.NVIC_IRQChannelPreemptionPriority = 0x02;
  NVIC_InitStructure_USART3.NVIC_IRQChannelSubPriority = 0x02;
  NVIC_InitStructure_USART3.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure_USART3.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure_USART3);

  NVIC_InitTypeDef NVIC_InitStructure_USART2;
  NVIC_InitStructure_USART2.NVIC_IRQChannelPreemptionPriority = 0x03;
  NVIC_InitStructure_USART2.NVIC_IRQChannelSubPriority = 0x02;
  NVIC_InitStructure_USART2.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure_USART2.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure_USART2);
	
  NVIC_InitTypeDef NVIC_InitStructure_USART1;
  NVIC_InitStructure_USART1.NVIC_IRQChannelPreemptionPriority = 0x04;
  NVIC_InitStructure_USART1.NVIC_IRQChannelSubPriority = 0x04;
  NVIC_InitStructure_USART1.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure_USART1.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure_USART1);	
}

void UsartConfig()
{
	USART1_Config();
  USART2_Config();
  USART3_Config();
  USARTxNVIC_Config();
}
void UART_send_byte(uint8_t byte) 							//发送1字节数据
{
 while(!((USART2->SR)&(1<<7)));
 USART2->DR=byte;	
}		
 void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    
UART_send_byte(ch);
//    if (chan == MAVLINK_COMM_0)
//    {
//        uart0_transmit(ch);
//    }
//    if (chan == MAVLINK_COMM_1)
//    {
//    	uart1_transmit(ch);
//    }
}
//void UART_Send(uint8_t *Buffer, uint32_t Length)
//{
//	while(Length != 0)
//	{
//		while(!((USART2->SR)&(1<<7)));							//等待发送完
//		USART2->DR = *Buffer;
//		Buffer++;
//		Length--;
//	}
//}

//uint8_t UART_Recive(void)
//{	
//	while(!(USART2->SR & (1<<5)));								//等待接收到数据
//	return(USART2->DR);			 										//读出数据
//}





//========================================================================================================
//串口封装成数据缓存区
//========================================================================================================
/** @brief 串口初始化
  *        1) 定义两个数据缓冲区，一个用于发射，一个用于接收
  *        2) 初始化串口波特率，全双工，使能接收中断(发射中断，在有数据发射后使能)
  */
void serial_open(uint8_t port, uint32_t baud) {
	fifo_init(&uart_tx_fifo, uart_tx_buf, UART_TX_BUFFER_SIZE);	
	fifo_init(&uart_rx_fifo, uart_rx_buf, UART_RX_BUFFER_SIZE);
	
  UsartConfig();
}


/** @brief 写数据到串口，启动发射
  *        
  * @note 数据写入发射缓冲区后，启动发射中断，在中断程序，数据自动发出
  */
uint8_t serial_write_buf(uint8_t* buf, uint16_t length)
{
    uint16_t i;

    if(length == 0) return false;
    for(i = 0; length > 0; length--, i++)
    {
        fifo_write_ch(&uart_tx_fifo, buf[i]);
    }
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);

    return true;
}

/** @brief 自串口读数据 
  * @return 一字节数据
  */
uint8_t serial_read_ch(void){
	uint8_t ch;	
	fifo_read_ch(&uart_rx_fifo, &ch);	
	return ch;
}

/** @breif 检测发射缓冲区剩余字节长度 
  * @return 剩余字节长度
  */
uint16_t serial_free(void){
	return fifo_free(&uart_tx_fifo);
}

uint16_t serial_available(void){
	return fifo_used(&uart_rx_fifo);
}
#ifdef Add_remote
/** @brief 写数据到串口，启动发射
  *
  * @note 数据写入发射缓冲区后，启动发射中断，在中断程序，数据自动发出
  */
uint8_t serial_write_buf2(uint8_t* buf, uint16_t length)
{
    uint16_t i;

    if(length == 0) return false;
    for(i = 0; length > 0; length--, i++)
    {
        fifo_write_ch(&remote_tx_fifo, buf[i]);
    }
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);

    return true;
}

/** @brief 自串口读数据
  * @return 一字节数据
  */
uint8_t serial_read_ch2(void)
{
    uint8_t ch;
    fifo_read_ch(&remote_rx_fifo, &ch);
    return ch;
}



uint16_t serial_available2(void)
{
    return fifo_used(&remote_rx_fifo);
}
uint16_t serial_free2(void)
{
    return fifo_free(&remote_tx_fifo);
}

#endif

//void USART2_IRQHandler(void)
//{
//  uint8_t c;
//
//  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
//  {
//      c = USART2->DR;
//      fifo_write_ch(&uart_rx_fifo, c);
//
//    //USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
//  }

//  if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
//  {
//      if(fifo_read_ch(&uart_tx_fifo, &c)) USART2->DR = c;
//      else USART2->DR = 0x55;
//
//    if (fifo_used(&uart_tx_fifo) == 0)              // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
//    {
//      // Disable the EVAL_COM1 Transmit interrupt
//      USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
//    }
//  }
//}
typedef struct __ReceiverRemote_t
{
    uint32_t cmdReadyFlag; //Rx_Handle_Flag
    uint8_t  cmdIn[10]; //Rx_buff
    int32_t  rxIndex;  //Rx_adr
    int32_t  rxLength; //Rx_length
} ReceiverRemote_t;
ReceiverRemote_t myReceiver;
static uint8_t size=0;
static uint8_t type=0;
uint8_t sendBuf[30];
uint8_t   startState=0;
uint16_t u16Height=0;
uint16_t u16Distance=0;
uint32_t u32Distance=0;

uint16_t u16Dspeed=0;
int8_t   s8Hspeed=0;
uint8_t  Health;
uint16_t  cnt;
void uart2_putchar(unsigned char ch)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
        ;
    USART_SendData(USART2, (unsigned char) ch);
}

void receiverPutChar(uint8_t ch)
{
    uart2_putchar(ch);
}

void receiverSend(uint8_t *dat,uint8_t size)
{
    uint8_t i=0;
    for(i=0; i<size; i++)
    {
        uart2_putchar(dat[i]);
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
//            memcpy(&sendBuf[2],&u16Height,2);
//            memcpy(&sendBuf[4],&u16Distance,2);
//            memcpy(&sendBuf[6],&u16Dspeed,2);
//            memcpy(&sendBuf[8],&s8Hspeed,1);
//            memcpy(&sendBuf[9],&Health,1);
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
#ifdef Add_remote_debug
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






void USART2_IRQHandler(void)
{
#if 0
    uint8_t c;

    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        c = USART2->DR;
        fifo_write_ch(&remote_rx_fifo, c);

        //USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    }

    if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
    {
        if(fifo_read_ch(&remote_tx_fifo, &c)) USART2->DR = c;
        else USART2->DR = 0x55;

        if (fifo_used(&remote_tx_fifo) == 0)            // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
        {
            // Disable the EVAL_COM1 Transmit interrupt
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        }
    }
#endif  //end of Add_remote

    if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET)
    {
        uint8_t oneByte = USART_ReceiveData(USART2);
        if (myReceiver.rxIndex == 0)
        {
            if (oneByte == 0xFA)
            {
                myReceiver.cmdIn[myReceiver.rxIndex] = oneByte;
                myReceiver.rxIndex = 1;
            }
        }
        else
        {
            if (oneByte == 0xFE)    //receive a 0xFE would lead to a command-execution
            {
                myReceiver.cmdIn[myReceiver.rxIndex] = oneByte;
                myReceiver.rxLength = myReceiver.rxIndex + 1;
                myReceiver.rxIndex = 0;
                myReceiver.cmdReadyFlag = 1;
            }
            else
            {
                myReceiver.cmdIn[myReceiver.rxIndex] = oneByte;
                myReceiver.rxIndex++;
            }
        }
    }
}

void USART3_IRQHandler(void)
{
    uint8_t c;

    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        c = USART3->DR;
        fifo_write_ch(&uart_rx_fifo, c);

        //USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    }

    if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
    {
        if(fifo_read_ch(&uart_tx_fifo, &c)) USART3->DR = c;
        else USART3->DR = 0x55;

        if (fifo_used(&uart_tx_fifo) == 0)              // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
        {
            // Disable the EVAL_COM1 Transmit interrupt
            USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
        }
    }
}

#ifdef __cplusplus
}
#endif // __cplusplus
