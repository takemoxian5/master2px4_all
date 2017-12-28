
#ifndef _FIFO_H_
#define _FIFO_H_

#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#define true 1
#define false 0

typedef struct _fifo {
	uint8_t* buf;
	uint16_t length;
	uint16_t head;
	uint16_t tail;
} fifo_t;


u8 fifo_read_ch(fifo_t* fifo, uint8_t* ch);
u8 fifo_write_ch(fifo_t* fifo, uint8_t ch);
u16 fifo_free(fifo_t* fifo);
u16 fifo_used(fifo_t* fifo);
void fifo_init(fifo_t* fifo, uint8_t* buf, uint16_t length);

#endif  /*_FIFO_H_*/



