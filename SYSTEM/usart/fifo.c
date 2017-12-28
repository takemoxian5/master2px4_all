
#include "fifo.h"


/** @brief ��FIFO
  * @param fifo ����������
	*        *ch   ����������
	* @return 
	*        ��ȷ��ȡ��1�� �����ݣ�0
  */
u8 fifo_read_ch(fifo_t* fifo, uint8_t* ch)
{
	if(fifo->tail == fifo->head) return false;
	*ch = fifo->buf[fifo->tail];  
	
	if(++fifo->tail >= fifo->length) fifo->tail = 0;
  return true;
}


/** @brief дһ�ֽ����ݵ�FIFO
  * @param fifo ��д�뻺����
	*        ch   ��д�������
	* @return 
	*        ��ȷ��1�� ����������0
  */
u8 fifo_write_ch(fifo_t* fifo, uint8_t ch)
{
	uint16_t h = fifo->head;
	
	if(++h >= fifo->length) h = 0;
	if(h == fifo->tail) return false;
	
	fifo->buf[fifo->head] = ch;
	fifo->head = h;
  return true;
}


/** @brief ���ػ�����ʣ���ֽڳ���
  * @param fifo 
	* @return 
	*        ʣ��ռ�
  *
  * @note  ʣ���ֽڳ��ȴ��ڵ���2ʱ���ſ�д������
  */
u16 fifo_free(fifo_t* fifo)  
{
	uint16_t free;
	
	if(fifo->head >= fifo->tail) free = fifo->tail + (fifo->length - fifo->head);
	else free = fifo->tail - fifo->head;
	
  return free;
}

u16 fifo_used(fifo_t* fifo)
{
	uint16_t used;
	
	if(fifo->head >= fifo->tail) used = fifo->head - fifo->tail;
	else used = fifo->head + (fifo->length - fifo->tail);
	
	return used;	
}


/** @brief ��ʼ��������
  * @param *fifo
  *        *buf 
  *        length
  */
void fifo_init(fifo_t* fifo, uint8_t* buf, uint16_t length)  
{
	uint16_t i;
	
	fifo->buf = buf;
	fifo->length = length;
	fifo->head = 0;
	fifo->tail = 0;
	
	for(i=0; i<length; i++) fifo->buf[i] = 0;	
}





