/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
#include "sdio_sdcard.h"
//#include "w25qxx.h"
//#include "malloc.h"		

#ifdef __cplusplus
extern "C" {
#endif
#define SD_CARD	 0  //SD?��,?������?a0
#define EX_FLASH 1	//��a2?flash,?������?a1

#define FLASH_SECTOR_SIZE 	512			  
//??����W25Q128
//?��12M��??��??fatfs��?,12M��??��o��,��?������?��?��??a,��??a??��?3.09M.	���ꨮ��2?��?,???��?���??o��?	 			    
u16	    FLASH_SECTOR_COUNT=2048*12;	//W25Q1218,?��12M��??��??FATFS??��?
#define FLASH_BLOCK_SIZE   	8     	//????BLOCK��D8??����??

//3?��??����??��
DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber (0..) */
)
{
	u8 res=0;	    
#ifdef W25QXX_OPEN
	switch(pdrv)
	{
		case SD_CARD://SD?��
			res=SD_Init();//SD?��3?��??�� 
  			break;
		case EX_FLASH://��a2?flash
			W25QXX_Init();
			FLASH_SECTOR_COUNT=2048*12;//W25Q1218,?��12M��??��??FATFS??��? 
 			break;
		default:
			res=1; 
	}	
#else
res=SD_Init();//SD?��3?��??�� 
#endif  //end of W25QXX_OPEN
	if(res)return  STA_NOINIT;
	else return 0; //3?��??��3��1|
}  

//??��?��??�����䨬?
DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber (0..) */
)
{ 
	return 0;
} 

//?������??
//drv:��??������o?0~9
//*buff:��y?Y?����??o3?������??��
//sector:����??��??��
//count:D����a?����?��?����??��y
DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	UINT count		/* Number of sectors to read (1..128) */
)
{
	u8 res=0; 
    if (!count)return RES_PARERR;//count2??���̨�����0��?��??������??2?��y�䨪?��		 	 
#ifdef W25QXX_OPEN
	switch(pdrv)
	{
		case SD_CARD://SD?��
			res=SD_ReadDisk(buff,sector,count);	 
			while(res)//?��3?�䨪
			{
				SD_Init();	//??D?3?��??��SD?��
				res=SD_ReadDisk(buff,sector,count);	
				//printf("sd rd error:%d\r\n",res);
			}
			break;
		case EX_FLASH://��a2?flash
			for(;count>0;count--)
			{
				W25QXX_Read(buff,sector*FLASH_SECTOR_SIZE,FLASH_SECTOR_SIZE);
				sector++;
				buff+=FLASH_SECTOR_SIZE;
			}
			res=0;
			break;
		default:
			res=1; 
	}
#else
res=SD_ReadDisk(buff,sector,count);  
while(res)//?��3?�䨪
{
	SD_Init();	//??D?3?��??��SD?��
	res=SD_ReadDisk(buff,sector,count); 
	//printf("sd rd error:%d\r\n",res);
}

#endif  //end of EXFUN_OPEN
   //��|��������???�̡�???SPI_SD_driver.c��?����???�̡�a3��ff.c��?����???��
    if(res==0x00)return RES_OK;	 
    else return RES_ERROR;	   
}

//D�䨦��??
//drv:��??������o?0~9
//*buff:����?����y?Y������??��
//sector:����??��??��
//count:D����aD�䨨?��?����??��y
#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	UINT count			/* Number of sectors to write (1..128) */
)
{
	u8 res=0;  
    if (!count)return RES_PARERR;//count2??���̨�����0��?��??������??2?��y�䨪?��		 	 
#ifdef W25QXX_OPEN
	switch(pdrv)
	{
		case SD_CARD://SD?��
			res=SD_WriteDisk((u8*)buff,sector,count);
			while(res)//D��3?�䨪
			{
				SD_Init();	//??D?3?��??��SD?��
				res=SD_WriteDisk((u8*)buff,sector,count);	
				//printf("sd wr error:%d\r\n",res);
			}
			break;
		case EX_FLASH://��a2?flash
			for(;count>0;count--)
			{										    
				W25QXX_Write((u8*)buff,sector*FLASH_SECTOR_SIZE,FLASH_SECTOR_SIZE);
				sector++;
				buff+=FLASH_SECTOR_SIZE;
			}
			res=0;
			break;
		default:
			res=1; 
	}
#else
res=SD_WriteDisk((u8*)buff,sector,count);
while(res)//D��3?�䨪
{
	SD_Init();	//??D?3?��??��SD?��
	res=SD_WriteDisk((u8*)buff,sector,count);	
	//printf("sd wr error:%d\r\n",res);
}

#endif  //end of W25QXX_OPEN
    //��|��������???�̡�???SPI_SD_driver.c��?����???�̡�a3��ff.c��?����???��
    if(res == 0x00)return RES_OK;	 
    else return RES_ERROR;	
}
#endif


//????����2?��y��???��?
 //drv:��??������o?0~9
 //ctrl:????�䨲??
 //*buff:����?��/?����??o3???????
#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;						  			     
#ifdef W25QXX_OPEN
	if(pdrv==SD_CARD)//SD?��
#endif  //end of W25QXX_OPEN
	{
	    switch(cmd)
	    {
		    case CTRL_SYNC:
				res = RES_OK; 
		        break;	 
		    case GET_SECTOR_SIZE:
				*(DWORD*)buff = 512; 
		        res = RES_OK;
		        break;	 
		    case GET_BLOCK_SIZE:
				*(WORD*)buff = SDCardInfo.CardBlockSize;
		        res = RES_OK;
		        break;	 
		    case GET_SECTOR_COUNT:
		        *(DWORD*)buff = SDCardInfo.CardCapacity/512;
		        res = RES_OK;
		        break;
		    default:
		        res = RES_PARERR;
		        break;
	    }
	}
#ifdef W25QXX_OPEN
    else if(pdrv==EX_FLASH)	//��a2?FLASH  
	{
	    switch(cmd)
	    {
		    case CTRL_SYNC:
				res = RES_OK; 
		        break;	 
		    case GET_SECTOR_SIZE:
		        *(WORD*)buff = FLASH_SECTOR_SIZE;
		        res = RES_OK;
		        break;	 
		    case GET_BLOCK_SIZE:
		        *(WORD*)buff = FLASH_BLOCK_SIZE;
		        res = RES_OK;
		        break;	 
		    case GET_SECTOR_COUNT:
		        *(DWORD*)buff = FLASH_SECTOR_COUNT;
		        res = RES_OK;
		        break;
		    default:
		        res = RES_PARERR;
		        break;
	    }
	}else res=RES_ERROR;//????��?2??��3?
#endif  //end of W25QXX_OPEN
    return res;
}
#endif
//??��?����??
//User defined function to give a current time to fatfs module      */
//31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */                                                                                                                                                                                                                                          
//15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */                                                                                                                                                                                                                                                
DWORD get_fattime (void)
{				 
	return 0;
}			 

#define mymalloc_OPEN 0
//?����?��????����?
void *ff_memalloc (UINT size)			
{
#if  mymalloc_OPEN
	return (void*)mymalloc(SRAMIN,size);
#else
//	return ;//(void*)new(size);

#endif  //end of W25QXX_OPEN
}


#if  !mymalloc_OPEN
struct mem_control_block { 
 int is_available;	  //����һ����ǣ� 
 int size;			  //����ʵ�ʿռ�Ĵ�С 
 };
#endif  //end of mymalloc_OPEN

//������??����?
void ff_memfree (void* mf)		 
{
#if  mymalloc_OPEN
	myfree(SRAMIN,mf);
#else
//struct mem_control_block *free; 
//free->size = mf - sizeof(struct mem_control_block); 
//free->is_available = 1; 
return; 

#endif  //end of mymalloc_OPEN
}

#ifdef __cplusplus
}
#endif // __cplusplus















