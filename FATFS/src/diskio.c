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
#define SD_CARD	 0  //SD?¨,?í±ê?a0
#define EX_FLASH 1	//ía2?flash,?í±ê?a1

#define FLASH_SECTOR_SIZE 	512			  
//??óúW25Q128
//?°12M×??ú??fatfsó?,12M×??úoó,ó?óú′?·?×??a,×??a??ó?3.09M.	ê￡óà2?·?,???í?§×??oó?	 			    
u16	    FLASH_SECTOR_COUNT=2048*12;	//W25Q1218,?°12M×??ú??FATFS??ó?
#define FLASH_BLOCK_SIZE   	8     	//????BLOCKóD8??éè??

//3?ê??ˉ′??ì
DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber (0..) */
)
{
	u8 res=0;	    
#ifdef W25QXX_OPEN
	switch(pdrv)
	{
		case SD_CARD://SD?¨
			res=SD_Init();//SD?¨3?ê??ˉ 
  			break;
		case EX_FLASH://ía2?flash
			W25QXX_Init();
			FLASH_SECTOR_COUNT=2048*12;//W25Q1218,?°12M×??ú??FATFS??ó? 
 			break;
		default:
			res=1; 
	}	
#else
res=SD_Init();//SD?¨3?ê??ˉ 
#endif  //end of W25QXX_OPEN
	if(res)return  STA_NOINIT;
	else return 0; //3?ê??ˉ3é1|
}  

//??μ?′??ì×′ì?
DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber (0..) */
)
{ 
	return 0;
} 

//?áéè??
//drv:′??ì±ào?0~9
//*buff:êy?Y?óê??o3?ê×μ??·
//sector:éè??μ??·
//count:Dèòa?áè?μ?éè??êy
DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	UINT count		/* Number of sectors to read (1..128) */
)
{
	u8 res=0; 
    if (!count)return RES_PARERR;//count2??üμèóú0￡?·??ò·μ??2?êy′í?ó		 	 
#ifdef W25QXX_OPEN
	switch(pdrv)
	{
		case SD_CARD://SD?¨
			res=SD_ReadDisk(buff,sector,count);	 
			while(res)//?á3?′í
			{
				SD_Init();	//??D?3?ê??ˉSD?¨
				res=SD_ReadDisk(buff,sector,count);	
				//printf("sd rd error:%d\r\n",res);
			}
			break;
		case EX_FLASH://ía2?flash
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
while(res)//?á3?′í
{
	SD_Init();	//??D?3?ê??ˉSD?¨
	res=SD_ReadDisk(buff,sector,count); 
	//printf("sd rd error:%d\r\n",res);
}

#endif  //end of EXFUN_OPEN
   //′|àí·μ???μ￡???SPI_SD_driver.cμ?·μ???μ×a3éff.cμ?·μ???μ
    if(res==0x00)return RES_OK;	 
    else return RES_ERROR;	   
}

//D′éè??
//drv:′??ì±ào?0~9
//*buff:·￠?íêy?Yê×μ??·
//sector:éè??μ??·
//count:DèòaD′è?μ?éè??êy
#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	UINT count			/* Number of sectors to write (1..128) */
)
{
	u8 res=0;  
    if (!count)return RES_PARERR;//count2??üμèóú0￡?·??ò·μ??2?êy′í?ó		 	 
#ifdef W25QXX_OPEN
	switch(pdrv)
	{
		case SD_CARD://SD?¨
			res=SD_WriteDisk((u8*)buff,sector,count);
			while(res)//D′3?′í
			{
				SD_Init();	//??D?3?ê??ˉSD?¨
				res=SD_WriteDisk((u8*)buff,sector,count);	
				//printf("sd wr error:%d\r\n",res);
			}
			break;
		case EX_FLASH://ía2?flash
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
while(res)//D′3?′í
{
	SD_Init();	//??D?3?ê??ˉSD?¨
	res=SD_WriteDisk((u8*)buff,sector,count);	
	//printf("sd wr error:%d\r\n",res);
}

#endif  //end of W25QXX_OPEN
    //′|àí·μ???μ￡???SPI_SD_driver.cμ?·μ???μ×a3éff.cμ?·μ???μ
    if(res == 0x00)return RES_OK;	 
    else return RES_ERROR;	
}
#endif


//????±í2?êyμ???μ?
 //drv:′??ì±ào?0~9
 //ctrl:????′ú??
 //*buff:·￠?í/?óê??o3???????
#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;						  			     
#ifdef W25QXX_OPEN
	if(pdrv==SD_CARD)//SD?¨
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
    else if(pdrv==EX_FLASH)	//ía2?FLASH  
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
	}else res=RES_ERROR;//????μ?2??§3?
#endif  //end of W25QXX_OPEN
    return res;
}
#endif
//??μ?ê±??
//User defined function to give a current time to fatfs module      */
//31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */                                                                                                                                                                                                                                          
//15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */                                                                                                                                                                                                                                                
DWORD get_fattime (void)
{				 
	return 0;
}			 

#define mymalloc_OPEN 0
//?ˉì?·????ú′?
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
 int is_available;	  //这是一个标记？ 
 int size;			  //这是实际空间的大小 
 };
#endif  //end of mymalloc_OPEN

//êí·??ú′?
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















