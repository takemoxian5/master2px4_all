#include "mytimer.h"
//#include "led.h"
#include "include.h"
#include "MissionAPI.h"

#define SYS_TIMx					TIM2
#define SYS_RCC_TIMx			RCC_APB1Periph_TIM2

void TIM_CONF()   //APB1  84M
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		
		/* 使能时钟 */
    RCC_APB1PeriphClockCmd(SYS_RCC_TIMx,ENABLE);

		TIM_DeInit(SYS_TIMx);
	
	/* 自动重装载寄存器周期的值(计数值) */
    TIM_TimeBaseStructure.TIM_Period=1000;
	
    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
	  /* 时钟预分频数为72 */
    TIM_TimeBaseStructure.TIM_Prescaler= 84 - 1;
	
		/* 对外部时钟进行采样的时钟分频,这里没有用到 */
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;   //向上计数
	
		TIM_TimeBaseInit(SYS_TIMx,&TIM_TimeBaseStructure);

		TIM_ClearFlag(SYS_TIMx,TIM_FLAG_Update);

		TIM_ITConfig(SYS_TIMx,TIM_IT_Update,ENABLE);
		
		
    TIM_Cmd(SYS_TIMx, ENABLE);																		
    
    RCC_APB1PeriphClockCmd(SYS_RCC_TIMx , DISABLE);		/*先关闭等待使用*/  
}
void TIM_NVIC()
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_TIME_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_TIME_S;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    
    
    
    
//    	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟,84M
//  	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
//	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
//	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; //时钟不分频
//	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
}

void TIM_INIT()
{
    TIM_CONF();
    TIM_NVIC();
	
		/* TIM2 重新开时钟，开始计时 */
	  RCC_APB1PeriphClockCmd(SYS_RCC_TIMx , ENABLE);
}


volatile uint32_t sysTickUptime = 0;

#define TICK_PER_SECOND 1000 
#define TICK_US	(1000000/TICK_PER_SECOND)

void  SysTick_Configuration(void)
{
	RCC_ClocksTypeDef  rcc_clocks;
	uint32_t         cnts;

	RCC_GetClocksFreq(&rcc_clocks);

	cnts = (uint32_t)rcc_clocks.HCLK_Frequency / TICK_PER_SECOND;
	cnts = cnts / 8;

	SysTick_Config(cnts);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

uint32_t GetSysTime_us(void) 
{
	register uint32_t ms;
	u32 value;
	ms = sysTickUptime;
	value = ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD;
	return value;
}

void Delay_us(uint32_t us)
{
    uint32_t now = GetSysTime_us();
    while (GetSysTime_us() - now < us);
}

void Delay_ms(uint32_t ms)
{
    while (ms--)
        Delay_us(1000);
}

int time_1h,time_1m,time_1s,time_1ms;

void sys_time()
{ 
	if( !Init_Finish )
	{
		return;
	}
	
  if(time_1ms < 999)
	{
    time_1ms++;

Loop_check();
	}
	else
	{
		
    time_1ms =0;
	  if(time_1s<59)
	  {
      time_1s++;
		}
		else
		{
			time_1s = 0;
			if(time_1m<59)
			{
				time_1m++;
			}
			else
			{
				time_1m = 0;
				if(time_1h<23)
				{
					time_1h++;
				}
				else
				{
					time_1h = 0;
				}
			}
		}
	}
}

volatile float Cycle_T[GET_TIME_NUM][3];

enum
{
	NOW = 0,
	OLD,
	NEW,
};

float Get_Cycle_T(u8 item)	
{
	Cycle_T[item][OLD] = Cycle_T[item][NOW];	//上一次的时间
	Cycle_T[item][NOW] = GetSysTime_us()/1000000.0f; //本次的时间
	Cycle_T[item][NEW] = ( ( Cycle_T[item][NOW] - Cycle_T[item][OLD] ) );//间隔的时间（周期）
	return Cycle_T[item][NEW];
}

void Cycle_Time_Init()
{
	u8 i;
	for(i=0;i<GET_TIME_NUM;i++)
	{
		Get_Cycle_T(i);
	}

}

//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟,84M
  	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; //时钟不分频
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
}

