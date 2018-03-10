/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * ×÷Õß   £ºÄäÃû¿Æ´´
 * ÎÄ¼þÃû  £ºpwm_out.c
 * ÃèÊö    £ºPWMÊä³ö
 * ¹ÙÍø    £ºwww.anotc.com
 * ÌÔ±¦    £ºanotc.taobao.com
 * ¼¼ÊõQÈº £º190169595
**********************************************************************************/

#include "pwm_out.h"
#include "include.h"
#include "mymath.h"

//21·ÖÆµµ½ 84000000/21 = 4M   0.25us
#define PWMPeriod  84// (SystemCoreClock / 20000 ) - 1//8400-1

#define INIT_DUTY 4000 //u16(1000/0.25)
#define ACCURACY 10000 //u16(2500/0.25) //accuracy
#define PWM_RADIO 4//(8000 - 4000)/1000.0
void TIM1_GPIO_Config(void)
{
  //PE 8 9 10 11 12 13 14è¾“å‡º
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_10 ;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed =GPIO_Speed_100MHz;
  GPIO_Init(GPIOA,&GPIO_InitStructure);

  
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1); //CH1
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1);//CH2
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_TIM1);
//  GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_TIM1);
//  GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1);
//  GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1);
}

//TIM1åšPWMè¾“å‡º
void Tim1_Config(u16 hz)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	u32 TimerPeriod,ccr1,ccr2,ccr3,ccr4;
	  uint16_t PrescalerValue = 0;
	  u32 hz_set = ACCURACY*hz;
	
	  hz_set = LIMIT (hz_set,1,SystemCoreClock);
  TimerPeriod =  (SystemCoreClock / 20000 ) - 1;
//  ccr1 = TimerPeriod / 2;  //å ç©ºæ¯?/2 = 50%
  ccr2 = INIT_DUTY;  //å ç©ºæ¯?/3 = 33%
  ccr3 = INIT_DUTY;  //å ç©ºæ¯?/4 = 25%
  ccr1 = INIT_DUTY;  //å ç©ºæ¯?/3 = 33%
  ccr4 = INIT_DUTY;  //å ç©ºæ¯?/4 = 25%
//  ccr4 = TimerPeriod / 5;  //å ç©ºæ¯?/5 = 20%
    PrescalerValue = (uint16_t) ( ( SystemCoreClock  ) / hz_set ) - 1;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
  //æ—¶åŸºåˆå§‹åŒ?
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //æ­»åŒºæŽ§åˆ¶ç”¨ã€?
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  //è®¡æ•°å™¨æ–¹å?
//  TIM_TimeBaseInitStructure.TIM_Prescaler = 0;   //Timer clock = sysclock /(TIM_Prescaler+1) = 168M
  TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
//  TIM_TimeBaseInitStructure.TIM_Period = TimerPeriod - 1;    //Period = (TIM counter clock / TIM output clock) - 1 = 20K  50US

  TIM_TimeBaseInitStructure.TIM_Period = ACCURACY;									
  TIM_TimeBaseInitStructure.TIM_Prescaler = PrescalerValue;	

  TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);

  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = ccr1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;// TIM_OCPolarity_High;//è¾“å‡ºåŒç›¸ï¼ŒTIM_OCNPolarity_Highæ—¶è¾“å‡ºåç›?
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
  
   TIM_OC1Init(TIM1,&TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = ccr2;
  TIM_OC2Init(TIM1,&TIM_OCInitStructure);
  
  TIM_OCInitStructure.TIM_Pulse = ccr3;
  TIM_OC3Init(TIM1,&TIM_OCInitStructure);
  
  TIM_OCInitStructure.TIM_Pulse = ccr4;
   TIM_OC4Init(TIM1,&TIM_OCInitStructure);
  
  TIM_Cmd(TIM1,ENABLE);
  TIM_CtrlPWMOutputs(TIM1,ENABLE);
}

u8 PWM_Out_Init(uint16_t hz)//400hz
{
	TIM1_GPIO_Config();
	 Tim1_Config(hz);
}

#ifdef open_print
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	uint16_t PrescalerValue = 0;
	u32 hz_set = ACCURACY*hz;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);
	
	hz_set = LIMIT (hz_set,1,84000000);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
#if 0
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
#endif
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);

/////////////////////////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIO_Pin_0 | GPIO_Pin_1 | 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5); 
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM2);
	
	/* Compute the prescaler value */
  PrescalerValue = (uint16_t) ( ( SystemCoreClock /2 ) / hz_set ) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);


  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

//  /* PWM1 Mode configuration: Channel1 */
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC1Init(TIM5, &TIM_OCInitStructure);
//  TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

//  /* PWM1 Mode configuration: Channel2 */
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
//  TIM_OC2Init(TIM5, &TIM_OCInitStructure);
//  TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC3Init(TIM5, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC4Init(TIM5, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM5, ENABLE);
  TIM_Cmd(TIM5, ENABLE);
/////////////////////////////////////////////////////////////////////////////
#if 0   //TIM1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOE, &GPIO_InitStructure); 

  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1); 
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
	
	/* Compute the prescaler value */
  PrescalerValue = (uint16_t) ( ( SystemCoreClock ) / hz_set ) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);


  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  //TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
  //TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
  //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);
  //TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
  TIM_ARRPreloadConfig(TIM1, ENABLE);
  TIM_Cmd(TIM1, ENABLE);	
#endif  //end of MAV_LOG_TSET
	////////////////////////////////////////////////////////////////////////////////////

#if 0 //TIM8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);

	/* Compute the prescaler value */
  PrescalerValue = (uint16_t) ( ( SystemCoreClock ) / hz_set ) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = ACCURACY;									
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;		
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);


  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

  /* PWM1 Mode configuration: Channel3 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC3Init(TIM8, &TIM_OCInitStructure);
  //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = INIT_DUTY;
  TIM_OC4Init(TIM8, &TIM_OCInitStructure);
  //TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
 
 
  TIM_CtrlPWMOutputs(TIM8, ENABLE);
  TIM_ARRPreloadConfig(TIM8, ENABLE);
  TIM_Cmd(TIM8, ENABLE);
#endif  //end of MAV_LOG_TSET


	if( hz_set > 84000000 )
	{
		return 0;
	}
	else
	{
		return 1;
	}
}
#endif  //end of open_print

u8 CH_out_Mapping[MAXMOTORS] = {0,1,2,3};

void CH_out_Mapping_Fun(u16 *out,u16 *mapped )
{
	u8 i;
	for( i = 0 ; i < MAXMOTORS ; i++ )
	{
		*( mapped + i ) = *( out + CH_out_Mapping[i] );
	}
}

void SetPwm(u8 PWMpercent)//(int16_t pwm[MAXMOTORS],s16 min,s16 max)
{
//	u8 i;
//	s16 pwm_tem[MAXMOTORS];

//	for(i=0;i<MAXMOTORS;i++)
//	{
//			pwm_tem[i] = pwm[i] ;
//			pwm_tem[i] = LIMIT(pwm_tem[i],min,max);
//	}
	
//	 	TIM1->CCR2 =  PWMpercent*83;			//5	
// 		TIM1->CCR3 =  PWMpercent*83;			//6	
// 		if(PWMpercent>99)
 		{
			TIM1->CCR4 =7399;
		    TIM1->CCR3 =5399;
//			TIM1->CCR1 =7399;
//		    TIM1->CCR4 =7399;

		}
//	TIM1->CCR4 = PWM_RADIO *( pwm_tem[CH_out_Mapping[0]] ) + INIT_DUTY;				//1	
//	TIM1->CCR3 = PWM_RADIO *( pwm_tem[CH_out_Mapping[1]] ) + INIT_DUTY;				//2
//	TIM1->CCR2 = PWM_RADIO *( pwm_tem[CH_out_Mapping[2]] ) + INIT_DUTY;				//3	
//	TIM1->CCR1 = PWM_RADIO *( pwm_tem[CH_out_Mapping[3]] ) + INIT_DUTY;				//4
	
// 	TIM5->CCR4 = PWM_RADIO *( pwm_tem[CH_out_Mapping[4]] ) + INIT_DUTY;				//5	
// 	TIM5->CCR3 = PWM_RADIO *( pwm_tem[CH_out_Mapping[5]] ) + INIT_DUTY;				//6	
// 	TIM8->CCR4 = PWM_RADIO *( pwm_tem[CH_out_Mapping[6]] ) + INIT_DUTY;				//7	
// 	TIM8->CCR3 = PWM_RADIO *( pwm_tem[CH_out_Mapping[7]] ) + INIT_DUTY;				//8	
	
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
