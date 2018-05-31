#include "break.h"
#include "main.h"
#include <stdio.h>
#include "throttle.h"
#include "includes.h"

void Manual_Break_Init(void)
{ 
	EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;
	
  /* Enable GPIOBclock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  
  /* Configure PB2 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI2 Line to PB2 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource7);

  /* Configure EXTI2line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line7;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 10;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void Manual_Change_Machine_Back_or_Forward_Init(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;
	
  /* Enable GPIOBclock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  
  /* Configure PB2 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI2 Line to PB2 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource5);

  /* Configure EXTI2line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line5;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 9;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}




__IO uint32_t Speed_Right_Count = 0;//右边轮子的脉冲值

void EXTI9_5_IRQHandler(void)
{
	OSIntEnter();		//OSIntExit();
  if(EXTI_GetITStatus(EXTI_Line7) != RESET)
  {
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)) //1的时候刹车处于放开的状态
		{
		  Manual_Break_State = 1;
			//printf("manual break 1\r\n");
		}
		else if(!GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)) //0的时候刹车处于踩下的状态
		{
			Manual_Break_State = 0;
			//printf("manual break 0\r\n");
		}
    EXTI_ClearITPendingBit(EXTI_Line7);
  }
	if(EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		//if(!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6))
		{
		   Speed_Right_Count++;
		}
		if(Speed_Right_Count == MAX_SPEED_COUNT)
		{
			//OSSemPost(Sem_Event); //发送信号量
		}		   
	  //printf("Speed_Right_Count:%d\r\n",Speed_Right_Count);
		EXTI_ClearITPendingBit(EXTI_Line6);
	}
	OSIntExit();
}
