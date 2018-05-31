/**
  ******************************************************************************
  * @file    stm32f1xx_it.c 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    11-April-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "main.h"
#include "string.h"
#include "anglequeue.h"
#include "speedqueue.h"
#include "stdio.h"
#include "includes.h"
/** @addtogroup IO_Toggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M Processor Exceptions Handlers                          */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
	
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
	printf("HardFault \r\n");
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
		
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
		
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
		
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
		
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
	
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
	
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//	
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
__IO uint32_t delay_time = 0;

void SysTick_Handler(void)
{
	OSIntEnter();	 	    //OSIntExit();
	OSTimeTick();       //调用ucos的时钟服务程序 
	delay_time++;
	if(1000 ==delay_time)
	{
		delay_time = 0;
		OSSemPost(Sem_Event);
	}
	//TimingDelay_Decrement();	
	OSIntExit();        //触发任务切换软中断
}

/**功能：跟随和方向盘功能切换按键
  * 
  * 
  */
void EXTI2_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line2) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line2);
  }
}


/******************************************************************************/
/*                 STM32F1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_md.s).                                            */
/******************************************************************************/
/**
  * @brief  This function handles EXTI15_10_IRQHandler Handler.
  * @param  None
  * @retval None
  */

//extern uint32_t Machine_Front_PWM_Count;

static u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
static u16 USART_RX_STA=0;       //接收状态标记	  
static u8 len;
__IO uint8_t Break_State=MACHINE_NOBREAK_STATE;

static ANGLEQUEUE_TYPE angle_temp;
static SPEEDQUEUE_TYPE speed_temp;

enum
{
	RECIVE_SPEED,
	RECIVE_ANGLE,
	RECIVE_STOP,
	RECIVE_NONE,
	RECIVE_DRIVE_MODE,
	RECIVE_DRIVE_DIRECTION,
};

static __IO u8 RecvBuf[40] ; 
static __IO u8 Flag = RECIVE_NONE; 
static u8 i = 0;
static __IO ANGLEQUEUE_TYPE AngleType ; 
static __IO SPEEDQUEUE_TYPE SpeedType;

//extern u8 TurnEnableFlag ; 
extern __IO u8 TrunEnableFlag; 
//uint8_t Break_Flag=0;


__IO uint8_t Control_By_Steeringwheel_Or_UWB_Flag=0;
void USART1_IRQHandler(void)      //串口1中断服务程序
{
	static u8 Res;
  OSIntEnter();		//进入中断
	if(TrunEnableFlag == 0)
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE );
		return; 
	}	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART1);	//读取接收到的数据
		//printf("%c\r\n",Res);	   		
		//A:100,1000      angle
		//B:0    B:1      break
		//S:200           speed
		//D:0  1          //0  代表手动驾驶     1 代表自动驾驶		
		if((Res == 'A') || (Flag == RECIVE_ANGLE))
		{
			Flag = RECIVE_ANGLE ; 
			//get value
			
			RecvBuf[i] = Res ; 
			i++ ; 
			
			if(Res == '\r')
			{
					Flag = RECIVE_NONE ;
					i = 0 ; 
					//operation of the value
					AngleType.angle = atoi((const char *)(RecvBuf+2));
					char *str = strchr((const char *)RecvBuf , ',');
					AngleType.time = atoi((const char *)(str+1));					
					anglequeue_enqueue(AngleType);
			}			
		}else if((Res == 'B') || (Flag == RECIVE_STOP))
		{
			Flag = RECIVE_STOP ; 			
			RecvBuf[i] = Res ; 
			i++ ; 	
			if(Res == '\r')
			{
					Flag = RECIVE_NONE ;
					i = 0 ; 				  
					uint8_t temp;
				  temp = atoi((const char *)(RecvBuf+2));
					if(temp)
					{
						 Break_State = MACHINE_BREAK_STATE;
					}
					else{					
						Break_State = MACHINE_NOBREAK_STATE;
					}
			}
		}else if((Res == 'S') || (Flag == RECIVE_SPEED))
		{
			Flag = RECIVE_SPEED ; 			
			RecvBuf[i] = Res ; 
			i++ ; 
			if(Res == '\r')
			{
					Flag = RECIVE_NONE ; 
					i = 0 ; 				
				  SpeedType.speed = atoi((const char*)(RecvBuf+2));
				  //printf("SPEED:%d\r\n" , SpeedType.speed);
				  speedqueue_enqueue(SpeedType);
			}
		}else if((Res == 'D') || (Flag == RECIVE_DRIVE_MODE))
		{
			Flag = RECIVE_DRIVE_MODE ; 			
			RecvBuf[i] = Res ; 
			i++; 			
			if(Res == '\r')
			{
					Flag = RECIVE_NONE ; 
					i = 0 ; 				
				  if(atoi((const char*)(RecvBuf+2)))
					{
						Control_By_Steeringwheel_Or_UWB_Flag = 1;
						//printf("auto follow  1111111111!!\r\n");
					}else
					{
						Control_By_Steeringwheel_Or_UWB_Flag = 0;
						//printf("MANUAL 0000000000\r\n");
					}
					//NVIC_SystemReset();
					//Machine_Front_Motor_Reset();
			}
		}else
		{
			Flag = RECIVE_NONE; 
			i = 0 ; 
		}	
	  USART_ClearITPendingBit(USART1,USART_IT_RXNE );
  }
    OSIntExit();        //触发任务切换软中断	
} 

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
