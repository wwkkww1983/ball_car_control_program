/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x_it.h"
#include "STM32F10X.h"
#include "motorSalgorithm.h"
#include "includes.h"
#include "main.h"

/*see 
http://picprog.strongedge.net/step_prof/step-profile.html
*/


 /*���S�������㷨����������*/
void TIMX_UP_IRQHandler_S(MOTOR_CONTROL_S* pmotor)
{   
	if(1==pmotor->en)
	{ 
		//λ�ü���
		if(pmotor->clockwise==pmotor->dir)
		{
			pmotor->CurrentPosition_Pulse++;
			if(pmotor->CurrentPosition_Pulse>=pmotor->MaxPosition_Pulse)
			{
				pmotor->CurrentPosition_Pulse=0;
			}
		}
		else
		{
			pmotor->CurrentPosition_Pulse--;
			if(pmotor->CurrentPosition_Pulse==0xffffffff)
			{
				pmotor->CurrentPosition_Pulse=pmotor->MaxPosition_Pulse-1;
			}
		}
		pmotor->CurrentPosition=pmotor->CurrentPosition_Pulse/pmotor->divnum;
		
		//�ٶȿ���
		if(pmotor->speedenbale&&(pmotor->CurrentIndex==pmotor->TargetIndex||pmotor->TargetIndex+pmotor->CurrentIndex==pmotor->StartTableLength+pmotor->StopTableLength-1))
		{
			return;
		}
		pmotor->PulsesHaven++; //���������
		pmotor->pulsecount++;  //�Ը�Ƶ������������������ 
		
		//�ԳƷ�ת
		if(pmotor->RevetDot==pmotor->PulsesHaven)
		{
			pmotor->pulsecount=pmotor->Step_Table[pmotor->CurrentIndex];
		}
		if(pmotor->pulsecount>=pmotor->Step_Table[pmotor->CurrentIndex])
		{ 
			if(pmotor->PulsesHaven<=pmotor->StartSteps)
			{
				//�𲽽׶�
				if(pmotor->CurrentIndex<pmotor->StartTableLength-1)
				{
					pmotor->CurrentIndex++;
					pmotor->pulsecount=0;
					if(pmotor->CurrentIndex>=pmotor->StartTableLength)pmotor->CurrentIndex=pmotor->StartTableLength;
				}
			}
			//�����ٶȿ��ƣ��˴������ж�pmotor->PulsesHaven>=(pmotor->PulsesGiven>>1)
			//if(pmotor->PulsesGiven-pmotor->PulsesHaven<=pmotor->StopSteps&&pmotor->PulsesHaven>=(pmotor->PulsesGiven>>1))
			if((pmotor->PulsesGiven-pmotor->PulsesHaven<=pmotor->StopSteps&&pmotor->speedenbale==1)||
				(pmotor->PulsesGiven-pmotor->PulsesHaven<=pmotor->StopSteps&&pmotor->speedenbale==0&&pmotor->PulsesHaven>=(pmotor->PulsesGiven>>1))) 
			{
				//ֹͣ�׶�
				if(pmotor->CurrentIndex<pmotor->StartTableLength-1)
				{
					pmotor->CurrentIndex=pmotor->StartTableLength+pmotor->StopTableLength-pmotor->CurrentIndex;  
				}
				pmotor->CurrentIndex++;
				pmotor->pulsecount=0;
				if(pmotor->CurrentIndex>=pmotor->StartTableLength+pmotor->StopTableLength)
					pmotor->CurrentIndex=pmotor->StartTableLength+pmotor->StopTableLength-1;
			}  		
			pmotor->TIMx->ARR = pmotor->Counter_Table[pmotor->CurrentIndex] ; //��������
			pmotor->TIMx->CCR1 =( pmotor->Counter_Table[pmotor->CurrentIndex])>>1;       //����ռ�ձ�	    
		}	  
		//��תԤ����������ֹͣ��running=0�����Խ�����һ����ת
		if(pmotor->PulsesHaven>=pmotor->PulsesGiven&&pmotor->PulsesHaven>3)
		{
			pmotor->en=0;
			pmotor->running=0;
			pmotor->CurrentIndex=0;
		
		  TIM_Cmd(pmotor->TIMx, DISABLE);		  //DISABLE 	
			//USART1_Printfstr("1\r\n");
      
		}
		else
		{			
			pmotor->Time_Cost_Act+=pmotor->TIMx->ARR;
		}
	}
}


//extern uint32_t Machine_Front_PWM_Count;
extern __IO uint8_t Machine_Front_Motor_State;

void TIM4_IRQHandler(void)
{   
	  OSIntEnter();		//OSIntExit();
		if( 3 == Machine_Front_Motor_State)	
		{
				if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
					{
						TIMX_UP_IRQHandler_S(&motor4);
						TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
					}
		}
	 else{
			if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
			 {
						if(Machine_Front_PWM_Count)
						{
								Machine_Front_PWM_Count--;
						}
						if(0==Machine_Front_PWM_Count)
						{
							TIM_Cmd(TIM4, DISABLE);
						}
						//capture = TIM_GetCapture1(TIM4); 
						//TIM_SetCompare1(TIM3, capture + period1 ); 
						TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
			 }
	  }
	  OSIntExit();
}







