#include <stdio.h>
#include "speed.h"
#include "main.h"
#include "throttle.h"
#include "kalman.h"
#include "stm32f10x_it.h"

#define speed_active_timestamp 4
#define direction_control_active_timestamp 2
#define SPEED_SET_TIMESTAMP 5
#define SPEED_STEP 2

extern uint8_t speed_active_time;
extern __IO uint32_t time_count;

static uint8_t DutycycleL = 0;
static uint8_t DutycycleR = 0;
static uint8_t DutycycleL_Temp = 0;
static uint8_t DutycycleR_Temp = 0; 

static double angle_factor=1;
static __IO  uint8_t Control_Direction=2;//ת��״̬
static uint8_t set_angle_speed_state=0;//�ٶȺͽǶȶ������˵ı�־λ	
extern __IO uint8_t  Break_State;
static uint8_t Break_State_Inter=0;
static uint8_t speed_state=0;
static __IO uint16_t Angle; 
uint8_t DriveMotorSpeed=0;       //��������ٶ�
static ANGLEQUEUE_TYPE angletemp;
static SPEEDQUEUE_TYPE speedtemp;
uint16_t newspeedtemp = 0;
uint16_t oldspeedtemp = 0;
uint16_t delspeedtemp = 0;
static uint8_t Machine_Direction_Switch_Flag = 0; //1��ʱ������󵹳�����ʱ�ٶ�Ӧ�ø���һ�㡣0 ��ʱ������ǰ������ٶ�Ӧ����������������������Ƶ����ٶȵġ�
uint8_t printf_count = 0;
//extern __IO uint32_t Machine_Front_PWM_Count;

static u8 SlowStopFlag = 0 ; 
//Follow New Add CJY ****

void time_tick_active(void)
{
		   //�Ƕȶ��д���	 	 
			 //if((Machine_Front_PWM_Count == 0) && (!anglequeue_dequeue(&angletemp)))
			 if (!anglequeue_dequeue(&angletemp))
			 {        
					//printf("angle:%d time:%d \r\n",angletemp.angle,angletemp.time);
					//ǰ�ֵ���Ŀ���					
					Machine_Front_Motor_Angle_Control(angletemp.angle,angletemp.time);
				  if(angletemp.angle > 1000)
						angletemp.angle = 1000; //����Ϊ���1000
					Angle = angletemp.angle ;         					
					if((Angle >= (500-5)) && (Angle <=(500+5)))
					{
						Control_Direction = 2;
            //printf("line \r\n");						
					} 
					//printf("%d\r\n",angletemp.angle);
					else if(Angle>500+5)
					{					
						Control_Direction=1;//��ת
						//printf("right \r\n");
					}					 
					else if(Angle<500-5)
					{
						Control_Direction=0;//��ת
						//printf("left \r\n");
					}						
			 }	 	 	 	
		  /***********************************************************/			 
		  if(Control_By_Steeringwheel_Or_UWB_Flag) //�Զ���ʻ
	    {   //Auto  Follow
					//printf("auto \r\n");
					if(speedqueue_dequeue(&speedtemp) == 0)
					{									
						DriveMotorSpeed = speedtemp.speed;
						if(DriveMotorSpeed ==0 )
						{
								Break_State = MACHINE_BREAK_STATE;
						}else{
								Break_State = MACHINE_NOBREAK_STATE;
						}
					   //printf("DriveMotorSpeed:%d\r\n",DriveMotorSpeed);								
						//Follow New Add CJY  ****
						//if(abs((int)(Angle-0x80)) >= 60)
						{
							//DriveMotorSpeed = DriveMotorSpeed / 2 ; 
						}
						if(Break_State != MACHINE_BREAK_STATE)
						{										
							SlowStopFlag = 0 ; 
							Machine_Enable();	
							Machine_Nobreak();
							TIM_Cmd(TIM3,ENABLE);//Ҫ����ʱ��									
								/*�Ƕ��ں�*/
							if(Control_Direction==1 )//��ת
							{									
								DutycycleL=DriveMotorSpeed*(500-abs(Angle-500))/500;
								DutycycleR=DriveMotorSpeed;								
							}else if(Control_Direction==0)//��ת
							{
								
							  DutycycleR=DriveMotorSpeed*(500-abs(Angle-500))/500;
								DutycycleL=DriveMotorSpeed;
							}
							else if(Control_Direction==2)
							{
								DutycycleL=DriveMotorSpeed;
								DutycycleR=DriveMotorSpeed; 
							}								
							//angle_factor=1;
							//printf("DutycycleL:%d DutycycleR:%d Control_Direction:%d\r\n",DutycycleL,DutycycleR,Control_Direction);		
							if(DutycycleL>=255)
							{
								DutycycleL=255;
							}
							else if( DutycycleR>=255)
							{
								DutycycleR=255;
							}//�Ƕ��ں����
							//printf("left: %d right: %d",DutycycleL,DutycycleR);
								Machine_Speed_Set(DutycycleL,DutycycleR);
							//Machine_Speed_Set(DriveMotorSpeed,DriveMotorSpeed);
						}
				 }		 
			}else //�ֶ���ʻ
			{      //Man Drive		
						 oldspeedtemp = newspeedtemp;
						 //newspeedtemp = Get_Throttle_Adc(ADC_Channel_9);
             newspeedtemp = After_filter[1];			
						 //printf("AD:%d\r\n",newspeedtemp);			
						 if(newspeedtemp < 1200)
								newspeedtemp =0;
						 else if(newspeedtemp>=1200 && newspeedtemp<3400)				 
								newspeedtemp = newspeedtemp-1000;
						 else if(newspeedtemp>=3400)
								newspeedtemp = 2400;
						 if(newspeedtemp ==0)
						 {
								Break_State = MACHINE_BREAK_STATE;
						 }		 
						 if(Manual_Break_State)// 1 ��ʱ��ɲ���ſ�
							{   
								  //break release  ������Ч
								  //������������		
                  //printf("break!\r\n");								
									if(newspeedtemp>100)  			
									{
											Break_State = MACHINE_NOBREAK_STATE;
											DriveMotorSpeed = (uint8_t)(newspeedtemp/10);					
											if(Break_State != MACHINE_BREAK_STATE)
											{																		
												SlowStopFlag = 0;
												Machine_Enable();	
												Machine_Nobreak();
												TIM_Cmd(TIM5,ENABLE);//Ҫ����ʱ��									
												/*�Ƕ��ں�*/
												if(Control_Direction==1 )//��ת
												{
													uint16_t AngleTemp;
													AngleTemp = Angle;
													if(AngleTemp>800)
													{
														AngleTemp = 800;
													}
													
													DutycycleL=DriveMotorSpeed*(500-abs(AngleTemp-500))/500;
													DutycycleR=DriveMotorSpeed;															
												}else if(Control_Direction==0)//��ת
												{
													uint16_t AngleTemp;
													AngleTemp = Angle; 
													if(AngleTemp<200)
													{
														AngleTemp = 200;
													}
													DutycycleR=DriveMotorSpeed*(500-abs(AngleTemp-500))/500;
													DutycycleL=DriveMotorSpeed;						
												}
												else if(Control_Direction==2)
												{
													DutycycleL=DriveMotorSpeed;
													DutycycleR=DriveMotorSpeed; 
												}								
												//angle_factor=1;
												//printf("DutycycleL:%d DutycycleR:%d Control_Direction:%d\r\n",DutycycleL,DutycycleR,Control_Direction);		
												if(DutycycleL>=250)
												{
													DutycycleL=250;
												}
												else if( DutycycleR>=250)
												{
													DutycycleR=250;
												}//�Ƕ��ں����
												if(!Machine_Direction_Switch_Flag)//Machine_Direction_Switch_FlagΪ0 ��ʱ��ǰ����Ϊ1��ʱ�򵹳�
												{
													Machine_Speed_Set(DutycycleL,DutycycleR);
													///printf("%d %d\r\n",DutycycleL,DutycycleR);
												}else {
													//printf("backspeed: %d %d\r\n",DutycycleL/4,DutycycleR/4);
													Machine_Speed_Set(DutycycleL/4,DutycycleR/4);
												}												
											}
									}
							}					
						//����ǰ����
						if(!Manual_Break_State)// 0 ��ʱ��ɲ������
					  {
							Break_State = MACHINE_BREAK_STATE;
							if(!Car_Change_Direction)
							{
								if(!newspeedtemp)
								{
									//�������
									 Machine_Direction_Switch(GO_BACK);
									 Machine_Direction_Switch_Flag = 1;
									 //printf("back \r\n");
								}							
							}
							else if(Car_Change_Direction)
							{
								if(!newspeedtemp)
								{
									//������ǰ
									 Machine_Direction_Switch(GO_FORWARD);
									 Machine_Direction_Switch_Flag = 0;
									 //printf("forward \r\n");
								}							
							}
					 }							
				}
				if(Break_State == MACHINE_BREAK_STATE)
				{
					//stop
					Machine_Speed_Set(0,0);
					//SlowStopFlag = 1 ;				
					//Follow New Add CJY  ****
					if(Control_By_Steeringwheel_Or_UWB_Flag)
					{
						//follow
						Machine_Break();
						TIM_Cmd(TIM5,DISABLE);					
					}
					else {
						TIM_Cmd(TIM5,DISABLE);
						//Machine_Break();
						Machine_Disable();	
					}							
			 }
}



