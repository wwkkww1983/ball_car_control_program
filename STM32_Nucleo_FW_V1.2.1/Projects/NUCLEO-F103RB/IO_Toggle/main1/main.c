#include "main.h"
#include  "motorSalgorithm.h"
#include "throttle.h"
#include <stdio.h>
#include "stm32f103rb_nucleo_eval.h"
#include "speedqueue.h"
#include "anglequeue.h"
#include "speed.h"
#include "break.h"
#include "includes.h"
#include "port.h"

static __IO  uint8_t Index = NONEINDEX ;  // 0   up   1  mid    2   down
static __IO uint32_t TimingDelay;
//extern __IO uint8_t Control_By_Steeringwheel_Or_UWB_Flag;

RCC_ClocksTypeDef RCC_Clocks;
__IO  uint32_t Machine_Front_PWM_Count = 0;
__IO  u8 TrunEnableFlag = 0 ; 

MOTOR_CONTROL_S motor1;     	 
MOTOR_CONTROL_S motor2;	     	 
MOTOR_CONTROL_S motor3;	      
MOTOR_CONTROL_S motor4;

//设置任务堆栈大小
#define START_STK_SIZE   512
//设置任务优先级
#define START_TASK_Prio     4
//任务堆栈
OS_STK  TASK_START_STK[START_STK_SIZE];
//任务申明
void TaskStart(void *pdata);//开始任务用来做速度控制相关

//设置任务堆栈大小
#define Front_Motor_Reset_STK_SIZE   64
//设置任务优先级
#define Front_Motor_ResetTASK_Prio   6
//任务堆栈
OS_STK  Front_Motor_ResetTASK_STK[Front_Motor_Reset_STK_SIZE];
//任务申明
void Front_Motor_ResetTASK(void *pdata);//前轮复位任务

//设置任务堆栈大小
#define BoxTASK_STK_SIZE   64
//设置任务优先级
#define BoxTASK_Prio     8
//任务堆栈
OS_STK  BoxTASK_STK[BoxTASK_STK_SIZE];
//任务申明
void BOX_TASK(void *pdata);//拉箱任务

//设置任务堆栈大小
#define Send_Task_STK_SIZE   64
//设置任务优先级
#define Send_Task_Prio     5
//任务堆栈
OS_STK  Send_TASK_STK[Send_Task_STK_SIZE];
//任务申明
void Send_Task(void *pdata);//串口发送数据给上位机的任务

//急停开关任务要有最高优先级
#define EmergencyStop_Task_STK_SIZE   64
//设置任务优先级
#define EmergencyStop_Task_Prio     2
//任务堆栈
OS_STK  EmergencyStop_TASK_STK[EmergencyStop_Task_STK_SIZE];
//任务申明
void EmergencyStop_Task(void *pdata);

int main(void)
{
		SystemCoreClockUpdate();
		/* SysTick end of count event each 1ms */
		RCC_GetClocksFreq(&RCC_Clocks);
		SysTick_Config(RCC_Clocks.HCLK_Frequency / OS_TICKS_PER_SEC);
	  OSInit();	   //UCOSII初始化
	  //时钟使能
	  {			
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOE|
														 RCC_APB2Periph_GPIOB |RCC_APB2Periph_AFIO
			                       |RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD
			                       |RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOG, ENABLE);
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5|RCC_APB1Periph_TIM4, ENABLE);
      NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);//设置中断优先组0   0 bit 抢占中断 4bit响应中断		
	  }
			Middle_Motor_Init(); 
			Middle_Motor_Stop();
			Middle_Motor_Break();
		  
	  //USART配置
	  {
			USART_InitTypeDef USART_InitStructure;
			NVIC_InitTypeDef NVIC_InitStructure;		
			//Usart1 NVIC 配置    
			NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
			NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器	
	    
	    //uart_init(115200);
      USART_InitStructure.USART_BaudRate = 9600;
      USART_InitStructure.USART_WordLength = USART_WordLength_8b;
      USART_InitStructure.USART_StopBits = USART_StopBits_1;
      USART_InitStructure.USART_Parity = USART_Parity_No;
      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
      USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
      STM_EVAL_COMInit(COM1, &USART_InitStructure);
		
		  //串口2的配置
		  USART_InitStructure.USART_BaudRate = 9600;
      USART_InitStructure.USART_WordLength = USART_WordLength_8b;
      USART_InitStructure.USART_StopBits = USART_StopBits_1;
      USART_InitStructure.USART_Parity = USART_Parity_No;
      USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
      USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
      STM_EVAL_COMInit(COM2, &USART_InitStructure);
	  }  
	    printf("Starting...\r\n");	
	    //Timer5 Control Back PWM    后轮两路PWM  PA0--后右边电机  PA1--后左边电机 用Timer5来产生PWM波
	    //Timer4 Control front PWM   前轮一路PWM  PB8
	{		
		//PWM输出引脚配置 PB8为TIM4的PWM输出  GPIO Configuration
    { 
			GPIO_InitTypeDef GPIO_InitStructure;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Pin =  fr_ahead_motor_pwm_out;
			GPIO_Init(ahead_motor_pwm_port, &GPIO_InitStructure);
			
			GPIO_InitStructure.GPIO_Pin =  fr_right_motor_pwm_out;
			GPIO_Init(right_motor_pwm_port, &GPIO_InitStructure);
			
			GPIO_InitStructure.GPIO_Pin =  fr_left_motor_pwm_out;
			GPIO_Init(left_motor_pwm_port, &GPIO_InitStructure);
	  }
	 
	  //TIMER5配置  Timer 5 Configuration   PA0---TIMER5-CH1   PA1---TIMER5-CH2
	  {
			TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
			TIM_OCInitTypeDef  TIM_OCInitStructure;
			uint16_t TimerPeriod = 0;
			uint16_t Channel1Pulse = 0, Channel2Pulse = 0;
		 
      /* TIM5 Configuration ---------------------------------------------------
		 
		  TIM1CLK = SystemCoreClock, Prescaler = 0, TIM1 counter clock = SystemCoreClock
		  SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
		  and Connectivity line devices and to 24 MHz for Low-Density Value line and
		  Medium-Density Value line devices
		 
		  The objective is to generate 7 PWM signal at 17.57 KHz:
			 - TIM5_Period = (SystemCoreClock / 17570) - 1
		  The Timer pulse is calculated as follows:
			 - ChannelxPulse = DutyCycle * (TIM5_Period - 1) / 100
		  ----------------------------------------------------------------------- */
		
			/* Compute the value to be set in ARR regiter to generate signal frequency at 17.57 Khz */
			TimerPeriod = (SystemCoreClock / SPEED_TIMER_PERIOD ) - 1;
			/* Compute CCR1 value to generate a duty cycle at 50% for channel 1 and 1N */
			Channel1Pulse = (uint16_t) (((uint32_t) 5 * (TimerPeriod - 1)) / 10);
			/* Compute CCR2 value to generate a duty cycle at 37.5%  for channel 2 and 2N */
			Channel2Pulse = (uint16_t) (((uint32_t) 375 * (TimerPeriod - 1)) / 1000);
			
			/* Time Base configuration */
			TIM_TimeBaseStructure.TIM_Prescaler = 0;
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
			TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
			TIM_TimeBaseStructure.TIM_ClockDivision = 0;
			TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

			TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

			/* Channel 1, 2,3 and 4 Configuration in PWM mode */   //TCNT  TCMP  AUTORELOAD
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
			TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
			TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
			TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
			TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

			TIM_OC1Init(TIM5, &TIM_OCInitStructure);

			TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
			TIM_OC2Init(TIM5, &TIM_OCInitStructure);
			
			/* TIM5 counter enable */
			TIM_Cmd(TIM5, DISABLE);
			/* TIM5 Main Output Enable */
			TIM_CtrlPWMOutputs(TIM5, ENABLE);	
   }//TIM5配置结束
	 
	  //TIM4配置 通道3输出，PB8输出
	 {
			NVIC_InitTypeDef NVIC_InitStructure;
			TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
			TIM_OCInitTypeDef  TIM_OCInitStructure;
			uint16_t TimerPeriod = 0;
			uint16_t Channel3Pulse = 0;
			TimerPeriod = (SystemCoreClock /1000/ 1000) - 1;
			/* Compute CCR1 value to generate a duty cycle at 50% for channel 1 and 1N */
			Channel3Pulse = (uint16_t) (((uint32_t) 5* (TimerPeriod - 1)) / 10);
			/* Time Base configuration */
			TIM_TimeBaseStructure.TIM_Prescaler = 999;
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
			TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
			TIM_TimeBaseStructure.TIM_ClockDivision = 0;
			TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

			TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

			/* Channel 1, 2,3 and 4 Configuration in PWM mode */
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
			TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
			TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
			TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
			TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

			TIM_OC3Init(TIM4, &TIM_OCInitStructure);	
			
			/* Enable the TIM4 gloabal Interrupt */
			NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
			NVIC_Init(&NVIC_InitStructure); 		 
			/*
			TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);//要不要使能重装载?
      TIM_ARRPreloadConfig(TIM4, ENABLE);
		  */
			
			/* TIM4 Main Output Enable */
			TIM_CtrlPWMOutputs(TIM4, ENABLE);
			//清中断以免一开中断后就进入中断
			TIM_ClearFlag(TIM4, TIM_FLAG_Update);
			TIM_ITConfig(TIM4,TIM_IT_Update, ENABLE);			
			/* TIM4 counter enable */
			TIM_Cmd(TIM4, ENABLE);		   
		}//TIME4 配置结束
	}//PWM配置结束		     
	     Backmotor_Power_Init();	
	     Backmotor_Power_On();
       Aheadmotor_Power_Init();
       Aheadmotor_Power_On();
	     Ahead_Light_Power_Init();
	     Ahead_Light_Power_On();
	     
       //Aheadmotor_Power_OFF();     
       Machine_Front_Motor_Dirtion_Init();
	     Machine_Front_Motor_Directon_Control(Machine_Front_Motor_Turn_Right);	
       Machine_Front_Motor_Enable_Or_Disable_Init();
	     Machine_Front_Motor_Enable_or_Disable(ENABLE);
       //Machine_Front_Motor_Reset_Dectected_Init();
		   //MIC_Motor_Init();	    
	     Machine_Enable_Init();
			 Machine_Enable();
			 //Back Free
	     Machine_Break_Init();
			 Machine_Nobreak();
			 //Direction 
	     Machine_Diretion_Init();
			 Machine_Direction_Switch(GO_FORWARD);
	     
			 Middle_Motor_Init();
			 Detected_Rain_Init();
			 Middle_Motor_Run();
			 Middle_Motor_Stop();
       Middle_Motor_NO_Break();
			 
			 Electric_Push_Rod_Init();
			 //Electric_Push_Rod_Up();
			 //Electric_Push_Rod_Down();
			 Electric_Push_Rod_Stop();
			 
			 MIC_Motor_Init();
			 MIC_Motor_Close_Detected_Init();
			 
			 Car_Forward_Back_Detected_Init();
		   Open_Car_Lid_Detected_Init();
			 Close_Car_Lid_Detected_Init();
			 Detected_Car_Lid_Init();
			 People_Seatsensor_Detected_Init();
			 Detected_Carbreak_Detected_Init();
			 Stopnow_Detected_Init();
			 Obstacle_From_Ultrasonic_Detected_Init();
			 Switch_Auto_Manual_Drivecar_Detected_Init();
			 
			 //Middle_Motor_Break();
		 	 /*初始化电机运行参数，主要是根据S型曲线参数生成表格*/
	     MotorRunParaInitial();
			 Initial_Motor(4,M4DIV,400);//400 转一圈			 
		   //Throttle_Adc_Init();
	     Machine_Adc_Init();
		   //Manual_Break_Init();			
		   //Manual_Change_Machine_Back_or_Forward_Init();
			 //Machine_GetSpeed_Init(); 			 
       printf("initial completed!!\r\n");
       OS_ENTER_CRITICAL(); 			 
	     OSTaskCreate(TaskStart, //创建开始任务
									 (void *)0,	 //parameter
									 (OS_STK *)&TASK_START_STK[START_STK_SIZE-1],	//task stack top pointer
									 START_TASK_Prio);//task priority
			 OS_EXIT_CRITICAL();						
	     OSStart();//UCOSII系统启动									
       while(1)
         ;
}

/* 球包电动推杆 */
/*功能:电子推杆初始化
 */
void Electric_Push_Rod_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = up_electric_push_rod_left_io_out|down_electric_push_rod_left_io_out;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(electric_push_rod_out_port, &GPIO_InitStructure); 
	/*左边电流检测和霍尔传感器检测 在这里写*/
	
	GPIO_InitStructure.GPIO_Pin = off_electric_push_rod_left_io_input|on_electric_push_rod_left_io_input;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(electric_push_rod_detected_port, &GPIO_InitStructure);
	
	/*以上是左边的电动推杆的初始化，以下是右边的电动推杆的初始化*/
	GPIO_InitStructure.GPIO_Pin = up_electric_push_rod_right_io_out
	                             |down_electric_push_rod_right_io_out;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(electric_push_rod_out_port, &GPIO_InitStructure); 
	/*右边电流检测和霍尔传感器检测 在这里写*/
	
	GPIO_InitStructure.GPIO_Pin = off_electric_push_rod_right_io_input
	                             |on_electric_push_rod_right_io_input;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(electric_push_rod_detected_port, &GPIO_InitStructure);
}

/*功能：盖子打开
 */
void Electric_Push_Rod_Up(void)
{
  GPIO_SetBits(electric_push_rod_out_port,up_electric_push_rod_left_io_out|up_electric_push_rod_right_io_out);
  GPIO_ResetBits(electric_push_rod_out_port,down_electric_push_rod_left_io_out|down_electric_push_rod_right_io_out);
}

/*功能：盖子关
 */
void Electric_Push_Rod_Down(void)
{
	GPIO_ResetBits(electric_push_rod_out_port,up_electric_push_rod_left_io_out|up_electric_push_rod_right_io_out);
  GPIO_SetBits(electric_push_rod_out_port,down_electric_push_rod_left_io_out|down_electric_push_rod_right_io_out);
}

/* 功能：盖子停
 */
void Electric_Push_Rod_Stop(void)
{
	GPIO_ResetBits(electric_push_rod_out_port,up_electric_push_rod_left_io_out|up_electric_push_rod_right_io_out);
  GPIO_ResetBits(electric_push_rod_out_port,down_electric_push_rod_left_io_out|down_electric_push_rod_right_io_out);
}

/* 功能：检测到限位开关停
 */
void Electric_Push_Rod_Detected_Stop(void)
{
	if(!GPIO_ReadInputDataBit(electric_push_rod_detected_port,off_electric_push_rod_right_io_input))
	{
		Electric_Push_Rod_Stop();
	}
	if(!GPIO_ReadInputDataBit(electric_push_rod_detected_port,on_electric_push_rod_right_io_input))
	{
		Electric_Push_Rod_Stop();
	}
	if(!GPIO_ReadInputDataBit(electric_push_rod_detected_port,off_electric_push_rod_left_io_input))
	{
		Electric_Push_Rod_Stop();
	}
	if(!GPIO_ReadInputDataBit(electric_push_rod_detected_port,on_electric_push_rod_left_io_input))
	{
		Electric_Push_Rod_Stop();
	}
}

/* 功能：打开盖子按键初始化
 */
void Open_Car_Lid_Detected_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = open_car_lid_button_io;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(open_car_lid_port, &GPIO_InitStructure);
}

/* 功能：打开盖子按键检测
 */
uint8_t Open_Car_Lid_Detected(void)
{
	uint8_t temp;
	static uint8_t state = 0;
	temp = GPIO_ReadInputDataBit(open_car_lid_port,open_car_lid_button_io);
	return temp;
}

/*功能：关闭盖子按键初始化
 */
void Close_Car_Lid_Detected_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = close_car_lid_button_io;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(close_car_lid_port, &GPIO_InitStructure);
}

/*功能：关闭盖子按键检测
 */
uint8_t Close_Car_Lid_Detected(void)
{
	uint8_t temp;
	temp = GPIO_ReadInputDataBit(close_car_lid_port,close_car_lid_button_io);
	return temp;
}

/* 功能：盖子sensor初始化
 */
void Detected_Car_Lid_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = detected_car_lid_io_input;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(detected_car_lid_port, &GPIO_InitStructure);
}

/* 功能：盖子sensor检测
 */
uint8_t Detected_Car_Lid(void)
{
	uint8_t temp;
	temp = GPIO_ReadInputDataBit(detected_car_lid_port,detected_car_lid_io_input);
	return temp;
}
/*球包电动推杆结束*/

/*USB电源*/
/* 功能：usb 电源初始化
 */
void Usb_Power_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = switch_usb_power_io_out;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(usb_power_port, &GPIO_InitStructure); 
}

/*功能：usb电源开
 */
void Usb_Power_On(void)
{
	GPIO_SetBits(usb_power_port,switch_usb_power_io_out);
}

/*功能：usb电源关
 */
void Usb_Power_OFF(void)
{
  GPIO_ResetBits(usb_power_port,switch_usb_power_io_out);
}
/*USB电源*/

/*超声波电源*/
/* 功能：超声波电源初始化
 */
void Ultrasonsic_Power_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = switch_ultrasonic_power_io_out;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(ultrasonic_power_port, &GPIO_InitStructure);  	
}

/*功能：超声波电源开
 */
void Ultrasonsic_Power_On(void)
{
	GPIO_SetBits(ultrasonic_power_port,switch_ultrasonic_power_io_out);
}
/* 功能：超声波电源关
 */
void Ultrasonsic_Power_OFF(void)
{
	GPIO_ResetBits(ultrasonic_power_port,switch_ultrasonic_power_io_out);
}
/*超声波电源*/

/*后轮电源*/
/*功能：后轮电机电源初始化
 */
void Backmotor_Power_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = switch_backmotor_power_io_out;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(backmotor_power_port, &GPIO_InitStructure);	
}

/*功能：后轮电机电源开
 */
void Backmotor_Power_On(void)
{
	GPIO_SetBits(backmotor_power_port,switch_backmotor_power_io_out);
}

/*功能：后轮电机电源关
 */
void Backmotor_Power_OFF(void)
{
	GPIO_ResetBits(backmotor_power_port,switch_backmotor_power_io_out);
}

/*后轮电源*/


/*前轮电源*/
/* 功能：前轮电机电源初始化
 */
void Aheadmotor_Power_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = switch_aheadmotor_power_io_out;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(aheadmotor_power_port, &GPIO_InitStructure); 	
}


/*功能：前轮电机电源开
 */
void Aheadmotor_Power_On(void)
{
	GPIO_SetBits(aheadmotor_power_port,switch_aheadmotor_power_io_out);
}

/*功能：前轮电源关
 */
void Aheadmotor_Power_OFF(void)
{
	GPIO_ResetBits(aheadmotor_power_port,switch_aheadmotor_power_io_out);
}
/*前轮电源*/


/*功能：前面车灯电源初始化
 */
void Ahead_Light_Power_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = switch_ahead_light_power_io_out;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(ahead_light_power_port, &GPIO_InitStructure); 
}
/*功能：前面车灯电源开
 */
void Ahead_Light_Power_On(void)
{
	GPIO_SetBits(ahead_light_power_port,switch_ahead_light_power_io_out);
}
/* 功能：前面车灯电源关
 */
void Ahead_Light_Power_OFF(void)
{
	GPIO_ResetBits(ahead_light_power_port,switch_ahead_light_power_io_out);
}

/*功能：24V电源初始化 原理图上面没连
 */
void Car24v_Power_Init(void)
{
   
}

/*功能：MIC电机初始化
 */
void MIC_Motor_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF|RCC_APB2Periph_GPIOA, ENABLE);
	
  GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = mic_motor_io_out1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(mic_motor_out1_port, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = mic_motor_io_out2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(mic_motor_out2_port, &GPIO_InitStructure); 
	
}

/*功能：MIC电机运行
 */
void MIC_Motor_Run(void)
{
	GPIO_SetBits(mic_motor_out1_port,mic_motor_io_out1);
	//GPIO_ResetBits(mic_motor_out1_port,mic_motor_io_out1);
  //GPIO_ResetBits(mic_motor_out2_port,mic_motor_io_out2);
}

/*功能：MIC电机停止
 */
void MIC_Motor_Stop(void)
{
	GPIO_ResetBits(mic_motor_out1_port,mic_motor_io_out1);
  //GPIO_SetBits(mic_motor_out2_port,mic_motor_io_out2);
}

/*功能：MIC 关闭检测初始化
 */
void MIC_Motor_Close_Detected_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = mic_close_detected_io_input;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(mic_close_detected_port, &GPIO_InitStructure);
}

/*功能：MIC 关闭检测
 *返回：0 没有检测到 1 有检测到
 */
uint8_t MIC_Motor_Close_Detected(void)
{
	uint8_t temp;
	temp = GPIO_ReadInputDataBit(mic_close_detected_port,mic_close_detected_io_input);
	return temp;
}
/***/

/* 功能：前进后退按键初始化
 */
void Car_Forward_Back_Detected_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = car_forward_back_button_io;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(car_forward_back_port, &GPIO_InitStructure);
}

/*功能：前进后退按键检测
 */
uint8_t Car_Forward_Back_Detected(void)
{
	uint8_t temp;
	temp = GPIO_ReadInputDataBit(car_forward_back_port,car_forward_back_button_io);
	return temp;
}

/* 功能：座椅传感器初始化
 */
void People_Seatsensor_Detected_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = detected_people_seatsensor_io_input;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(detected_people_seatsensor_port, &GPIO_InitStructure);
}

/* 功能：座椅传感器检测
 */
uint8_t People_Seatsensor_Detected(void)
{
	uint8_t temp;
	temp = GPIO_ReadInputDataBit(detected_people_seatsensor_port,detected_people_seatsensor_io_input);
	return temp;
}

/* 功能：刹车检测初始化
 */
void Detected_Carbreak_Detected_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = detected_carbreak_io_input;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(detected_carbreak_port, &GPIO_InitStructure);
}

/*功能：刹车检测
 */
uint8_t Detected_Carbreak_Detected(void)
{
	uint8_t temp;
	temp = GPIO_ReadInputDataBit(detected_carbreak_port,detected_carbreak_io_input);
	return temp;
}

/* 功能：急停按钮初始化
 */
void Stopnow_Detected_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = stopnow_button_io;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(stopnow_port, &GPIO_InitStructure);
}

/*功能：急停检测
 */
uint8_t Stopnow_Detected(void)
{
	uint8_t temp;
	temp = GPIO_ReadInputDataBit(stopnow_port,stopnow_button_io);
	return temp;
}

/* 功能：超声波检测到障碍物信号检测初始化
 */
void Obstacle_From_Ultrasonic_Detected_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = detected_obstacle_from_ultrasonic_io_input;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(detected_obstacle_from_ultrasonic_port, &GPIO_InitStructure);
}

/* 功能：超声波检测
 */
uint8_t Obstacle_From_Ultrasonic_Detected(void)
{
	uint8_t temp;
	temp = GPIO_ReadInputDataBit(detected_obstacle_from_ultrasonic_port,detected_obstacle_from_ultrasonic_io_input);
	return temp;
}

/*  功能：手自动切换初始化
 */
void Switch_Auto_Manual_Drivecar_Detected_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = switch_auto_manual_drivecar_button_io;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(switch_auto_manual_drivecar_port, &GPIO_InitStructure);
}

/* 功能：手自动检测
 */
uint8_t Switch_Auto_Manual_Drivecar_Detected(void)
{
	uint8_t temp;
	temp = GPIO_ReadInputDataBit(switch_auto_manual_drivecar_port,switch_auto_manual_drivecar_button_io);
	return temp;
}
/***/

///*其他AD检测*/
//void  AD48V_Adc_Init(void)
//{ 	
//	ADC_InitTypeDef ADC_InitStructure; 
//	GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_ADC1	, ENABLE );	  //使能ADC1通道时钟
// 
//	RCC_ADCCLKConfig(RCC_PCLK2_Div8);   //64M/8=8,ADC最大时间不能超过14M
//	//PA0/1/2/3 作为模拟通道输入引脚                         
//	GPIO_InitStructure.GPIO_Pin = AD48V;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
//	GPIO_Init(AD48V_port, &GPIO_InitStructure);	

//	ADC_DeInit(ADC1);  //将外设 ADC1 的全部寄存器重设为缺省值
//  
//	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
//	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//模数转换工作在单通道模式
//	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
//	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
//	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
//	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
//	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   
// 

//	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
//	
//	ADC_ResetCalibration(ADC1);	//重置指定的ADC1的校准寄存器
//	 
//	while(ADC_GetResetCalibrationStatus(ADC1));	//获取ADC1重置校准寄存器的状态,设置状态则等待
//	
//	ADC_StartCalibration(ADC1);		//开始指定ADC1的校准状态
// 
//	while(ADC_GetCalibrationStatus(ADC1));		//获取指定ADC1的校准程序,设置状态则等待
// 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能
//}

///*通道8*/
//u16 Get_AD48V_Adc(u8 ch)   
//{
//  //设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
//	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道3,规则采样顺序值为1,采样时间为239.5周期	  			    
//  
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能	
//	 
//	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

//	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
//}

//void  battery_temperature_Adc_Init(void)
//{ 	
//	ADC_InitTypeDef ADC_InitStructure; 
//	GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	, ENABLE );	  //使能ADC1通道时钟
// 
//	RCC_ADCCLKConfig(RCC_PCLK2_Div8);   //64M/8=8,ADC最大时间不能超过14M
//	//PA0/1/2/3 作为模拟通道输入引脚                         
//	GPIO_InitStructure.GPIO_Pin = battery_temperature_detected;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
//	GPIO_Init(battery_temperature_detected_port, &GPIO_InitStructure);	

//	ADC_DeInit(ADC1);  //将外设 ADC1 的全部寄存器重设为缺省值

//	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
//	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
//	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
//	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
//	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
//	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
//	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   
// 
//	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
//	
//	ADC_ResetCalibration(ADC1);	//重置指定的ADC1的校准寄存器
//	 
//	while(ADC_GetResetCalibrationStatus(ADC1));	//获取ADC1重置校准寄存器的状态,设置状态则等待
//	
//	ADC_StartCalibration(ADC1);		//开始指定ADC1的校准状态
// 
//	while(ADC_GetCalibrationStatus(ADC1));		//获取指定ADC1的校准程序,设置状态则等待
// 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能
//  
//}

///*通道14*/
//u16 Get_battery_temperature_Adc(u8 ch)   
//{
//  //设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
//	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道3,规则采样顺序值为1,采样时间为239.5周期	  			    
//  
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能	
//	 
//	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

//	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
//}
/**/

/*功能：雨量检测初始化
 */
void Detected_Rain_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
 	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = detected_rain_io_input;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(detected_rain_port, &GPIO_InitStructure);
}

/*功能：前轮复位sensor初始化
 */
void Machine_Front_Motor_Reset_Dectected_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = detected_middle_ahead_motor_io_input
	                              |detected_left_ahead_motor_io_input
	                              |detected_right_ahead_motor_io_input;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(detected_ahead_motor_port, &GPIO_InitStructure);
}

/* 功能：前轮使能初始化
 */
void Machine_Front_Motor_Enable_Or_Disable_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = enb_ahead_motor_io_out;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(enb_ahead_motor_port, &GPIO_InitStructure);
}

/* 功能：前轮方向初始化
 */
void Machine_Front_Motor_Dirtion_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = dir_ahead_motor_io_out;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(dir_ahead_motor_port, &GPIO_InitStructure);
}


/* 功能：使能前轮或者不使能前轮
 */
void Machine_Front_Motor_Enable_or_Disable(FunctionalState NewState)
{
	if(NewState)
	{
		GPIO_ResetBits(enb_ahead_motor_port,enb_ahead_motor_io_out);
	}else{	
		GPIO_SetBits(enb_ahead_motor_port,enb_ahead_motor_io_out);
	}
}

/* 功能：前轮电机的转向
 * 
 *
 */
void Machine_Front_Motor_Directon_Control(uint8_t Dir)
{
	if(Dir == Machine_Front_Motor_Turn_Right)
	{
		GPIO_SetBits(dir_ahead_motor_port,dir_ahead_motor_io_out);
	}else{
		GPIO_ResetBits(dir_ahead_motor_port,dir_ahead_motor_io_out);
	}
}


/* 功能：后轮方向初始化
 *
 */
void Machine_Diretion_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	GPIO_InitStructure.GPIO_Pin =  dir_right_motor_io_out;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(right_motor_port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  dir_left_motor_io_out;
	GPIO_Init(left_motor_port, &GPIO_InitStructure);
}
/* 功能：后轮电机方向转换
 * 
 */
void Machine_Direction_Switch(uint8_t Dir)
{
	//关使能
	Machine_Disable();
	switch(Dir)
	{
		case GO_LEFT:
			//GPIO_SetBits(GPIOC,Direction1|Direction2);
		 break;
		case GO_RIGHT:
			//GPIO_ResetBits(GPIOC,Direction1|Direction2);
			break;
		case GO_FORWARD:
			GPIO_SetBits(right_motor_port,dir_right_motor_io_out);
		  GPIO_ResetBits(left_motor_port,dir_left_motor_io_out);		  
			break;
		case GO_BACK:
		  GPIO_ResetBits(right_motor_port,dir_right_motor_io_out);		 
		  GPIO_SetBits(left_motor_port,dir_left_motor_io_out);
			break;
		default:
			break;
	}
	Machine_Enable();
}

/* 功能：后轮使能初始化
 */
void Machine_Enable_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	GPIO_InitStructure.GPIO_Pin =  enb_right_motor_io_out;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(right_motor_port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = enb_left_motor_io_out;
	GPIO_Init(left_motor_port,&GPIO_InitStructure);
}


/*功能：后轮使能
 */
void Machine_Enable(void)
{
	GPIO_SetBits(right_motor_port,enb_right_motor_io_out);
	GPIO_SetBits(left_motor_port,enb_left_motor_io_out);
}

/*功能：后轮不使能
 */
void Machine_Disable(void)
{
	GPIO_ResetBits(right_motor_port,enb_right_motor_io_out);
	GPIO_ResetBits(left_motor_port,enb_left_motor_io_out);	
}

/* 功能：刹车初始化
 */
void Machine_Break_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	GPIO_InitStructure.GPIO_Pin =  break_right_motor_io_out;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(right_motor_port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  break_left_motor_io_out;
	GPIO_Init(left_motor_port, &GPIO_InitStructure);
}

/* 功能：没刹车
 *
 */
void Machine_Nobreak(void)
{
   //GPIO_ResetBits(GPIOC,Break1|Break2);	
   GPIO_ResetBits(right_motor_port, break_right_motor_io_out); 	
	 GPIO_ResetBits(left_motor_port,  break_left_motor_io_out);
}

/* 功能：刹车
 *
 */
void Machine_Break(void)
{
	 GPIO_SetBits(right_motor_port, break_right_motor_io_out); 	
	 GPIO_SetBits(left_motor_port,  break_left_motor_io_out);
}

/*功能：后轮警告初始化
 */
void Machine_Motor_Alarm_Detected_Init( void )
{
  GPIO_InitTypeDef GPIO_InitStructure;	
	GPIO_InitStructure.GPIO_Pin = alarm_right_motor_io_input;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	
	GPIO_Init(right_motor_port, &GPIO_InitStructure);	
	GPIO_InitStructure.GPIO_Pin = alarm_left_motor_io_input;
	GPIO_Init(left_motor_port, &GPIO_InitStructure);
}

/*  功能：后轮电机警告检测
 *  返回值：返回0，没有警告，返回1，右轮有警告，返回2，左轮有警告，返回3，两个轮子都有警告
 */
uint8_t Machine_Motor_Alarm_Detected(void)
{
	uint8_t temp;
   
	return temp;
}

/*功能：后轮电机PG初始化
 */
void Machine_Motor_PG_Dectected_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	GPIO_InitStructure.GPIO_Pin =  pg_right_motor_io_input;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	
	GPIO_Init(right_motor_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =  pg_left_motor_io_input;
	GPIO_Init(left_motor_port, &GPIO_InitStructure);
}

/*功能：后轮PG检测
 * 返回0,没有检测到，返回1，右轮检测到，返回2，左轮检测到，返回3，两个轮都检测到
 */
uint8_t Machine_Motor_PG_Dectected(void)
{
	uint8_t temp;
	return temp;
}

/* 功能：设置速度， 输入的是占空比.最大为250
 *
 */
void Machine_Speed_Set(uint16_t DutycycleL,uint16_t DutycycleR)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t TimerPeriod = 0;
	uint16_t Channel1Pulse = 0, Channel2Pulse = 0;
	TimerPeriod = (SystemCoreClock / SPEED_TIMER_PERIOD);
  /* Compute CCR1 value to generate a duty cycle at Dutycycle‰ for channel 1 and 1N */
  Channel1Pulse = (uint16_t) (((uint32_t) (250-DutycycleL )* (TimerPeriod)) / 250);
  /* Compute CCR2 value to generate a duty cycle at Dutycycle‰  for channel 2 and 2N */
  Channel2Pulse = (uint16_t) (((uint32_t) (250-DutycycleR ) * (TimerPeriod)) / 250);
	TIM5->CCR2 = Channel2Pulse;  
  TIM5->CCR1 = Channel1Pulse;
}

/** 功能：前轮的方向控制
 *  参数：angle：转弯的角度
 *        time：到达要转弯的角度要求的时间  
 */
uint16_t  LastAngle = 0x80; 
int16_t   DeltaAngle = 0; 
uint8_t   Direction = 0; 
uint8_t   Direction_Flag = 0; 

static int Array[3] = {600,1200,1800};
//Follow New Add CJY ****
//extern uint8_t Control_By_Steeringwheel_Or_UWB_Flag;  // if flag is 1 then Follow  if flag is 0 then drive
void Machine_Front_Motor_Angle_Control(uint16_t angle,uint16_t time)
{
	time=time;
	if(angle<2)
		 angle=0;
	else if(angle>=999)
		angle=999;
	angle = (uint16_t)(angle/12.5)+0;
	/* 改变转弯角度的方法
	 * 方法一：  通过这个公式来改变 angle = (uint16_t)(angle/5)+0 缺点：当比例因子小的时候步进电机再比较小和比较大的两个边界会打滑
	 * angle = (uint16_t)(angle/5)+0; //当转速比为20：1的时候，转过的最大角度是a:27°（物理角度），程序逻辑角度为30。5，为上位机发送过来的角度uA和逻辑角度lA的比例因子。
	                                  //0,为修正因子,作用让两边转的角度对称
   * 6,是以前机械的减速比，20，是现在机械的减速比。
	 * SetPosition(unsigned char MotorID,unsigned int dest) ：dest为电机要运行的位置
	 * 当dest为400的时候，电机转360度，这个跟Initial_Motor(4,M4DIV,400) 初始化有关 当dest为100的时候，电机转90度
	 * 在不改变M4DIV的情况下且Initial_Motor 初始化函数里面的中点位置为100，逻辑角度 lA = （500/比例因子）* 6/20,物理角度 a = lA * 90 /100。
 	 * 可以先定比例因子在计算逻辑角度，也可以先定物理角度，逻辑调度，再算比例因子。
	 *
	 *方法二：调试应用笔记：1.首先确定驱动器的细分和减速比跟软件的细分和转动最大的位置的关系。此程序选用的驱动器的细分是1600，减速比是1:20，程序的最大位置是400，细分是160.
	 *                    2.然后确定要转的角度，比如此程序左右要转36度。因为中点是100，比例因子是5的时候左右转的是90度，所以此时要将比例因子调成12.5，中点调成40，则左右转动36度。
	 */
	 SetPosition(4,angle);
}

uint8_t  Machine_Front_Motor_Reset_Detected(void);
__IO uint8_t Machine_Front_Motor_State=0;
/* 功能：初始化拉箱的电机
 * 
 *
 */
void Middle_Motor_Init(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
 	/*Enable GPIOB clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOG, ENABLE);	
	GPIO_InitStructure.GPIO_Pin =  run1_ballbag_motor_io_out|run2_ballbag_motor_io_out;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(run_dir_port, &GPIO_InitStructure);	
	GPIO_InitStructure.GPIO_Pin = break_ballbag_motor_io_out;
	GPIO_Init(break_port, &GPIO_InitStructure);	
	GPIO_InitStructure.GPIO_Pin = detected_limited_ahead_ballbag_motor_io_input|
                               	detected_ahead_ballbag_motor_io_input|      
	                              detected_center_ballbag_motor_io_input|
	                              detected_back_ballbag_motor_io_input|
	                              detected_limited_back_ballbag_motor_io_input;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(detected_ballbag_port, &GPIO_InitStructure);	
	GPIO_InitStructure.GPIO_Pin = up_button_ballbag_motor_io;	
	GPIO_InitStructure.GPIO_Pin = down_button_ballbag_motor_io;
	GPIO_Init(button_ballbag_port, &GPIO_InitStructure);
}

static void Middle_Motor_Run(void)
{
  //GPIO_SetBits(run_dir_port,run1_ballbag_motor_io_out);
}

static void Middle_Motor_Break(void)
{
	GPIO_SetBits(break_port,break_ballbag_motor_io_out);
}

static void Middle_Motor_NO_Break(void)
{
  GPIO_ResetBits(break_port,break_ballbag_motor_io_out);
}

static void Middle_Motor_Stop(void)
{
	GPIO_ResetBits(run_dir_port,run1_ballbag_motor_io_out);
	GPIO_ResetBits(run_dir_port,run2_ballbag_motor_io_out);
}

static void Middle_Motor_Goforward(void)
{
	GPIO_SetBits(run_dir_port,run2_ballbag_motor_io_out);
	GPIO_ResetBits(run_dir_port,run1_ballbag_motor_io_out);
}

static void Middle_Motor_Goback(void)
{
  GPIO_ResetBits(run_dir_port,run2_ballbag_motor_io_out);
	GPIO_SetBits(run_dir_port,run1_ballbag_motor_io_out);
}

__IO uint8_t Manual_Break_State       = 1; // 刚开始的时候是1 不刹车的。
__IO uint8_t Car_Change_Direction     = 1; // 刚开始的时候是1 车子是向前的。
__IO uint8_t Car_Change_Direction_Old = 0;

void TaskStart(void *pdata)
{
		pdata = pdata;
	  INT8U Mutex_Err;
	  OS_ENTER_CRITICAL(); 			 
		OSTaskCreate(Front_Motor_ResetTASK,	//创建前轮复位任务
								(void *)0,	            //parameter
								(OS_STK *)&Front_Motor_ResetTASK_STK[Front_Motor_Reset_STK_SIZE-1],	//task stack top pointer
								Front_Motor_ResetTASK_Prio );	//task priority
		OSTaskCreate(BOX_TASK,	//创建拉箱任务
								(void *)0,	//parameter
								(OS_STK *)&BoxTASK_STK[BoxTASK_STK_SIZE-1],	//task stack top pointer
								BoxTASK_Prio );	//task priority
    OSTaskCreate(Send_Task,	//创建串口发送数据给上位机任务
								(void *)0,	//parameter
								(OS_STK *)&Send_TASK_STK[Send_Task_STK_SIZE-1],	//task stack top pointer
								Send_Task_Prio );	//task priority
								
    OSTaskCreate(EmergencyStop_Task,	
								(void *)0,	//parameter
								(OS_STK *)&EmergencyStop_TASK_STK[EmergencyStop_Task_STK_SIZE-1],	//task stack top pointer
								EmergencyStop_Task_Prio);	//task priority										
	  OS_EXIT_CRITICAL();								
    //Backmotor_Power_On();
		while(1)
		{			
      int i;			
			//		OSSchedLock();//OSSchedUnl();
			//    printf("start task\r\n");		
			//    OSSchedUnlock();
			//    OSTaskResume(Front_Motor_ResetTASK_Prio);
			//    OSTaskResume(CarLidTASK_Prio);
      Manual_Break_State = Detected_Carbreak_Detected();			
			if(!Manual_Break_State)// 0 的时候刹车踩下
			{
				Car_Change_Direction = Car_Forward_Back_Detected();	
			}
			//Get_Machine_Adc();
			filter();			
			//			for(i=0;i<M;i++)
			//			{
			//				OSSchedLock();
			//				printf("value[%d]:%d\r\n",i,After_filter[i]);
			//				OSSchedUnlock();				
			//				OSTimeDly(4000);
			//				//OSTimeDlyHMSM(0,0,0,5000);	
			//			}		
      //printf("\r\n");
			//printf("value:%d\r\n",After_filter[6]);
			//OSTimeDly(200);
			//printf("Manual_Break_State %d\r\n",Manual_Break_State);
			//printf("Car_Change_Direction %d\r\n",Car_Change_Direction);
	    time_tick_active();	
			//Machine_Front_Motor_Angle_Control(500,500);
			//printf("%d\r\n",Get_Throttle_Adc(ADC_Channel_9));
      //OSSchedLock();//OSSchedUnlock();
			//printf("start task!!\r\n");
      //OSSchedUnlock();
			//printf("yeyiliang\r\n");
			//OSTimeDly(300);
      //Machine_Speed_Set(125,125);			
			//Start_Motor_S(4,0,100); 
			//		  OSTimeDly(3000);	 			
			//		  SetPosition(4,300);
			//			OSTimeDly(3000);
			//      SetPosition(4,50);	    
			OSTaskSuspend(START_TASK_Prio);				
      //Electric_Push_Rod_Detected_Stop();			
			//printf("%d\r\n",MIC_Motor_Close_Detected());			
			//			printf("\r\n Car_Forward_Back_Detected:%d\r\n",Car_Forward_Back_Detected());
			//			printf("Open_Car_Lid_Detected:%d\r\n",Open_Car_Lid_Detected());
			//      
			//			printf("Close_Car_Lid_Detected:%d\r\n",Close_Car_Lid_Detected());
			//			printf("Detected_Car_Lid:%d\r\n",Detected_Car_Lid());
			//			
			//			printf("People_Seatsensor_Detected:%d\r\n",People_Seatsensor_Detected());
			//			printf("Detected_Carbreak_Detected:%d\r\n",Detected_Carbreak_Detected());
						//printf("Stopnow_Detected:%d\r\n",Stopnow_Detected());
			//			printf("Obstacle_From_Ultrasonic_Detected:%d\r\n",Obstacle_From_Ultrasonic_Detected());
			//			printf("Switch_Auto_Manual_Drivecar_Detected:%d\r\n",Switch_Auto_Manual_Drivecar_Detected());
						//printf("Get_AD48V_Adc:%d\r\n",Get_AD48V_Adc(ADC_Channel_8));
		  //printf("Get_battery_temperature_Adc:%d\r\n",Get_battery_temperature_Adc(ADC_Channel_14));		
			//printf("rain detected %d\r\n",GPIO_ReadInputDataBit(detected_rain_port,detected_rain_io_input));
		}
}

void Front_Motor_ResetTASK(void *pdata)//前轮复位任务
{
	pdata = pdata;
	Machine_Front_PWM_Count = 1600*Deceleration_Ratio; 
	TIM4->ARR = (SystemCoreClock/1000/800) - 1;	
	TIM4->CCR3 = ((SystemCoreClock/1000/800) - 1)>>1;
	
	TIM_Cmd(TIM4, ENABLE); 
	if(!GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_middle_ahead_motor_io_input))	
	{
		OSTimeDly(10);
		if(!GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_middle_ahead_motor_io_input))	
		{
			Machine_Front_PWM_Count = 0;
			TIM_Cmd(TIM4, DISABLE);
			Machine_Front_Motor_State =3;
			TrunEnableFlag = 1 ; 
      //OSTaskResume(START_TASK_Prio);					
		  OSTaskSuspend(Front_Motor_ResetTASK_Prio);	//suspend but not delete 
			//printf("reset3\r\n");
		}				  
	}	
	if(!GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_right_ahead_motor_io_input) && GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_left_ahead_motor_io_input))
	{
		OSTimeDly(100);
		if(!GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_right_ahead_motor_io_input) && GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_left_ahead_motor_io_input))
				Machine_Front_Motor_Directon_Control(Machine_Front_Motor_Turn_Right);
	}
	else if(GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_right_ahead_motor_io_input) && !GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_left_ahead_motor_io_input))
	{
		OSTimeDly(100);
		if(GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_right_ahead_motor_io_input) && !GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_left_ahead_motor_io_input))
				Machine_Front_Motor_Directon_Control(Machine_Front_Motor_Turn_Left);
	}
	
	TIM_Cmd(TIM4, ENABLE); 
	while(Machine_Front_PWM_Count)
  //while(1)
	{	
		if(!GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_right_ahead_motor_io_input) && GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_left_ahead_motor_io_input))
			{
				OSTimeDly(10);
				if(!GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_right_ahead_motor_io_input) && GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_left_ahead_motor_io_input))
						Machine_Front_Motor_Directon_Control(Machine_Front_Motor_Turn_Right);
			}		
		else if(GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_right_ahead_motor_io_input) && !GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_left_ahead_motor_io_input))
			{
				OSTimeDly(10);
				if(GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_right_ahead_motor_io_input) && !GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_left_ahead_motor_io_input))
						Machine_Front_Motor_Directon_Control(Machine_Front_Motor_Turn_Left);
			}			
		if(!GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_middle_ahead_motor_io_input))	
			{
				  OSTimeDly(10);
					if(!GPIO_ReadInputDataBit(detected_ahead_motor_port,detected_middle_ahead_motor_io_input))	
					{
							Machine_Front_PWM_Count = 0;
							TIM_Cmd(TIM4, DISABLE);
						  Machine_Front_Motor_State =3;
						  TrunEnableFlag = 1 ; 
						  //OSTaskResume(START_TASK_Prio);	
							OSTaskSuspend(Front_Motor_ResetTASK_Prio);	//suspend but not delete 							
					}				  
			}
      //OSSchedLock();//OSSchedUnlock();
			//printf("Front_Motor_ResetTASK\r\n");
			//OSSchedUnlock();
			//OSTimeDly(50);
	}
	//OSTaskResume(START_TASK_Prio);	
	OSTaskSuspend(Front_Motor_ResetTASK_Prio);	//suspend but not delete 	
}

enum {
	LeftUpState = 0x01,
	LeftDownState = 0x02,
	RightDownState = 0x04,
	RightUpState = 0x08
};

//enum {
//	norunning,
//	running
//};

__IO uint8_t CarLidState = 0;
__IO uint8_t CarLidDetectedState = 0;
__IO uint8_t BoxState = 0;
__IO uint8_t model = 0;

//CarLidDetectedState = Detected_Car_Lid(); //检测到的话是1，没检测到是零.
void BOX_TASK(void *pdata)//拉箱任务
{
	pdata = pdata;
	static uint8_t state                = 0;
	static uint8_t Car_Lid_Button_State = 0;
	
	while(1)
	{	
		//	OSSchedLock();//OSSchedUnlock();
		//  printf("BOX_TASK\r\n");
		//  OSSchedUnlock();
		OSTaskResume(START_TASK_Prio);		
    //拉箱和盖子有关
		//		if(BoxState == norunning)
		//		{
		//			if(CarLidState == (RightDownState|LeftDownState))
		//			{
		//				model = 1;
		//			}
		//		}
		//		else if(CarLidState == (RightUpState|LeftUpState))
		//		{
		//			model = 2;
		//		}
		//    else if(BoxState == running)
		//		{
		//			if(CarLidState == 0)
		//			{
		//				//down
		//				if((GPIO_ReadInputDataBit(detected_ballbag_port,detected_back_ballbag_motor_io_input)
		//				||GPIO_ReadInputDataBit(detected_ballbag_port,detected_limited_back_ballbag_motor_io_input))//？？？
		//				&& (Index != DOWNINDEX))
		//				{
		//					Middle_Motor_Goback();
		//				  Middle_Motor_NO_Break();
		//				  Middle_Motor_Run();
		//				}
		//			}
		//		}
		//拉箱和盖子有关

		 /*MIC电机*/
		 //printf("MIC Motor Close %d\r\n",MIC_Motor_Close_Detected());
     //MIC_Motor_Stop();
		/*MIC电机*/
    //MIC_Motor_Run();
		/*单独盖子*/
   	 if(!Open_Car_Lid_Detected())
		 {
				Electric_Push_Rod_Up();
			  //printf("open\r\n");		  
			  CarLidState = 0;
			  state = 0;			
		 }
		else if(!Close_Car_Lid_Detected())
		 {
				//printf("Close_Car_Lid_Detected\r\n");
				if((BoxState == 3) && (CarLidDetectedState == 0))
				{
					Electric_Push_Rod_Down();							
					//printf("close\r\n");
          //CarLidState = 0;					
				}			
	 	 }
//		else {
//			Electric_Push_Rod_Stop();
//			CarLidState = 0;
//		}					
		if(!GPIO_ReadInputDataBit(electric_push_rod_detected_port,off_electric_push_rod_right_io_input))
		 {
				//printf("RightDownState\r\n");//右下
			  CarLidState |= RightDownState;
		 }
		if(!GPIO_ReadInputDataBit(electric_push_rod_detected_port,on_electric_push_rod_right_io_input))
		{
				//printf("RightUpState\r\n");//右上
			  CarLidState |= RightUpState;	
		}
		if(!GPIO_ReadInputDataBit(electric_push_rod_detected_port,off_electric_push_rod_left_io_input))
		{		
				//printf("LeftDownState\r\n");//左下
			  CarLidState |= LeftDownState;		
		}
		if(!GPIO_ReadInputDataBit(electric_push_rod_detected_port,on_electric_push_rod_left_io_input))
		{
				//printf("LeftUpState\r\n"); //左上
			  CarLidState |= LeftUpState;	
		}
		CarLidDetectedState = Detected_Car_Lid();//检测到的话是1，没检测到是零.
		if(CarLidState == (RightUpState|LeftUpState))
		 { 
			 static uint32_t Current_Time = 0;	
       //printf("RightUpState|LeftUpState\r\n");			 
			 if(0 == state)
			 {
				 OSTimeSet(0);
				 state = 1;
				 //printf("RightUpState|LeftUpState state = 0\r\n");
			 }
			 else if(state == 1)
			 {
				  if(OSTimeGet()>2000)
					 {
						  MIC_Motor_Stop();
             // printf("RightUpState|LeftUpState state = 1 Motor Down\r\n");						 
					 }
					 else {						 
						  MIC_Motor_Run();	
						  //printf("RightUpState|LeftUpState state = 1 Motor up\r\n");
					 }
			 }			 
		 }
		 else if(CarLidState == (RightDownState|LeftDownState))
		 {			 
			 //printf("RightDownState|LeftDownState\r\n");
			 if(!MIC_Motor_Close_Detected())
			 {
				 MIC_Motor_Stop();				  
			 }
			 else {				 
				 MIC_Motor_Run();
			 }
		 }		 
		/*单独盖子*/
		//		if(!MIC_Motor_Close_Detected())
		//		 {
		//			 
		//		 }
		/*单独拉箱*/
    //if(model == 2)
    {			
			if((!GPIO_ReadInputDataBit(detected_ballbag_port,detected_limited_ahead_ballbag_motor_io_input) 
					|| !GPIO_ReadInputDataBit(detected_ballbag_port,detected_ahead_ballbag_motor_io_input))//detected_ahead_ballbag_motor_io_input
					&& (Index != UPINDEX))
				{		
					
						Middle_Motor_Stop();
						Middle_Motor_Break();
						Index = UPINDEX ; 
						BoxState = 1;
						//OSTaskSuspend(CarLidTASK_Prio);
						//printf("UPINDEX\r\n");					
				}
			 if(!GPIO_ReadInputDataBit(detected_ballbag_port,detected_center_ballbag_motor_io_input)&& (Index != MIDINDEX))
				{
						 Middle_Motor_Stop();
						 Middle_Motor_Break();
						 Index = MIDINDEX ; 
						 BoxState = 2;
						 //OSTaskSuspend(CarLidTASK_Prio);
						 //printf("MIDINDEX\r\n");					
				}			
			 if((!GPIO_ReadInputDataBit(detected_ballbag_port,detected_back_ballbag_motor_io_input)
					||!GPIO_ReadInputDataBit(detected_ballbag_port,detected_limited_back_ballbag_motor_io_input))
					&& (Index != DOWNINDEX))
				{
							Middle_Motor_Stop();
							Middle_Motor_Break();
							Index = DOWNINDEX ; 
							BoxState = 3;
							//OSTaskResume(CarLidTASK_Prio);	
							//printf("DOWNINDEX\r\n");				
				}
				else if(GPIO_ReadInputDataBit(detected_rain_port,detected_rain_io_input)) 
					{
             OSTimeDly(10);
						 if(GPIO_ReadInputDataBit(detected_rain_port,detected_rain_io_input))	
						 {							 
							  //printf("raining ballbag back!\r\n");
								//down
								if((!GPIO_ReadInputDataBit(detected_ballbag_port,detected_back_ballbag_motor_io_input)) 
									||!GPIO_ReadInputDataBit(detected_ballbag_port,detected_limited_back_ballbag_motor_io_input))
								{
									continue ; 
								}		
								 Middle_Motor_Goback();
								 Middle_Motor_NO_Break();
								 Middle_Motor_Run();
							}								
					}				
				if(!GPIO_ReadInputDataBit(button_ballbag_port,up_button_ballbag_motor_io))    
				{				
           OSTimeDly(10);
					 if(!GPIO_ReadInputDataBit(button_ballbag_port,up_button_ballbag_motor_io))	
					 {						 
							//up
							if((!GPIO_ReadInputDataBit(detected_ballbag_port,detected_ahead_ballbag_motor_io_input)) 
								|| !GPIO_ReadInputDataBit(detected_ballbag_port,detected_limited_ahead_ballbag_motor_io_input))
							{
								 continue ; 
							}
							
             else if(CarLidState == (RightUpState|LeftUpState))
             {
							 Middle_Motor_Goforward();//up
							 Middle_Motor_NO_Break();   
							 Middle_Motor_Run();
//						 BoxState = running;
						 }							 						 
						 //printf("up button!!\r\n");
					 }
				}		  
				else if(!GPIO_ReadInputDataBit(button_ballbag_port,down_button_ballbag_motor_io)) 
				{
					   OSTimeDly(10);
						 if(!GPIO_ReadInputDataBit(button_ballbag_port,down_button_ballbag_motor_io))
						 {
								//down
								if((!GPIO_ReadInputDataBit(detected_ballbag_port,detected_back_ballbag_motor_io_input))
									|| !GPIO_ReadInputDataBit(detected_ballbag_port,detected_limited_back_ballbag_motor_io_input))
								{
								 continue; 
								}						
								 //printf("down button!\r\n");
								 Middle_Motor_Goback();
								 Middle_Motor_NO_Break();
								 Middle_Motor_Run();
								//BoxState = running;	
						}							
				}		
				if(GPIO_ReadInputDataBit(detected_ballbag_port,detected_limited_ahead_ballbag_motor_io_input)
					&&GPIO_ReadInputDataBit(detected_ballbag_port,detected_ahead_ballbag_motor_io_input)
					&&GPIO_ReadInputDataBit(detected_ballbag_port,detected_center_ballbag_motor_io_input)
					&&GPIO_ReadInputDataBit(detected_ballbag_port,detected_back_ballbag_motor_io_input)
					&&GPIO_ReadInputDataBit(detected_ballbag_port,detected_limited_back_ballbag_motor_io_input))
				{
					Index = NONEINDEX ;
				  //BoxState = running;
					//OSTaskResume(CarLidTASK_Prio);	
				}
		}
    /*单独拉箱*/			
    //OSTimeDly(10);
		//printf("boxtask\r\n");      		 
		//OSTaskSuspend(BoxTASK_Prio);      				
	}
}

extern __IO uint32_t Speed_Left_Count;
extern __IO uint32_t Speed_Right_Count;
OS_EVENT * Sem_Event;

void Send_Task(void *pdata)//串口发送数据给上位机的任务
{
	pdata = pdata;
	u8 err;
	char sendbuf[20];
	uint32_t time=0;
	uint32_t speed=0;
	OS_SEM_DATA result[1];
	OS_ENTER_CRITICAL();
  Sem_Event = OSSemCreate(0);	
	OS_EXIT_CRITICAL();
	OSSemQuery(Sem_Event,result);//查询信号量Sem_Event的状态
	printf("Sem_Event count:%d\r\n",result->OSCnt);
	while(1)
	{
		Speed_Left_Count = 0;
		Speed_Right_Count = 0;
	  //OSTimeSet(0);//设置软件定时器为0
		OSSemPend(Sem_Event,0,&err);
		///time = OSTimeGet();
		//speed = ((Speed_Right_Count+Speed_Left_Count)>>1)*1000/(time+1);
		//sprintf(sendbuf,"s:%d\r\n",speed);
		sprintf(sendbuf,"bt:%d\r\n",After_filter[0]);
		OSSchedLock();
		SendDatatoMaster((uint8_t *)sendbuf);				
		//if(Car_Change_Direction_Old != Car_Change_Direction)
		{
			  if(!Car_Change_Direction)
				{
					strcpy(sendbuf,"B\r\n");
				}
				else if(Car_Change_Direction)
				{
					strcpy(sendbuf,"F\r\n");		
				}	
				SendDatatoMaster((uint8_t *)sendbuf);	
        //printf("back or forwarld\r\n");				
		}   
		OSSchedUnlock();
    Car_Change_Direction_Old = Car_Change_Direction;				
		//printf("speed_count:%d\r\n",speed);
		//printf("time:%d\r\n",time);
		//printf("bt:%d\r\n",After_filter[0]);
	}
}

//急停任务 硬件接线：IO口串一个二极管 
void EmergencyStop_Task(void *pdata)
{
    static __IO uint8_t state = 0;
	  while(1)
		{		
			state = Stopnow_Detected();
			OSTimeDly(20);
			state = Stopnow_Detected();			
			if(1 == state)
			{
				    OSTaskSuspend (BoxTASK_Prio);
						OSTaskSuspend (START_TASK_Prio);
						MIC_Motor_Stop();
						Middle_Motor_Stop();
						Electric_Push_Rod_Stop();
						Machine_Front_Motor_Enable_or_Disable(DISABLE);
						Machine_Break();		
            Machine_Disable();						
						OSSchedLock();
					  printf("EmergencyStop!!!\r\n");			
						OSSchedUnlock();
			}
			else if(0 == state)
			{
				    
						//Middle_Motor_Stop();
						//Electric_Push_Rod_Stop();
						Machine_Front_Motor_Enable_or_Disable(ENABLE);
						Machine_Nobreak();		
            Machine_Enable();					
						OSTaskResume(START_TASK_Prio);
						OSTaskResume(BoxTASK_Prio);		
			}
      OSTimeDly(20);	
      //printf("emergency\r\n");			
		}
}

//将上位机需要的数据发送到上位机。
void SendDatatoMaster(uint8_t sendbuf[])
{
	while(*sendbuf != '\0')
	{
		USART_SendData(USART1,(uint8_t)*sendbuf);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
			;
		sendbuf++;
	} 
}

/**
* @brief  Inserts a delay time.
* @param  nTime: specifies the delay time length, in 1 ms.
* @retval None
*/
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;
  while(TimingDelay != 0);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(EVAL_COM2, (uint8_t) ch);
  /* Loop until the end of transmission USART_FLAG_TXE USART_FLAG_TC*/
  while (USART_GetFlagStatus(EVAL_COM2, USART_FLAG_TXE) == RESET)
  {} 
  return ch;
}

#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
		
  }
}
#endif


