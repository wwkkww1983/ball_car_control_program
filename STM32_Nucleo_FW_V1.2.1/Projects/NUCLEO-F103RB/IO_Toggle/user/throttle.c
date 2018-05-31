#include "throttle.h"
#include "stdio.h"
#include "includes.h"
#include "port.h"
#include "stm32f10x.h"
#include "main.h"

/*
After_filter[0]---PB0---IN8---48v的电压检测
After_filter[1]---PB1---IN9---油门的电压检测  
After_filter[2]---PC0---IN10---球包电机
After_filter[3]---PC1---IN11---左推杆的电流
After_filter[4]---PC2---IN12---右推杆的电流
After_filter[5]---PC3---IN13---球包电机的刹车电流
After_filter[6]---PC4---IN14---温度检测的电压
*/

uint16_t AD_Value[N][M];
uint16_t After_filter[M];
int i;
void filter(void)
{
	int sum = 0;
	uint8_t count;
	for(i=0;i<M;i++)
	{
	  for(count = 0;count<N;count++)
     {
			 sum += AD_Value[count][i];
		 }		
		 After_filter[i] = sum/N;
	   sum = 0;
	}
	//After_filter[1] = AD_Value[0][1];
}

void Machine_Adc_Init(void)
{
  ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;	
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC|RCC_APB2Periph_ADC1	, ENABLE );	  //使能ADC1通道时钟
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //使能DMA传输
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);   //64M/8=8,ADC最大时间不能超过14M
	                       
	GPIO_InitStructure.GPIO_Pin = AD_throttle;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//油门
	GPIO_Init(throttle_port, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = AD48V;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//48V电压检测
	GPIO_Init(AD48V_port, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = battery_temperature_detected;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//温度检测
	GPIO_Init(battery_temperature_detected_port, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = current_electric_push_rod_left_ad;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//推杆左
	GPIO_Init(current_electric_push_rod_left_ad_port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = current_electric_push_rod_right_ad;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//推杆右
	GPIO_Init(current_electric_push_rod_right_ad_port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = current_running_ballbag_motor_ad;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//球包运行时候电流
	GPIO_Init(current_ballbag_motor_ad_port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = current_break_ballbag_motor_ad;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//球包刹车时候电流
	GPIO_Init(current_ballbag_motor_ad_port, &GPIO_InitStructure);
	
	ADC_DeInit(ADC1); //将外设ADC1的全部寄存器重设为缺省值

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//工作在扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//工作在持续转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = M;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   
 
  //设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_239Cycles5 );
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_239Cycles5 );
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 4, ADC_SampleTime_239Cycles5 );
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 5, ADC_SampleTime_239Cycles5 );
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 6, ADC_SampleTime_239Cycles5 );
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 7, ADC_SampleTime_239Cycles5 );

	
	DMA_DeInit(DMA1_Channel1);//将DMA的通道1寄存器重设为缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;//DMA外设ADC基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&AD_Value;//DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//外设为源地址
	DMA_InitStructure.DMA_BufferSize = N*M;//DMA通道的DMA缓冲大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不增加
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//数据宽度为半字
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//数据宽度为半字
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//工作在循环缓冲模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//拥有高的优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//内存到内存传输不使能
  DMA_Init(DMA1_Channel1,&DMA_InitStructure);//DMA参数初始化.
	
	//开启ADC的DMA支持(要实现DMA功能，还要配置DMA通道等参数)
  ADC_DMACmd(ADC1,ENABLE);
	
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	
	ADC_ResetCalibration(ADC1);	//重置指定的ADC1的校准寄存器
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//获取ADC1重置校准寄存器的状态,设置状态则等待
	
	ADC_StartCalibration(ADC1);		//开始指定ADC1的校准状态
 
	while(ADC_GetCalibrationStatus(ADC1));		//获取指定ADC1的校准程序,设置状态则等待
 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能
	DMA_Cmd(DMA1_Channel1,ENABLE);
}

void Get_Machine_Adc(void)
{
	DMA_Cmd(DMA1_Channel1,ENABLE);
	//ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能

	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//等待转换结束
}

//初始化ADC
//这里我们仅以规则通道为例
//我们默认将开启通道6																   
void  Throttle_Adc_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_ADC1	, ENABLE );	  //使能ADC1通道时钟
 
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);   //64M/8=8,ADC最大时间不能超过14M
	//PA0/1/2/3 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = AD_throttle;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(throttle_port, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //将外设 ADC1 的全部寄存器重设为缺省值

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   
 
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	
	ADC_ResetCalibration(ADC1);	//重置指定的ADC1的校准寄存器
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//获取ADC1重置校准寄存器的状态,设置状态则等待
	
	ADC_StartCalibration(ADC1);		//开始指定ADC1的校准状态
 
	while(ADC_GetCalibrationStatus(ADC1));		//获取指定ADC1的校准程序,设置状态则等待
 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能
	
}

//获得ADC值
//ch:通道值 0~3
u16 Get_Throttle_Adc(u8 ch)   
{
  //设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道3,规则采样顺序值为1,采样时间为239.5周期	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}	

//PB11 PA6 速度编码器的输入初始化
void Machine_GetSpeed_Init(void)
{
	EXTI_InitTypeDef  EXTI_InitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
	
  /* Enable GPIOB GPIOA clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
  
	//PA6
  /* Configure PA11 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI6 Line to PA6 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6);
  
  /* Configure EXTI6 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line6;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 11;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	
  /* Configure PB11 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Enable AFIO clock */
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI11 Line to PB11 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource11);

  /* Configure EXTI11line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line11;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI15_10_IRQn Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 12;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
}


__IO uint32_t Speed_Left_Count  = 0;//左边的轮子的脉冲值
void EXTI15_10_IRQHandler(void)
{
	OSIntEnter();		//OSIntExit();
	if(EXTI_GetITStatus(EXTI_Line11) != RESET)
  { 
		//if(!GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11))
		{
		   Speed_Left_Count++;
		}  
		if(Speed_Left_Count == MAX_SPEED_COUNT)
		{
			OSSemPost(Sem_Event); //发送信号量
		}
	  //printf("Speed_Left_Count:%d\r\n",Speed_Left_Count);
    EXTI_ClearITPendingBit(EXTI_Line11);
  }
	OSIntExit();	
}



