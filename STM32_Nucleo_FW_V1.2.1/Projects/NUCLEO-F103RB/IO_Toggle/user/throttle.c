#include "throttle.h"
#include "stdio.h"
#include "includes.h"
#include "port.h"
#include "stm32f10x.h"
#include "main.h"

/*
After_filter[0]---PB0---IN8---48v�ĵ�ѹ���
After_filter[1]---PB1---IN9---���ŵĵ�ѹ���  
After_filter[2]---PC0---IN10---������
After_filter[3]---PC1---IN11---���Ƹ˵ĵ���
After_filter[4]---PC2---IN12---���Ƹ˵ĵ���
After_filter[5]---PC3---IN13---��������ɲ������
After_filter[6]---PC4---IN14---�¶ȼ��ĵ�ѹ
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
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC|RCC_APB2Periph_ADC1	, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //ʹ��DMA����
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);   //64M/8=8,ADC���ʱ�䲻�ܳ���14M
	                       
	GPIO_InitStructure.GPIO_Pin = AD_throttle;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//����
	GPIO_Init(throttle_port, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = AD48V;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//48V��ѹ���
	GPIO_Init(AD48V_port, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = battery_temperature_detected;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//�¶ȼ��
	GPIO_Init(battery_temperature_detected_port, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = current_electric_push_rod_left_ad;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//�Ƹ���
	GPIO_Init(current_electric_push_rod_left_ad_port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = current_electric_push_rod_right_ad;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//�Ƹ���
	GPIO_Init(current_electric_push_rod_right_ad_port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = current_running_ballbag_motor_ad;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//�������ʱ�����
	GPIO_Init(current_ballbag_motor_ad_port, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = current_break_ballbag_motor_ad;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//���ɲ��ʱ�����
	GPIO_Init(current_ballbag_motor_ad_port, &GPIO_InitStructure);
	
	ADC_DeInit(ADC1); //������ADC1��ȫ���Ĵ�������Ϊȱʡֵ

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//������ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//�����ڳ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = M;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   
 
  //����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ��
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_239Cycles5 );
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_239Cycles5 );
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 4, ADC_SampleTime_239Cycles5 );
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 5, ADC_SampleTime_239Cycles5 );
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 6, ADC_SampleTime_239Cycles5 );
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 7, ADC_SampleTime_239Cycles5 );

	
	DMA_DeInit(DMA1_Channel1);//��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;//DMA����ADC����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&AD_Value;//DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//����ΪԴ��ַ
	DMA_InitStructure.DMA_BufferSize = N*M;//DMAͨ����DMA�����С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//���ݿ��Ϊ����
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//���ݿ��Ϊ����
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//������ѭ������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//ӵ�иߵ����ȼ�
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//�ڴ浽�ڴ洫�䲻ʹ��
  DMA_Init(DMA1_Channel1,&DMA_InitStructure);//DMA������ʼ��.
	
	//����ADC��DMA֧��(Ҫʵ��DMA���ܣ���Ҫ����DMAͨ���Ȳ���)
  ADC_DMACmd(ADC1,ENABLE);
	
	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	
	ADC_ResetCalibration(ADC1);	//����ָ����ADC1��У׼�Ĵ���
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//��ȡADC1����У׼�Ĵ�����״̬,����״̬��ȴ�
	
	ADC_StartCalibration(ADC1);		//��ʼָ��ADC1��У׼״̬
 
	while(ADC_GetCalibrationStatus(ADC1));		//��ȡָ��ADC1��У׼����,����״̬��ȴ�
 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������
	DMA_Cmd(DMA1_Channel1,ENABLE);
}

void Get_Machine_Adc(void)
{
	DMA_Cmd(DMA1_Channel1,ENABLE);
	//ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������

	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//�ȴ�ת������
}

//��ʼ��ADC
//�������ǽ��Թ���ͨ��Ϊ��
//����Ĭ�Ͻ�����ͨ��6																   
void  Throttle_Adc_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_ADC1	, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
 
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);   //64M/8=8,ADC���ʱ�䲻�ܳ���14M
	//PA0/1/2/3 ��Ϊģ��ͨ����������                         
	GPIO_InitStructure.GPIO_Pin = AD_throttle;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(throttle_port, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   
 
	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	
	ADC_ResetCalibration(ADC1);	//����ָ����ADC1��У׼�Ĵ���
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//��ȡADC1����У׼�Ĵ�����״̬,����״̬��ȴ�
	
	ADC_StartCalibration(ADC1);		//��ʼָ��ADC1��У׼״̬
 
	while(ADC_GetCalibrationStatus(ADC1));		//��ȡָ��ADC1��У׼����,����״̬��ȴ�
 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������
	
}

//���ADCֵ
//ch:ͨ��ֵ 0~3
u16 Get_Throttle_Adc(u8 ch)   
{
  //����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCͨ��3,�������˳��ֵΪ1,����ʱ��Ϊ239.5����	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}	

//PB11 PA6 �ٶȱ������������ʼ��
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


__IO uint32_t Speed_Left_Count  = 0;//��ߵ����ӵ�����ֵ
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
			OSSemPost(Sem_Event); //�����ź���
		}
	  //printf("Speed_Left_Count:%d\r\n",Speed_Left_Count);
    EXTI_ClearITPendingBit(EXTI_Line11);
  }
	OSIntExit();	
}



