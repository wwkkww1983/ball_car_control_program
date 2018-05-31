//ͷ�ļ�
#include "motorSalgorithm.h" 
#include "main.h"

#define F2TIME_PARA				64000//12000000   									//��Ƶ��ֵת��Ϊ��ʱ���Ĵ���ֵ��ת������
#define STEP_PARA					10										//����ʱ��ת��������������
#define STEP_AA						31//31       										//�Ӽ��ٽ׶Σ���ɢ������
#define STEP_UA						31//31			  									//�ȼ��ٽ׶Σ���ɢ������
#define STEP_RA						31//31													//�����ٽ׶Σ���ɢ������

#define STEP_SPTA					20													//SPTA����ٶȵȼ�
#define MAXSPEED_SPTA			80000												//SPTA����ٶ�
#define ACCSPEED_SPTA			150000											//SPTA���ٶ�


uint16_t Motor4TimeTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor4StepTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};	


uint16_t Motor4_23TimeTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor4_23StepTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};	

/*��������Ϊ1/3 S�����߲������ɵı��*/

uint16_t Motor4_13TimeTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor4_13StepTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};		

unsigned long long Get_Time_Cost(unsigned char MotorID);

/*
���1��PA8(pwm),PE9(CW),PE8(ENABLE),MXX:PA11,PA12,PE7
���2��PA0(pwm),PA1(CW),PC3(ENABLE),MXX:PC0,PC1,PC2
���3��PA6(pwm),PA7(CW),PC4(ENABLE),MXX:PA3,PA4,PA5
���4��PB6(pwm,gpio),PB9(CW),PB8(ENABLE),MXX:PD7,PB5,PB7
*/
void Initial_MotorIO(void)
{
   
}



 /**************************************************************************************
 ��ʼ������Ĳ�������Ҫ��ϸ��ѡ��ʹ�õĶ�ʱ����˳ʱ�뷽��ֵ�����ID��
 **************************************************************************************/
void Initial_Motor(unsigned char MotorID, unsigned char StepDive,unsigned int maxposition)
{
  unsigned int i=0;
	MOTOR_CONTROL_S *pmotor=NULL; 	
	uint16_t *MotorTimeTable;
	uint16_t *MotorStepTable;
  switch(MotorID)
  {
	  case 1:			 
//			pmotor=&motor1;
//			motor1.id=1;
//			motor1.clockwise=M1_CLOCKWISE;
//			motor1.TIMx=TIM1;
//			MotorTimeTable=Motor1TimeTable;
//			MotorStepTable=Motor1StepTable;			
			break;
		case 2:			 
//			pmotor=&motor2;
//			motor2.id=2;
//			motor2.clockwise=M2_CLOCKWISE;
//			motor2.TIMx=TIM2;
//			MotorTimeTable=Motor2TimeTable;
//			MotorStepTable=Motor2StepTable;
			 break;
		case 3:
//			pmotor=&motor3;
//			motor3.id=3;
//			motor3.clockwise=M3_CLOCKWISE;
//			motor3.TIMx=TIM3;
//			MotorTimeTable=Motor3TimeTable;
//			MotorStepTable=Motor3StepTable;
			break;
		case 4:			
			pmotor = &motor4;
			motor4.id=4;
			motor4.clockwise=M4_CLOCKWISE;
			motor4.TIMx=TIM4;
			MotorTimeTable=Motor4TimeTable;
			MotorStepTable=Motor4StepTable;
		  break;		
	  default:break;
  }
	if(MotorID<=4&&MotorID>=1)
	{
		pmotor->divnum=StepDive;
		pmotor->MaxPosition=maxposition;
		pmotor->MaxPosition_Pulse=maxposition*StepDive;

		pmotor->CurrentPosition = 40;//��ǰ���е�
		pmotor->CurrentPosition_Pulse = 40*StepDive;//��ǰ��������
		pmotor->StartTableLength=STEP_AA+STEP_UA+STEP_RA+1;
		pmotor->StopTableLength=STEP_AA+STEP_UA+STEP_RA; 
		pmotor->Counter_Table=MotorTimeTable;
		pmotor->Step_Table=MotorStepTable;

		pmotor->CurrentIndex=0;
		pmotor->speedenbale=0;
		pmotor->StartSteps=0;                  //�������㣬�������ۼӣ�������ǰһ�εļ���
		pmotor->StopSteps=0;                   //ͬ��
		for(i=0;i<pmotor->StartTableLength;i++)
		 pmotor->StartSteps+=pmotor->Step_Table[i];
		for(i=0;i<pmotor->StopTableLength;i++)
		 pmotor->StopSteps+=pmotor->Step_Table[i+pmotor->StartTableLength]; 
		pmotor->TIMx->ARR =pmotor->Counter_Table[0]; //��������
		pmotor->TIMx->CCR3 =pmotor->Counter_Table[0]>>1;//����ռ�ձ� ͨ��3   		
	}
}

/*����Эͬʹ�����㷨ԭ�����ʱ��Ԥ���������޸ĸ��㷨ʱ�ǵ�
 ����������ͬ��*/
/*����S�����߷�ת�㣬S������������ʱ���Ӽ��ٹ�������ȫ�ԳƵ�*/
unsigned long long Get_TimeCost_ReverDot_S(unsigned char MotorID)
{
	unsigned long long time_cost=0;
	unsigned long long time_cost2=0;
	unsigned int pulsecnt=0;
	int i=0,j;
	MOTOR_CONTROL_S *pmotor=NULL; 
	switch(MotorID)
	{
		case 1:
		//	pmotor=&motor1;  
			break;
		case 2:
			//pmotor=&motor2;  
			break;
		case 3:
			//pmotor=&motor3;  
		case 4:
			pmotor=&motor4;  
			break;
		default:
			return 0;
	}
	
	if(pmotor->PulsesGiven>=pmotor->StartSteps+pmotor->StopSteps)
	{
		for(i=0;i<pmotor->StartTableLength;i++)
			time_cost+=(pmotor->Step_Table[i]*pmotor->Counter_Table[i]);
		for(i=0;i<pmotor->StopTableLength;i++)
			time_cost+=(pmotor->Step_Table[i+pmotor->StartTableLength]*pmotor->Counter_Table[i+pmotor->StartTableLength]);		
		time_cost+=(pmotor->PulsesGiven-pmotor->StartSteps-pmotor->StopSteps)*pmotor->Counter_Table[pmotor->StartTableLength-1];
		
		pmotor->RevetDot=pmotor->PulsesGiven-pmotor->StopSteps;
	}
	else
	{
		//���������������һƵ��142 �����ڶ�Ƶ��148����Ҫ���˶�200������ô����
		//��������Ҫ�ı�ڶ�Ƶ�ʵĲ���
		while((pulsecnt+pmotor->Step_Table[i])<=(pmotor->PulsesGiven>>1))
		{					
			time_cost+=(pmotor->Step_Table[i]*pmotor->Counter_Table[i]);
			time_cost2+=(pmotor->Step_Table[i]*pmotor->Counter_Table[i]);
			pulsecnt+=pmotor->Step_Table[i];
			i++;
		}
		time_cost+=time_cost2;
		if(pmotor->Step_Table[i]<pmotor->PulsesGiven-2*pulsecnt)
		{
			pmotor->Step_Table[i]=pmotor->PulsesGiven-2*pulsecnt;
			pmotor->StartSteps=0;                  //�������㣬�������ۼӣ�������ǰһ�εļ���
			pmotor->StopSteps=0;                   //ͬ��
			for(j=0;j<pmotor->StartTableLength;j++)
			 pmotor->StartSteps+=pmotor->Step_Table[j];
			for(j=0;j<pmotor->StopTableLength;j++)
			 pmotor->StopSteps+=pmotor->Step_Table[j+pmotor->StartTableLength];
		}
		time_cost+=(pmotor->Counter_Table[i]*(pmotor->PulsesGiven-2*pulsecnt));
		pmotor->RevetDot=pmotor->PulsesGiven-pulsecnt;
	}
	pmotor->Time_Cost_Cal=time_cost;
	return time_cost;
}
unsigned long long Get_Time_Cost2(unsigned char MotorID)
{
	extern void TIM1_UP_IRQHandler(void);
	extern void TIM2_IRQHandler(void);
	extern void TIM3_IRQHandler(void);
	switch(MotorID)
	{
		case 1:
			while(motor1.running==1)
			{
				TIM1_UP_IRQHandler();
			}
			return motor1.Time_Cost_Act; 
		case 2:
			while(motor2.running==1)
			{
				TIM2_IRQHandler();
			}
			return motor2.Time_Cost_Act; 
		case 3:
			while(motor3.running==1)
			{
				TIM3_IRQHandler();
			}
			return motor3.Time_Cost_Act; 
	}
	return 0;
}


/*���³�ʼ���������ʱ��ز���*/
void Motor_Reinitial(unsigned char MotorID)
{
	int i=0; 
	MOTOR_CONTROL_S *pmotor=NULL;  
	uint16_t *MotorTimeTable;
	uint16_t *MotorStepTable;
	uint16_t *MotorTime23Table;
	uint16_t *MotorStep23Table;
	uint16_t *MotorTime13Table;
	uint16_t *MotorStep13Table;
	
	switch(MotorID)
	{
		case 1:
//			pmotor=&motor1;  
//		  MotorTimeTable=Motor1TimeTable;
//			MotorStepTable=Motor1StepTable;
//			MotorTime23Table=Motor1_23TimeTable;
//			MotorStep23Table=Motor1_23StepTable;
//			MotorTime13Table=Motor1_13TimeTable;
//			MotorStep13Table=Motor1_13StepTable;
			break;
		case 2:
//			pmotor=&motor2;  
//		  MotorTimeTable=Motor2TimeTable;
//			MotorStepTable=Motor2StepTable;
//			MotorTime23Table=Motor2_23TimeTable;
//			MotorStep23Table=Motor2_23StepTable;
//			MotorTime13Table=Motor2_13TimeTable;
//			MotorStep13Table=Motor2_13StepTable;
			break;
		case 3:
//			pmotor=&motor3; 
//		  MotorTimeTable=Motor3TimeTable;
//			MotorStepTable=Motor3StepTable;
//			MotorTime23Table=Motor3_23TimeTable;
//			MotorStep23Table=Motor3_23StepTable;
//			MotorTime13Table=Motor3_13TimeTable;
//			MotorStep13Table=Motor3_13StepTable;
			break;
		case 4:
			pmotor=&motor4; 
		  MotorTimeTable=Motor4TimeTable;
			MotorStepTable=Motor4StepTable;
			MotorTime23Table=Motor4_23TimeTable;
			MotorStep23Table=Motor4_23StepTable;
			MotorTime13Table=Motor4_13TimeTable;
			MotorStep13Table=Motor4_13StepTable;
			break;
		default:
			return ;
	}					 
	pmotor->pulsecount=0;
	pmotor->CurrentIndex=0;
	pmotor->speedenbale=0;
	
	pmotor->Counter_Table=MotorTimeTable;  		//ָ������ʱ��ʱ�����������
  pmotor->Step_Table=MotorStepTable;  			//ָ������ʱ��ÿ��Ƶ�����������
	pmotor->StartSteps=0;                  //�������㣬�������ۼӣ�������ǰһ�εļ���
	pmotor->StopSteps=0;                   //ͬ��
	for(i=0;i<pmotor->StartTableLength;i++)
	 pmotor->StartSteps+=pmotor->Step_Table[i];
	for(i=0;i<pmotor->StopTableLength;i++)
	 pmotor->StopSteps+=pmotor->Step_Table[i+pmotor->StartTableLength];
	if(pmotor->PulsesGiven<pmotor->StartSteps+pmotor->StopSteps){
		//������������в���С�����S�����ߣ�����ѡ��2/3S������
		pmotor->Counter_Table=MotorTime23Table;  		
		pmotor->Step_Table=MotorStep23Table;  			
		pmotor->StartSteps=0;                  
		pmotor->StopSteps=0;                   
		for(i=0;i<pmotor->StartTableLength;i++)
		 pmotor->StartSteps+=pmotor->Step_Table[i];
		for(i=0;i<pmotor->StopTableLength;i++)
		 pmotor->StopSteps+=pmotor->Step_Table[i+pmotor->StartTableLength];
		if(pmotor->PulsesGiven<pmotor->StartSteps+pmotor->StopSteps){
			//������������в���С�����S�����ߣ�����ѡ��1/3S������
			pmotor->Counter_Table=MotorTime13Table;  		
			pmotor->Step_Table=MotorStep13Table;  			
			pmotor->StartSteps=0;                  
			pmotor->StopSteps=0;                   
			for(i=0;i<pmotor->StartTableLength;i++)
			 pmotor->StartSteps+=pmotor->Step_Table[i];
			for(i=0;i<pmotor->StopTableLength;i++)
			 pmotor->StopSteps+=pmotor->Step_Table[i+pmotor->StartTableLength];
		}
	}
	    	
	pmotor->TIMx->ARR =pmotor->Counter_Table[0]; //��������
	pmotor->TIMx->CCR1 =pmotor->Counter_Table[0]>>1;       //����ռ�ձ�
	pmotor->Time_Cost_Act=pmotor->TIMx->ARR;
	Get_TimeCost_ReverDot_S(MotorID);		 
		
}
 
/*����S�����߲�����ȡĳ��ʱ�̵�Ƶ��*/
float GetFreAtTime(float fstart,float faa,float taa,float tua,float tra,float t)
{
		//���ݹ�ʽ����ӿ�ʼ������ٹ����У�tʱ�̵�ת��Ƶ��
	  if(t>=0&&t<=taa){
			//�Ӽ��ٽ׶�
			return fstart+0.5*faa*t*t;
		}else if(taa<t&&t<=(taa+tua)){
			//�ȼ��ٽ׶�
			return fstart+0.5*faa*taa*taa+(t-taa)*faa*taa;
		}else if((taa+tua)<t&&t<=(taa+tua+tra)){
			//�����ٽ׶�
			return fstart+0.5*faa*taa*taa+(tua)*faa*taa+0.5*faa*taa*tra-0.5*faa*taa*(taa+tua+tra-t)*(taa+tua+tra-t)/(tra);
		}		
		return 0;
}
 
 /*����S�������㷨��ÿһ����ʱ�����ڼ�������*/
void CalcMotorPeriStep_CPF(float fstart,float faa,float taa,float tua,float tra,uint16_t MotorTimeTable[],uint16_t MotorStepTable[])
{
  int  i;
	float fi;
	
	for(i=0;i<STEP_AA;i++)
	{
		fi=GetFreAtTime(fstart,faa,taa,tua,tra,taa/STEP_AA*i);
		MotorTimeTable[i]=F2TIME_PARA/fi;
		MotorStepTable[i]=fi*(taa/STEP_AA)/STEP_PARA;
	}
	for(i=STEP_AA;i<STEP_AA+STEP_UA;i++)
	{
		fi=GetFreAtTime(fstart,faa,taa,tua,tra,taa+(tua/STEP_UA)*(i-STEP_AA));
		MotorTimeTable[i]=F2TIME_PARA/fi;
		MotorStepTable[i]=fi*(tua/STEP_UA)/STEP_PARA;
	}
	for(i=STEP_AA+STEP_UA;i<STEP_AA+STEP_UA+STEP_RA;i++)
	{
		fi=GetFreAtTime(fstart,faa,taa,tua,tra,taa+tua+tra/STEP_RA*(i-STEP_AA-STEP_UA));
		MotorTimeTable[i]=F2TIME_PARA/fi;
		MotorStepTable[i]=fi*(tra/STEP_RA)/STEP_PARA;
	}
	fi=GetFreAtTime(fstart,faa,taa,tua,tra,taa+tua+tra);
	MotorTimeTable[STEP_AA+STEP_UA+STEP_RA]=F2TIME_PARA/fi;
	MotorStepTable[STEP_AA+STEP_UA+STEP_RA]=fi*(tra/STEP_RA)/STEP_PARA;
	
	for(i=STEP_AA+STEP_UA+STEP_RA+1;i<2*(STEP_AA+STEP_UA+STEP_RA)+1;i++)
	{ 
		MotorTimeTable[i]=MotorTimeTable[2*(STEP_AA+STEP_UA+STEP_RA)-i];
		MotorStepTable[i]=MotorStepTable[2*(STEP_AA+STEP_UA+STEP_RA)-i];
	}
}
 
/**************************************************************************************
������в�����ʼ��*/
void MotorRunParaInitial(void)
{ 
	/*FIXME:�û����Ըı�ò���ʵ��S�����ߵ���������*/ 
	CalcMotorPeriStep_CPF(M_FRE_START,M_FRE_AA,M_T_AA,M_T_UA,M_T_RA,Motor4TimeTable,Motor4StepTable);

	/*���Ĳ�����Ϊ2/3���ɵı��*/
	CalcMotorPeriStep_CPF(M_FRE_START*2.0/3,M_FRE_AA*2.0/3,M_T_AA*2.0/3,M_T_UA*2.0/3,M_T_RA*2.0/3,Motor4_23TimeTable,Motor4_23StepTable); 


	/*���Ĳ�����Ϊ1/3���ɵı��*/
  CalcMotorPeriStep_CPF(M_FRE_START*1.0/3,M_FRE_AA*1.0/3,M_T_AA*1.0/3,M_T_UA*1.0/3,M_T_RA*1.0/3,Motor4_13TimeTable,Motor4_13StepTable); 

}

 
/**************************************************************************************
�������ͬʱ����ʱ������ʱ����ٵĵ��Ҫ����ʱ�䳤�ĵ���������в���
*/
void Find_BestTimeCost(unsigned char ID,unsigned long long time_cost,unsigned char dir,unsigned int Degree)
{
	
}


/**************************************************************************************
�����������S�����߲�������*/
void Start_Motor_S(unsigned char MotorID,unsigned char dir,unsigned int Degree)
{
  unsigned int PulsesGiven=0;
	MOTOR_CONTROL_S *pmotor=NULL; 
	if(Degree==0)
	{ 		  	 
		return;
	}
	switch(MotorID)
	{
		case 1:
			//pmotor=&motor1; 
			if(0==dir)
		  {
		    //GPIO_SetBits(GPIOE,GPIO_Pin_9);
				
		  }
		  else
		  {
		    //GPIO_ResetBits(GPIOE,GPIO_Pin_9);
				
		  } 			
			break;
		case 2:
		  //pmotor=&motor2; 
		  if(1==dir)
		  {
		    //GPIO_SetBits(GPIOA,GPIO_Pin_1);
		  }
		  else
		  {
		    //GPIO_ResetBits(GPIOA,GPIO_Pin_1);  
		  }	
			break;
		case 3:
//		  //pmotor=&motor3; 
//		  if(0==dir)
//		  {
//		    //GPIO_SetBits(GPIOA,GPIO_Pin_7);
//				GPIO_SetBits(GPIOB,Machine_Front_Motor_Direction_Pin);
//		  }
//		  else
//		  {
//		    //GPIO_ResetBits(GPIOA,GPIO_Pin_7);
//				GPIO_ResetBits(GPIOB,Machine_Front_Motor_Direction_Pin);
//		  }	
			break;
	  case 4:
			//Delay(2);
//			if(0==dir)
//		  {
//		    //GPIO_SetBits(GPIOA,GPIO_Pin_7);
//				//GPIO_SetBits(GPIOB,Machine_Front_Motor_Direction_Pin);
//				Machine_Front_Motor_Directon_Control(dir);
//		  }
//		  else
//		  {
//		    //GPIO_ResetBits(GPIOA,GPIO_Pin_7);
//				GPIO_ResetBits(GPIOB,Machine_Front_Motor_Direction_Pin);
//		  }	
			Machine_Front_Motor_Directon_Control(dir);
			pmotor=&motor4;
			break;
		default:
			return;
	}
	pmotor->en=1;
	pmotor->dir=dir;
	pmotor->running=1;
	
	pmotor->PulsesHaven=0;
	PulsesGiven=Degree;
	pmotor->Time_Cost_Act=0;
	pmotor->PulsesGiven	=PulsesGiven*pmotor->divnum;
	
	Motor_Reinitial(MotorID);		
	pmotor->CurrentIndex=0;
	pmotor->speedenbale=0;
	pmotor->TIMx->ARR =pmotor->Counter_Table[0]; //��������
	pmotor->TIMx->CCR3 =pmotor->Counter_Table[0]>>1;       //����ռ�ձ�
  TIM_Cmd(pmotor->TIMx, ENABLE);		  //DISABLE
}

/*�����������SPTA��ʽ����*/
void Start_Motor_SPTA(unsigned char MotorID,unsigned char dir,unsigned int Degree)
{
}

/*������������ݵ���ž��������ĸ�*/
void Start_Motor(unsigned char MotorID,unsigned char dir,unsigned int Degree)
{
	if(MotorID>3){
		Start_Motor_S(MotorID,dir,Degree);
	}else{
		Start_Motor_SPTA(MotorID,dir,Degree);
	}
}

/*���¶�λ�����������еĵ�����е�ָ��λ��*/
void Reposition_Motor(unsigned char MotorID,unsigned int NewPos)
{
	MOTOR_CONTROL_S *pmotor_s=NULL;
	 
	switch(MotorID)
	{
		case 1:
			//pmotor_s=&motor1;
			break;
		case 2:
			//pmotor_s=&motor2;
			break;
		case 3:
			//pmotor_s=&motor3;
			break;
		case 4:
			//pmotor_spta=&motor4;
		  pmotor_s=&motor4;
			break;
		default:
			return;
	} 
	if(pmotor_s!=NULL){
		if(NewPos<=pmotor_s->MaxPosition&&NewPos!=pmotor_s->CurrentPosition)
		{
			if(NewPos>pmotor_s->CurrentPosition)
			{
				Start_Motor_S(MotorID,pmotor_s->clockwise,NewPos-pmotor_s->CurrentPosition); 
			}
			else
			{
				Start_Motor_S(MotorID,!pmotor_s->clockwise,pmotor_s->CurrentPosition-NewPos);  	
			}		 		 
			while(pmotor_s->running==1);
		}
	}
	
}

/*ͬʱ�������12*/
void Start_Motor12(unsigned char dir1,unsigned int Degree1,unsigned char dir2,unsigned int Degree2)
{
	
}

/*���õ�������ٶȣ���ڲ������ٶȵȼ�*/
void SetSpeed(unsigned char MotorID,signed char speedindex)
{
	
}

/*���õ����λ�ã�������е�ָ����λ��*/
void SetPosition(unsigned char MotorID,unsigned int dest)
{ 
	MOTOR_CONTROL_S *pmotor_s = NULL; 
	switch(MotorID)
	{
		case 1:
			//pmotor_s=&motor1; 
			break;
		case 2:
			//pmotor_s=&motor2; 
			break;
		case 3:
			//pmotor_s=&motor3; 
			break;
		case 4:
			
		  pmotor_s=&motor4; 
			break;
		default:
			return;
	}
	
	if(pmotor_s!=NULL){
		if(dest<=pmotor_s->MaxPosition&&dest!=pmotor_s->CurrentPosition)
		{
			if(dest>pmotor_s->CurrentPosition)
			{
				Start_Motor_S(MotorID,pmotor_s->clockwise,dest-pmotor_s->CurrentPosition); 
			}
			else 
			{
				Start_Motor_S(MotorID,!pmotor_s->clockwise,pmotor_s->CurrentPosition-dest);  	
			}      			
			while(pmotor_s->running==1);
		}
	}
	
}

/*��λ���*/
void Do_Reset(unsigned char MotorID)
{
	MOTOR_CONTROL_S *pmotor_s=NULL;	
	switch(MotorID)
	{
		case 1:
			//pmotor_s=&motor1;
			break;
		case 2:
			//pmotor_s=&motor2;
			break;
		case 3:
			//pmotor_s=&motor3;
			break;
		case 4:
			//pmotor_spta=&motor4;
		  pmotor_s=&motor4;
			break;
		default:
			return;
	} 
	/*do reset*/
	if(pmotor_s!=NULL){
		pmotor_s->rstflg=1;
		pmotor_s->running=1;
		SetSpeed(MotorID,16);
		while(pmotor_s->running==1);
	}
}


void Deal_Serail_Cmd(char *buf,int len)
{
	
}








