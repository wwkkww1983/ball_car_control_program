//头文件
#include "motorSalgorithm.h" 
#include "main.h"

#define F2TIME_PARA				64000//12000000   									//将频率值转换为定时器寄存器值得转换参数
#define STEP_PARA					10										//任意时刻转动步数修正因子
#define STEP_AA						31//31       										//加加速阶段，离散化点数
#define STEP_UA						31//31			  									//匀加速阶段，离散化点数
#define STEP_RA						31//31													//减加速阶段，离散化点数

#define STEP_SPTA					20													//SPTA最大速度等级
#define MAXSPEED_SPTA			80000												//SPTA最大速度
#define ACCSPEED_SPTA			150000											//SPTA加速度


uint16_t Motor4TimeTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor4StepTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};	


uint16_t Motor4_23TimeTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor4_23StepTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};	

/*将参数降为1/3 S型曲线参数生成的表格*/

uint16_t Motor4_13TimeTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};
uint16_t Motor4_13StepTable[2*(STEP_AA+STEP_UA+STEP_RA)+1] = { 0};		

unsigned long long Get_Time_Cost(unsigned char MotorID);

/*
电机1：PA8(pwm),PE9(CW),PE8(ENABLE),MXX:PA11,PA12,PE7
电机2：PA0(pwm),PA1(CW),PC3(ENABLE),MXX:PC0,PC1,PC2
电机3：PA6(pwm),PA7(CW),PC4(ENABLE),MXX:PA3,PA4,PA5
电机4：PB6(pwm,gpio),PB9(CW),PB8(ENABLE),MXX:PD7,PB5,PB7
*/
void Initial_MotorIO(void)
{
   
}



 /**************************************************************************************
 初始化电机的参数，主要是细分选择，使用的定时器，顺时针方向值，电机ID等
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

		pmotor->CurrentPosition = 40;//当前的中点
		pmotor->CurrentPosition_Pulse = 40*StepDive;//当前的脉冲数
		pmotor->StartTableLength=STEP_AA+STEP_UA+STEP_RA+1;
		pmotor->StopTableLength=STEP_AA+STEP_UA+STEP_RA; 
		pmotor->Counter_Table=MotorTimeTable;
		pmotor->Step_Table=MotorStepTable;

		pmotor->CurrentIndex=0;
		pmotor->speedenbale=0;
		pmotor->StartSteps=0;                  //必须清零，后面是累加，否则会把前一次的加上
		pmotor->StopSteps=0;                   //同上
		for(i=0;i<pmotor->StartTableLength;i++)
		 pmotor->StartSteps+=pmotor->Step_Table[i];
		for(i=0;i<pmotor->StopTableLength;i++)
		 pmotor->StopSteps+=pmotor->Step_Table[i+pmotor->StartTableLength]; 
		pmotor->TIMx->ARR =pmotor->Counter_Table[0]; //设置周期
		pmotor->TIMx->CCR3 =pmotor->Counter_Table[0]>>1;//设置占空比 通道3   		
	}
}

/*多轴协同使用了算法原理进行时间预估，所以修改该算法时记得
 这两处保持同步*/
/*计算S型曲线反转点，S型曲线在运行时，加减速过程是完全对称的*/
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
		//考虑这种情况，第一频率142 步，第二频率148步，要是运动200步该怎么运行
		//所以这里要改变第二频率的步数
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
			pmotor->StartSteps=0;                  //必须清零，后面是累加，否则会把前一次的加上
			pmotor->StopSteps=0;                   //同上
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


/*重新初始化电机运行时相关参数*/
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
	
	pmotor->Counter_Table=MotorTimeTable;  		//指向启动时，时间基数计数表
  pmotor->Step_Table=MotorStepTable;  			//指向启动时，每个频率脉冲个数表
	pmotor->StartSteps=0;                  //必须清零，后面是累加，否则会把前一次的加上
	pmotor->StopSteps=0;                   //同上
	for(i=0;i<pmotor->StartTableLength;i++)
	 pmotor->StartSteps+=pmotor->Step_Table[i];
	for(i=0;i<pmotor->StopTableLength;i++)
	 pmotor->StopSteps+=pmotor->Step_Table[i+pmotor->StartTableLength];
	if(pmotor->PulsesGiven<pmotor->StartSteps+pmotor->StopSteps){
		//如果给定的运行步数小余最大S型曲线，则尝试选择2/3S型曲线
		pmotor->Counter_Table=MotorTime23Table;  		
		pmotor->Step_Table=MotorStep23Table;  			
		pmotor->StartSteps=0;                  
		pmotor->StopSteps=0;                   
		for(i=0;i<pmotor->StartTableLength;i++)
		 pmotor->StartSteps+=pmotor->Step_Table[i];
		for(i=0;i<pmotor->StopTableLength;i++)
		 pmotor->StopSteps+=pmotor->Step_Table[i+pmotor->StartTableLength];
		if(pmotor->PulsesGiven<pmotor->StartSteps+pmotor->StopSteps){
			//如果给定的运行步数小余最大S型曲线，则尝试选择1/3S型曲线
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
	    	
	pmotor->TIMx->ARR =pmotor->Counter_Table[0]; //设置周期
	pmotor->TIMx->CCR1 =pmotor->Counter_Table[0]>>1;       //设置占空比
	pmotor->Time_Cost_Act=pmotor->TIMx->ARR;
	Get_TimeCost_ReverDot_S(MotorID);		 
		
}
 
/*根据S型曲线参数获取某个时刻的频率*/
float GetFreAtTime(float fstart,float faa,float taa,float tua,float tra,float t)
{
		//根据公式计算从开始到最高速过冲中，t时刻的转动频率
	  if(t>=0&&t<=taa){
			//加加速阶段
			return fstart+0.5*faa*t*t;
		}else if(taa<t&&t<=(taa+tua)){
			//匀加速阶段
			return fstart+0.5*faa*taa*taa+(t-taa)*faa*taa;
		}else if((taa+tua)<t&&t<=(taa+tua+tra)){
			//减加速阶段
			return fstart+0.5*faa*taa*taa+(tua)*faa*taa+0.5*faa*taa*tra-0.5*faa*taa*(taa+tua+tra-t)*(taa+tua+tra-t)/(tra);
		}		
		return 0;
}
 
 /*计算S型曲线算法的每一步定时器周期及步进数*/
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
电机运行参数初始化*/
void MotorRunParaInitial(void)
{ 
	/*FIXME:用户可以改变该参数实现S型曲线的升降特性*/ 
	CalcMotorPeriStep_CPF(M_FRE_START,M_FRE_AA,M_T_AA,M_T_UA,M_T_RA,Motor4TimeTable,Motor4StepTable);

	/*更改参数降为2/3生成的表格*/
	CalcMotorPeriStep_CPF(M_FRE_START*2.0/3,M_FRE_AA*2.0/3,M_T_AA*2.0/3,M_T_UA*2.0/3,M_T_RA*2.0/3,Motor4_23TimeTable,Motor4_23StepTable); 


	/*更改参数降为1/3生成的表格*/
  CalcMotorPeriStep_CPF(M_FRE_START*1.0/3,M_FRE_AA*1.0/3,M_T_AA*1.0/3,M_T_UA*1.0/3,M_T_RA*1.0/3,Motor4_13TimeTable,Motor4_13StepTable); 

}

 
/**************************************************************************************
两个电机同时运行时，花费时间较少的电机要根据时间长的电机调整运行参数
*/
void Find_BestTimeCost(unsigned char ID,unsigned long long time_cost,unsigned char dir,unsigned int Degree)
{
	
}


/**************************************************************************************
启动电机按照S型曲线参数运行*/
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
	pmotor->TIMx->ARR =pmotor->Counter_Table[0]; //设置周期
	pmotor->TIMx->CCR3 =pmotor->Counter_Table[0]>>1;       //设置占空比
  TIM_Cmd(pmotor->TIMx, ENABLE);		  //DISABLE
}

/*启动电机按照SPTA方式运行*/
void Start_Motor_SPTA(unsigned char MotorID,unsigned char dir,unsigned int Degree)
{
}

/*启动电机，根据电机号决定调用哪个*/
void Start_Motor(unsigned char MotorID,unsigned char dir,unsigned int Degree)
{
	if(MotorID>3){
		Start_Motor_S(MotorID,dir,Degree);
	}else{
		Start_Motor_SPTA(MotorID,dir,Degree);
	}
}

/*重新定位，让正在运行的电机运行到指定位置*/
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

/*同时启动电机12*/
void Start_Motor12(unsigned char dir1,unsigned int Degree1,unsigned char dir2,unsigned int Degree2)
{
	
}

/*设置电机运行速度，入口参数是速度等级*/
void SetSpeed(unsigned char MotorID,signed char speedindex)
{
	
}

/*设置电机的位置，电机运行到指定的位置*/
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

/*复位电机*/
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








