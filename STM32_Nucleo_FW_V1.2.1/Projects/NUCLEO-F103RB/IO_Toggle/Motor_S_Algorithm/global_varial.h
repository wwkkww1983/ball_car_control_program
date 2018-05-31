/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GLOBAL_VARIAL_H
#define __GLOBAL_VARIAL_H

/*以下包含必要的头文件，包括数据格式文件*/
/****************Start*******************/
//#include "STM32F10x_System.h"
//#include "stm32f10x_type.h"
#include "stm32f10x.h"
#include "stdint.h"
#include "stdlib.h"
#include "math.h"
#include "Macro.h" 


#include  <stdio.h>
#include  <string.h>
#include  <ctype.h>
#include  <stdlib.h>
#include  <stdarg.h>
//
//
#include  "stm32f10x_conf.h"
//#include  <stm32f10x.h>

//#include  <stm32f10x_lib.h>
//


#define MIN(a,b) (a<b) ? (a) : (b)
#define MAX(a,b) (a>b) ? (a) : (b)
#define rt_int8_t	   int8_t
#define rt_int16_t	   int16_t
#define rt_int32_t	   int32_t

#define rt_uint8_t	   uint8_t
#define rt_uint16_t	   uint16_t
#define rt_uint32_t	   uint32_t

#define IDLE						0
#define ACCELERATING		1
#define AT_MAX					2
#define DECELERATING		3

typedef  struct 
{
 __IO unsigned char en;	          //使能
 __IO unsigned char dir;			  		//方向
 __IO unsigned char running;		  		//转动完成标志 
 __IO unsigned char rstflg;		  	//复位标志
 __IO unsigned char divnum;		  	//分频数
 __IO unsigned char speedenbale;		//是否使能速度控制	
 __IO unsigned char clockwise;			//顺时针方向对应的值
 __IO unsigned char id;					//电机id
 
 __IO uint32_t pulsecount;
 __IO uint16_t *Counter_Table;  		//指向启动时，时间基数计数表
 __IO uint16_t *Step_Table;  			//指向启动时，每个频率脉冲个数表
 __IO uint16_t CurrentIndex;    	//当前表的位置
 __IO uint16_t TargetIndex;    	//目标速度在表中位置
 __IO uint16_t StartTableLength;   //启动数据表
 __IO uint16_t StopTableLength;    //启动数据表
 __IO uint32_t StartSteps;					//电机启动步数
 __IO uint32_t StopSteps;					//电机停止步数
 __IO uint32_t RevetDot;			  		//电机运动的减速点
 __IO uint32_t PulsesGiven;			  //电机运动的总步数
 __IO uint32_t PulsesHaven;				//电机已经运行的步数
 __IO uint32_t CurrentPosition;		//当前位置
 __IO uint32_t MaxPosition;				//最大位置，超过该位置置0
 __IO uint32_t CurrentPosition_Pulse;		//当前位置
 __IO uint32_t MaxPosition_Pulse;		//当前位置
 __IO unsigned long long Time_Cost_Act;	//实际运转花费的时间
 __IO unsigned long long Time_Cost_Cal;	//计算预估运转花费的时间
 TIM_TypeDef* TIMx;	
} MOTOR_CONTROL_S ;



#define M1_CLOCKWISE					0
#define M1_UNCLOCKWISE				1
#define M2_CLOCKWISE					0
#define M2_UNCLOCKWISE				1
#define M3_CLOCKWISE					0
#define M3_UNCLOCKWISE				1
#define M4_CLOCKWISE					0
#define M4_UNCLOCKWISE				1

#define PWM1_PreemptionPriority 1             //阶级
#define PWM1_SubPriority 0					//阶层
#define PWM2_PreemptionPriority 1             //阶级
#define PWM2_SubPriority 1					//阶层
#define PWM3_PreemptionPriority 2             //阶级
#define PWM3_SubPriority 0					//阶层
#define PWM4_PreemptionPriority 2             //阶级
#define PWM4_SubPriority 1					//阶层
/***********************END********************************************/

//电机
extern MOTOR_CONTROL_S motor1;
extern MOTOR_CONTROL_S motor2;
extern MOTOR_CONTROL_S motor3;
//extern MOTOR_CONTROL_SPTA motor4; 
extern  MOTOR_CONTROL_S motor4;


void Initial_MotorIO(void);
void Initial_Motor(unsigned char MotorID, unsigned char StepDive,unsigned int maxposition);
void MotorRunParaInitial(void);
void Start_Motor12(unsigned char dir1,unsigned int Degree1,unsigned char dir2,unsigned int Degree2);
void Start_Motor_S(unsigned char MotorID,unsigned char dir,unsigned int Degree);
void Start_Motor_SPTA(unsigned char MotorID,unsigned char dir,unsigned int Degree);
void SetSpeed(unsigned char MotorID, signed char speedindex);
void Do_Reset(unsigned char MotorID);
void Deal_Cmd(void);
void Initial_PWM_Motor1(void);
void Initial_PWM_Motor2(void);
void Initial_PWM_Motor3(void);
void Initial_PWM_Motor4(void);
void EXTI_Configuration(void);
void SetPosition(unsigned char MotorID,unsigned int dest);

#endif
