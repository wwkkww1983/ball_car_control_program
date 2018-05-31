/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GLOBAL_VARIAL_H
#define __GLOBAL_VARIAL_H

/*���°�����Ҫ��ͷ�ļ����������ݸ�ʽ�ļ�*/
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
 __IO unsigned char en;	          //ʹ��
 __IO unsigned char dir;			  		//����
 __IO unsigned char running;		  		//ת����ɱ�־ 
 __IO unsigned char rstflg;		  	//��λ��־
 __IO unsigned char divnum;		  	//��Ƶ��
 __IO unsigned char speedenbale;		//�Ƿ�ʹ���ٶȿ���	
 __IO unsigned char clockwise;			//˳ʱ�뷽���Ӧ��ֵ
 __IO unsigned char id;					//���id
 
 __IO uint32_t pulsecount;
 __IO uint16_t *Counter_Table;  		//ָ������ʱ��ʱ�����������
 __IO uint16_t *Step_Table;  			//ָ������ʱ��ÿ��Ƶ�����������
 __IO uint16_t CurrentIndex;    	//��ǰ���λ��
 __IO uint16_t TargetIndex;    	//Ŀ���ٶ��ڱ���λ��
 __IO uint16_t StartTableLength;   //�������ݱ�
 __IO uint16_t StopTableLength;    //�������ݱ�
 __IO uint32_t StartSteps;					//�����������
 __IO uint32_t StopSteps;					//���ֹͣ����
 __IO uint32_t RevetDot;			  		//����˶��ļ��ٵ�
 __IO uint32_t PulsesGiven;			  //����˶����ܲ���
 __IO uint32_t PulsesHaven;				//����Ѿ����еĲ���
 __IO uint32_t CurrentPosition;		//��ǰλ��
 __IO uint32_t MaxPosition;				//���λ�ã�������λ����0
 __IO uint32_t CurrentPosition_Pulse;		//��ǰλ��
 __IO uint32_t MaxPosition_Pulse;		//��ǰλ��
 __IO unsigned long long Time_Cost_Act;	//ʵ����ת���ѵ�ʱ��
 __IO unsigned long long Time_Cost_Cal;	//����Ԥ����ת���ѵ�ʱ��
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

#define PWM1_PreemptionPriority 1             //�׼�
#define PWM1_SubPriority 0					//�ײ�
#define PWM2_PreemptionPriority 1             //�׼�
#define PWM2_SubPriority 1					//�ײ�
#define PWM3_PreemptionPriority 2             //�׼�
#define PWM3_SubPriority 0					//�ײ�
#define PWM4_PreemptionPriority 2             //�׼�
#define PWM4_SubPriority 1					//�ײ�
/***********************END********************************************/

//���
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
