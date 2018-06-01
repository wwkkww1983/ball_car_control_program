/**
  ******************************************************************************
  * @file    main.h 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    11-April-2014
  * @brief   Header for main.c module
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
#include "stm32f10x.h"
#include "includes.h"

#define S_ANGLE 2
#define S_SPEED 1
#define B_MACHINE 3

#define MACHINE_BREAK_STATE 0
#define MACHINE_NOBREAK_STATE 1


#define Deceleration_Ratio 20
#define SPEED_TIMER_PERIOD 1500
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

#define    UPINDEX     0
#define    MIDINDEX    1
#define    DOWNINDEX   2 
#define    NONEINDEX   3

/*代码由叶以亮增加2017.11.15*/
// #define Direction1  GPIO_Pin_5  //PORTC
// #define Break1     GPIO_Pin_6  //PORTC  
// #define Enable1     GPIO_Pin_7  //PORTC
// #define PWM1        GPIO_Pin_0 //PORTB
// 
// #define Direction2   GPIO_Pin_8 //PORTC
// #define Break2     GPIO_Pin_9  //PORTC
// #define Enable2      GPIO_Pin_10  //PORTC
// #define PWM2         GPIO_Pin_1//PORTB
// 
// #define Machine_Front_Motor_Enable_Pin  GPIO_Pin_7 //PA7
// #define Machine_Front_Motor_Direction_Pin GPIO_Pin_9 //PB9
// #define Machine_Front_Motor_PWM_Pin   GPIO_Pin_8 //PB8
// 
//// #define Machine_Front_Motor_Reset1 GPIO_Pin_4//PB6
//// #define Machine_Front_Motor_Reset2 GPIO_Pin_5 //PB5
//// #define Machine_Front_Motor_Reset3 GPIO_Pin_6
 

 #define Machine_Front_Motor_Turn_Left 1
 #define Machine_Front_Motor_Turn_Right 0
 
 #define GO_FORWARD 0
 #define GO_BACK 1
 #define GO_LEFT 2
 #define GO_RIGHT 3
 /*叶以亮增加的代码结束2017.11.15*/
 #define MOTOR_RATIO 1600
 #define Machine_Front_Motor_Goforward 0
 #define Machine_Front_Motor_Goleft 1
 #define Machine_Front_Motor_Goright 2

 extern __IO uint32_t Machine_Front_PWM_Count;
 extern __IO uint8_t Manual_Break_State ;
 extern __IO uint8_t Car_Change_Direction ;	
 extern OS_EVENT *Sem_Event;
 extern __IO uint8_t Car_Change_Direction_Old;
  
/* Includes ------------------------------------------------------------------*/
//#include "stm32f1xx_nucleo.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void Middle_Motor_Init(void);
static void Middle_Motor_Run(void);
static void Middle_Motor_Break(void);
static void Middle_Motor_NO_Break(void);
static void Middle_Motor_Stop(void);
static void Middle_Motor_Goforward(void);
static void Middle_Motor_Goback(void);



void Delay(__IO uint32_t nTime);
void Machine_Speed_Set(uint16_t DutycycleL,uint16_t DutycycleR);
void Machine_Diretion_Init(void);
void Machine_Direction_Switch(uint8_t Dir);

void Machine_Enable_Init(void);
void Machine_Enable(void);
void Machine_Disable(void);

void Machine_Break_Init(void);
void Machine_Nobreak(void);
void Machine_Break(void);


void Machine_Front_Motor_Dirtion_Init(void);
void Machine_Front_Motor_Directon_Control(uint8_t Dir);

void Machine_Front_Motor_Angle_Control(uint16_t angle,uint16_t time);

void Machine_Front_Motor_Enable_Or_Disable_Init(void);
void Machine_Front_Motor_Enable_or_Disable(FunctionalState NewState);

void Machine_Motor_Alarm_Detected_Init( void );
uint8_t Machine_Motor_Alarm_Detected(void);
uint8_t Machine_Motor_Alarm_Detected(void);
uint8_t Machine_Motor_PG_Dectected(void);


void Machine_Front_Motor_Reset_Dectected_Init(void);
uint8_t  Machine_Front_Motor_Reset_Detected(void);

void MIC_Motor_Init(void);
void MIC_Motor_Run(void);
void MIC_Motor_Stop(void);
void MIC_Motor_Close_Detected_Init(void);
uint8_t MIC_Motor_Close_Detected(void);

void Battery_Temperature_Detected_Init(void);
void Detected_Rain_Init(void);

void Usb_Power_Init(void);
void Usb_Power_On(void);
void Usb_Power_OFF(void);

void Ultrasonsic_Power_Init(void);
void Ultrasonsic_Power_On(void);
void Ultrasonsic_Power_OFF(void);

void Backmotor_Power_Init(void);
void Backmotor_Power_On(void);
void Backmotor_Power_OFF(void);

void Aheadmotor_Power_Init(void);
void Aheadmotor_Power_On(void);
void Aheadmotor_Power_OFF(void);

void Ahead_Light_Power_Init(void);
void Ahead_Light_Power_On(void);
void Ahead_Light_Power_OFF(void);

void Car24v_Power_Init(void);

void Electric_Push_Rod_Init(void);
void Electric_Push_Rod_Up(void);
void Electric_Push_Rod_Down(void);
void Electric_Push_Rod_Stop(void);
void Electric_Push_Rod_Detected_Stop(void);

/***/
void Car_Forward_Back_Detected_Init(void);
uint8_t Car_Forward_Back_Detected(void);
void Open_Car_Lid_Detected_Init(void);

uint8_t Open_Car_Lid_Detected(void);
void Close_Car_Lid_Detected_Init(void);

uint8_t Close_Car_Lid_Detected(void);
void Detected_Car_Lid_Init(void);

uint8_t Detected_Car_Lid(void);
void People_Seatsensor_Detected_Init(void);

uint8_t People_Seatsensor_Detected(void);
void Detected_Carbreak_Detected_Init(void);

uint8_t Detected_Carbreak_Detected(void);
void Stopnow_Detected_Init(void);

uint8_t Stopnow_Detected(void);
void Obstacle_From_Ultrasonic_Detected_Init(void);

uint8_t Obstacle_From_Ultrasonic_Detected(void);
void Switch_Auto_Manual_Drivecar_Detected_Init(void);

uint8_t Switch_Auto_Manual_Drivecar_Detected(void);

void  AD48V_Adc_Init(void);
u16 Get_AD48V_Adc(u8 ch);
void  battery_temperature_Adc_Init(void);
u16 Get_battery_temperature_Adc(u8 ch);

/***/
void SendDatatoMaster(uint8_t sendbuf[]);
void Machine_Motor_PG_Dectected_Init(void);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
