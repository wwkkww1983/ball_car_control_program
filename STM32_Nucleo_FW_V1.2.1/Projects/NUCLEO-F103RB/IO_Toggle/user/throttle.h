#ifndef __THROTTLE_H
#define __THROTTLE_H

#include "stm32f10x.h"

/* AD：
球包电流：箱体电机电流：PC0,PC3
推杆电流：右推杆电流：PC2
          左推杆电流：PC1
AD48v电流：PB0
有门电流：PB1
电池温度检测：：PC4

PC0---AD1 IN10
PC1---AD1 IN11
PC2---AD1 IN12
PC3---AD1 IN13
PC4---AD1 IN14

PB0---AD1 IN8
PB1---AD1 IN9

After_filter[0]---PB0---IN8---48v的电压检测
After_filter[1]---PB1---IN9---油门的电压检测
After_filter[2]---PC0---IN10---球包电机
After_filter[3]---PC1---IN11---左推杆的电流
After_filter[4]---PC2---IN12---右推杆的电流
After_filter[5]---PC3---IN13---球包电机的刹车电流
After_filter[6]---PC4---IN14---温度检测的电压
*/
 
#define N  1
#define M  2
#define MAX_SPEED_COUNT 100
void Throttle_Adc_Init(void);
u16  Get_Throttle_Adc(u8 ch);  
void Machine_GetSpeed_Init(void);
void Machine_Adc_Init(void);
void filter(void);
void Get_Machine_Adc(void);
extern uint16_t AD_Value[N][M];
extern uint16_t After_filter[M];
//extern uint16_t After_filter[M];

#endif