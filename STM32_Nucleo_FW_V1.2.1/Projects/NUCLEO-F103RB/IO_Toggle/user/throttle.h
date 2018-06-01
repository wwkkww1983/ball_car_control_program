#ifndef __THROTTLE_H
#define __THROTTLE_H

#include "stm32f10x.h"

/* AD��
���������������������PC0,PC3
�Ƹ˵��������Ƹ˵�����PC2
          ���Ƹ˵�����PC1
AD48v������PB0
���ŵ�����PB1
����¶ȼ�⣺��PC4

PC0---AD1 IN10
PC1---AD1 IN11
PC2---AD1 IN12
PC3---AD1 IN13
PC4---AD1 IN14

PB0---AD1 IN8
PB1---AD1 IN9

After_filter[0]---PB0---IN8---48v�ĵ�ѹ���
After_filter[1]---PB1---IN9---���ŵĵ�ѹ���
After_filter[2]---PC0---IN10---������
After_filter[3]---PC1---IN11---���Ƹ˵ĵ���
After_filter[4]---PC2---IN12---���Ƹ˵ĵ���
After_filter[5]---PC3---IN13---��������ɲ������
After_filter[6]---PC4---IN14---�¶ȼ��ĵ�ѹ
*/
 
#define N  1
#define M  2

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