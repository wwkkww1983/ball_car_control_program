//����ͷ�ļ�
#include "motorSalgorithm.h" 
#include "ringbuffer.h"

///*�򵥵���ʱ����*/
void TimerDly(unsigned int Time)
{
	unsigned int i=0;
	while(Time)
	{
		for(i=0;i<8000;i++);
		Time--;
	}
} 
/*�����������*/
int motortest(void)
{	
	
	


}


