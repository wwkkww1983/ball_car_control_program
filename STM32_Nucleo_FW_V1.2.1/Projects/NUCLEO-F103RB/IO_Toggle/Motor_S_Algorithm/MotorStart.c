//包含头文件
#include "motorSalgorithm.h" 
#include "ringbuffer.h"

///*简单的延时函数*/
void TimerDly(unsigned int Time)
{
	unsigned int i=0;
	while(Time)
	{
		for(i=0;i<8000;i++);
		Time--;
	}
} 
/*电机驱动测试*/
int motortest(void)
{	
	
	


}


