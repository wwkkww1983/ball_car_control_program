#include "kalman.h"



double Data_Kalman(kalman_parm_t *kalman_parm,double data)
{
	double x_mid;	    //当前的估算值 一般和前一时刻的最优值相等
	double x_now;      //当前时刻的最优值 
	double p_mid;      //当前的估算值的误差
	double p_now;      //当前时刻的最优误差
	double data_measure;  //测试值
	double kg;         //权值    
	
	//测量值
	data_measure=data;
	
	//x_last=x(k-1|k-1),x_mid=x(k|k-1)
	x_mid=kalman_parm->x_last;   
	//p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
	p_mid=sqrt(kalman_parm->p_last * kalman_parm->p_last+kalman_parm->Q*kalman_parm->Q);  
	//p_mid/(p_mid+Q); //kg为kalman filter，R为噪声
	kg=sqrt(p_mid * p_mid/(p_mid * p_mid +kalman_parm->R * kalman_parm->R));
	//估计出的最优值
	x_now=x_mid+kg*(data_measure-x_mid);
	//最优值对应的covariance
	p_now=sqrt((1-kg)*(p_mid * p_mid));
  //更新covariance值
	kalman_parm->p_last = p_now;  
  //更新系统状态值
	kalman_parm->x_last = x_now;  
  //显示测量值以及真值与测量值之间的误差
	
	return x_now;
	
}