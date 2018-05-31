#include "kalman.h"



double Data_Kalman(kalman_parm_t *kalman_parm,double data)
{
	double x_mid;	    //��ǰ�Ĺ���ֵ һ���ǰһʱ�̵�����ֵ���
	double x_now;      //��ǰʱ�̵�����ֵ 
	double p_mid;      //��ǰ�Ĺ���ֵ�����
	double p_now;      //��ǰʱ�̵��������
	double data_measure;  //����ֵ
	double kg;         //Ȩֵ    
	
	//����ֵ
	data_measure=data;
	
	//x_last=x(k-1|k-1),x_mid=x(k|k-1)
	x_mid=kalman_parm->x_last;   
	//p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
	p_mid=sqrt(kalman_parm->p_last * kalman_parm->p_last+kalman_parm->Q*kalman_parm->Q);  
	//p_mid/(p_mid+Q); //kgΪkalman filter��RΪ����
	kg=sqrt(p_mid * p_mid/(p_mid * p_mid +kalman_parm->R * kalman_parm->R));
	//���Ƴ�������ֵ
	x_now=x_mid+kg*(data_measure-x_mid);
	//����ֵ��Ӧ��covariance
	p_now=sqrt((1-kg)*(p_mid * p_mid));
  //����covarianceֵ
	kalman_parm->p_last = p_now;  
  //����ϵͳ״ֵ̬
	kalman_parm->x_last = x_now;  
  //��ʾ����ֵ�Լ���ֵ�����ֵ֮������
	
	return x_now;
	
}