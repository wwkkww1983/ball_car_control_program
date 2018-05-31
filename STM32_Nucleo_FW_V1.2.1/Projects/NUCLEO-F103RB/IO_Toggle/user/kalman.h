#ifndef _KALMAN_H_
#define _KALMAN_H_
#include <math.h>
typedef struct{
	double x_last;
	double p_last;
	double Q;
	double R;
}kalman_parm_t;
double Data_Kalman(kalman_parm_t *kalman_parm,double data);
#endif