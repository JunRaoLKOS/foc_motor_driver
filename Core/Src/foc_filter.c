#include "foc_filter.h"


/*
*****************************************************************************************
* name  : filter
*					Y=A*X+(1-A)*Y0 -> Y=Y0+((X-Y0)*A)/1000; 0<A<1000
* input : 
*
* return: void
*****************************************************************************************
*/
int Low_pass_filter(int A,int X,int Y)
{
	return Y+((X-Y)*A)/1000;
}

/*
*****************************************************************************************
* name  : Speed_Low_pass_filter
*
* input : 
*
* return: void
*****************************************************************************************
*/
int   buff_sum = 0;
int   real_speed_buff_point;
int   real_speed_buff[10];
int Speed_smooth_pass_filter(int* Buffer,int X,int n)
{
	int i;

	buff_sum=0.0f;
	real_speed_buff_point++;
	if(real_speed_buff_point>=n)
	{
		real_speed_buff_point=0;
	}
	Buffer[real_speed_buff_point]=X;
	for(i=0;i<n;i++)
	{
		buff_sum+=Buffer[i];
	}
	return buff_sum/n;
}

/*
*****************************************************************************************
* name  : KalmanFilterSpeed
*
* input : 
*
* return: void
*****************************************************************************************
*/
double x_last;
double KalmanFilterSpeed(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R,double InitialPrediction)
{
	double R = MeasureNoise_R;
	double Q = ProcessNiose_Q;

	//static double x_last;

	double x_mid = x_last;
	double x_now;

	static double p_last;

	double p_mid ;
	double p_now;
	double kg;   
	
	if (InitialPrediction == 0)
	{
		x_last = ResrcData;
	}  

	x_mid = x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
	p_mid = p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=
	kg    = p_mid/(p_mid+R); //kalman filter
	x_now=x_mid+kg*(ResrcData-x_mid);
                
	p_now=(1-kg)*p_mid;       

	p_last = p_now; 
	x_last = x_now; 

	return x_now;                
}




