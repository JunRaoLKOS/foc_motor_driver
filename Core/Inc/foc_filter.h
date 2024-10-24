#ifndef _FOC_FILTER_H
#define _FOC_FILTER_H



extern int Low_pass_filter(int A,int X,int Y);
extern int Speed_smooth_pass_filter(int* Buffer,int X,int n);

extern double KalmanFilterSpeed(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R,double InitialPrediction);












#endif
