#ifndef _FOC_SVPWM_H
#define _FOC_SVPWM_H


extern int SVM(int alpha, int beta, int* tA, int* tB, int* tC);

void Svpwm_calculation(int mod_alpha, int mod_beta);

#endif