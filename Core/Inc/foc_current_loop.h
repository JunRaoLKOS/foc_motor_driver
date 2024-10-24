#ifndef _FOC_CURRENT_LOOP_H
#define _FOC_CURRENT_LOOP_H
#include <stdint.h>

typedef struct {
    int16_t phA;
    int16_t phB;
} foc_Iph_AB;

extern foc_Iph_AB Iph_Calib;

extern int get_calib_adc_current(unsigned int adcValue);

extern int get_run_adc_current(unsigned int adcValue);

extern void Current_loop(int Target_Id, int Target_Iq,int angle); 




#endif
