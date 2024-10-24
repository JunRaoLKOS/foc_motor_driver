#ifndef _FOC_ENCODE_CALIB_H
#define _FOC_ENCODE_CALIB_H
#include "stdint.h"

#define POLE_PAIRS          (10)
#define ENCODE_RANGE 				(16384)

extern uint16_t foc_encode1_angle;

extern void foc_encode_calib(void);
extern uint16_t foc_updata_encode(void);


#endif