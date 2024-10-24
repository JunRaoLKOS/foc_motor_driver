#ifndef _FOC_SPEED_LOOP_H
#define _FOC_SPEED_LOOP_H
#include  "stdint.h"
extern void foc_Speed_loop(void);

extern void foc_check_speed_fbk(void);
extern float spd_rad_to_rpm(float rad);
extern float spd_rpm_to_rad(float rpm);
extern float foc_updata_Fbk_speed_rad(void);

#endif