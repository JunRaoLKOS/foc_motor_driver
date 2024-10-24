#ifndef _FOC_STATES_H
#define _FOC_STATES_H

extern short Foc_states;

#define FOC_OFF   							(0)
#define FOC_SPEED_MODE    			(1)
#define FOC_POSITION_MODE 			(2)
#define FOC_CUR_POS_MODE    		(3)
#define FOC_CALIB 							(4)
#define FOC_ERROR 							(5)
#define FOC_CUR_OPEN_LOOP_TEST 	(6)

extern void foc_states_task(void);
extern void foc_states_change(short new_states);

#endif