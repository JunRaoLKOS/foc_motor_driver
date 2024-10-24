#include "app_error_check.h"
#include "Rte_Ctrl_interface.h"
#include "foc_states.h"

uint16_t block_timer_cnt = 0;

void APP_MotorBlockCheck(void)
{
	int fbk_iq = Rte_get_CurrentLoopFbkIq();
	
	if(fbk_iq > Rte_get_MotorBlockCurrent())
	{
		if(block_timer_cnt++ > Rte_get_Block_TimeOut())
		{
			//foc_states_change(FOC_ERROR);
		}
	}
	else
	{
		block_timer_cnt = 0;
	}
}	

