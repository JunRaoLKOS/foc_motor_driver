#include "foc_states.h"
#include "foc_speed_loop.h"
#include "foc_current_loop.h"
#include "Rte_Ctrl_interface.h"
#include "foc_encode_calib.h"
#include "foc_position_loop.h"
#include "drv8323.h"
#include "tim.h"
 
short Foc_states;
extern DRVStruct drv;
/*
*****************************************************************************************
* name  : foc_stats_change.
*
* input : 
*
* return: void
*****************************************************************************************
*/
void foc_states_change(short new_states)
{
	switch(new_states)
	{
		case FOC_OFF:
		case FOC_ERROR:
			if(new_states != Foc_states)
			{
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
				drv_disable_gd(drv);
				
				Rte_set_MotorOff();
				
				Foc_states = new_states;
			}
			break;
		
		case FOC_SPEED_MODE:
		case FOC_POSITION_MODE:
	  case FOC_CUR_POS_MODE:
		case FOC_CALIB:
			if(new_states != Foc_states)
			{
				if(FOC_ERROR != Foc_states)
				{
					/* Turn on PWM */
					HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
					HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
					HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

					Rte_set_MotorOn();
					drv_enable_gd(drv);
					Foc_states = new_states;
				}
			}
			break;
			
		case FOC_CUR_OPEN_LOOP_TEST:
			if(new_states != Foc_states)
			{
				if(FOC_ERROR != Foc_states)
				{
				/* Turn on PWM */
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

				drv_enable_gd(drv);
				Foc_states = new_states;
				}
			}
			break;
		
		default:
			break;
	}
}

extern float fbk_spd_rad;
/*
*****************************************************************************************
* name  : foc_states_task.
*
* input : 
*
* return: void
*****************************************************************************************
*/
int test_speed = 0;
uint8_t speed_loop_count = 0,position_loop_count=0,speed_updata_count=0;
 
void foc_states_task(void)
{ 
	uint16_t motor_phase = foc_updata_encode();
	
	foc_check_speed_fbk();
	foc_check_position_fbk();
	
	/* updata speed  */
	if(speed_updata_count++>20)
	{ 
			foc_updata_Fbk_speed_rad();
			speed_updata_count = 0;
	}
			
	switch(Foc_states)
	{
		case FOC_CUR_POS_MODE:
			
			foc_Torque_position_loop();
			
			break;
		
		case FOC_POSITION_MODE:
		case FOC_SPEED_MODE:
			if(Foc_states == FOC_POSITION_MODE)
			{
				if(position_loop_count++ > 6)
				{
					foc_position_loop();
					position_loop_count = 0;
				}
			}	
			if(speed_loop_count++ > 2)
			{
				foc_Speed_loop();
				speed_loop_count = 0;
			}
			break;
		
		case FOC_CALIB:
			motor_phase = 0;
			foc_encode_calib();
			break;
		
		case FOC_CUR_OPEN_LOOP_TEST:
				motor_phase += 1;
				motor_phase = motor_phase%2048;
				//motor_phase = foc_updata_encode();
			break;
	}
	Current_loop(Rte_get_CurrentLoopTargetId(),Rte_get_CurrentLoopTargetIq(),motor_phase);
}
