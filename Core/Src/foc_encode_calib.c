#include "foc_encode_calib.h"
#include "Rte_Ctrl_interface.h"
#include "position_sensor.h"
#include "foc_current_loop.h"
#include "foc_sin_table.h"

#define CALIB_Id_MAX_CUR (1500)

uint16_t foc_encode1_angle = 0;
/*
*****************************************************************************************
* name  : foc_updata_encode
*
* input : 
*
* return: void
*****************************************************************************************
*/ 
uint16_t foc_updata_encode(void)
{
	uint16_t tempPh = 0;
	
	foc_encode1_angle = get_encode_val(1);
	
	tempPh = ((SIN_TABLE_NUM*POLE_PAIRS * (foc_encode1_angle % ENCODE_RANGE))/ENCODE_RANGE);
	
	tempPh -= Rte_get_MotorEncodeOffset();

	tempPh = tempPh % (SIN_TABLE_NUM);
 
	return 0;
}
 
/*
*****************************************************************************************
* name  : foc_encode_calib
*
* input : 
*
* return: void
*****************************************************************************************
*/uint16_t  max_CALIB_Id_MAX_CUR = 1000;
void foc_encode_calib(void)
{
	static uint16_t calib_Ref_id = 0;

	if(calib_Ref_id < max_CALIB_Id_MAX_CUR)
	{
		Rte_set_CurrentLoopTargetId(calib_Ref_id++);
		Rte_set_CurrentLoopTargetIq(0);
	} 
	else
	{
		uint16_t encode_offset = (((2*M_PI) * (((AS5047_read1(READ_ANGLECOM)) % ENCODE_RANGE/POLE_PAIRS)))/ENCODE_RANGE/POLE_PAIRS);
		Rte_set_MotorEncodeOffset(encode_offset);
	}
}


