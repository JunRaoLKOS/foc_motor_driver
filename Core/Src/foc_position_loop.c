#include "foc_position_loop.h"
#include "Rte_Ctrl_interface.h"
#include "position_sensor.h"
#include "foc_encode_calib.h"
#include "foc_pid.h"

float Kt = 1.4;

	
int foc_position_fbk = 0;

/*
*****************************************************************************************
* name  : position_encode_to_rad
*
* input : 
*
* return: void
*****************************************************************************************
*/
float pos_encode_to_rad(float encode)
{
	//得到弧度制的角度，范围在0-6.28  
	return ((float)encode * (ONE_CIRCLE_RADIAN)/ENCODE_RANGE/MOTOR_REDUCTION_RATIO);
}
/*
*****************************************************************************************
* name  : position_rad_to_encode
*
* input : 
*
* return: void
*****************************************************************************************
*/
float pos_rad_to_encode(float rad)
{
	//得到弧度制的角度，范围在0-6.28
	return (rad *MOTOR_REDUCTION_RATIO*ENCODE_RANGE)/(ONE_CIRCLE_RADIAN);
}
/*
*****************************************************************************************
* name  : foc_Torque_position_loop
*
* input : 
*
* return: void
*****************************************************************************************
*/
float pos_desired;
float pos_current;
float init_touque = 0;

void foc_Torque_position_loop(void)
{
	float KP = Rte_get_KP();
	float KD = Rte_get_KD();
	
	pos_desired = Rte_get_TargetPositionRad();
	pos_current = Rte_get_FbkPositionRad();
	
	float spd_desired = Rte_get_TargetSpeedRad();
	float spd_current = Rte_get_FbkSpeedRad();
		
	int motor_current = (KP*(pos_desired-pos_current) + KD*(spd_desired-spd_current) + init_touque)/Kt;
	 
	Rte_set_CurrentLoopTargetIq(motor_current*1000);
}

/*
*****************************************************************************************
* name  : motor_positionSoftwareStartUp
*
* input : 
*
* return: void
*****************************************************************************************
*/
int foc_positionSoftwareStartUp(  int ref)
{
	int getTargetPosition = Rte_get_PositonEncode();
	int refTargetPosition = ref;
	int ret = 0;
	
	/* speed step up/down */
	if( getTargetPosition > 0 )
	{
		refTargetPosition = UPDNLIMT_SPEED(refTargetPosition,getTargetPosition,position_amp_val);
	}
	else 
	{
		refTargetPosition = UPDNLIMT_SPEED(-refTargetPosition,-getTargetPosition,position_amp_val);
		refTargetPosition = -refTargetPosition;
	}
	return refTargetPosition;
}
/*
*****************************************************************************************
* name  : foc_position_loop
*
* input : 
*
* return: void
*****************************************************************************************
*/
void foc_position_loop(void)
{
	g_MotorPIArgs.position.Ref = foc_positionSoftwareStartUp(g_MotorPIArgs.position.Ref);
	g_MotorPIArgs.position.Fbk = foc_position_fbk;

	CNTL_PI_IQ_C( g_MotorPIArgs.position );
	
	Rte_set_TargetSpeedRpm(g_MotorPIArgs.position.Out);
}
/*
*****************************************************************************************
* name  : foc_check_position_fbk
*
* input : 
*
* return: void
*****************************************************************************************
*/
int offset_position = 0;
int foc_check_position_fbk(void)
{
	static int32_t encoder_state_b = 0;
	
	int real_pos = get_encode_val(1);

	offset_position = ((real_pos - encoder_state_b)*position_amp_val*ENCODE_RANGE)/ENCODE_RANGE;
	encoder_state_b   = real_pos;
		  
	foc_position_fbk += offset_position/position_amp_val;
	
	return foc_position_fbk;
}




