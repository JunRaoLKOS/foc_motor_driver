#include "foc_speed_loop.h"
#include "foc_encode_calib.h"
#include "Rte_Ctrl_interface.h"
#include "position_sensor.h"
#include "foc_pid.h"
#include "foc_current_loop.h"
#include "foc_filter.h"

extern int   real_speed_buff[10];
int   real_speed=0,fbk_speed_filter=0;

/* motor startup Acc/Dec step */
#define UPDNLIMT_SPEED(Ref,Target,step)	\
				(((Ref)>=(Target))?((((Ref)<=(Target+step))?(Target):(Ref-step))):(Ref+step)) 

/*
*****************************************************************************************
* name  : spd_rad_to_rpm.
*          
* input : 
*     
* return: void
*****************************************************************************************
*/
float spd_rad_to_rpm(float rad)
{
	int rpm_spd = (rad*60*MOTOR_REDUCTION_RATIO*60/ONE_CIRCLE_RADIAN);
	return rpm_spd;
}
/*
*****************************************************************************************
* name  : spd_rpm_to_rad.
*         Fbk
* input : 
*     
* return: void
*****************************************************************************************
*/
float spd_rpm_to_rad(float rpm)
{
	float fbk_speed = KalmanFilterSpeed(rpm,0.00025,2.505,rpm);
	return ((fbk_speed*ONE_CIRCLE_RADIAN)/MOTOR_REDUCTION_RATIO/60/60);
}
/*
*****************************************************************************************
* name  : motor speed loop start up.
*         max speed limit = Rte_get_SpeedLooptargetSpeedMax();
* input : 
*     
* return: void
*****************************************************************************************
*/ 
int motor_speedSoftwareStartUp(int ref)
{
	int getTargetSpeed = Rte_get_TargetSpeedRpm();
	int refTargetSpeed = ref;
	int ret = 0;
	
	/* speed step up/down */
	if( getTargetSpeed > 0 )
	{
		refTargetSpeed = UPDNLIMT_SPEED(refTargetSpeed,getTargetSpeed,500);
	}
	else 
	{
		refTargetSpeed = UPDNLIMT_SPEED(-refTargetSpeed,-getTargetSpeed,500);
		refTargetSpeed = -refTargetSpeed;
	}

	return refTargetSpeed;
}
/*
*****************************************************************************************
* name  : foc_Speed_loop
*
* input : 
*
* return: void
*****************************************************************************************
*/
//int speed_out_filter = 0;
void foc_Speed_loop(void)
{
	g_MotorPIArgs.Speed.Ref = motor_speedSoftwareStartUp(g_MotorPIArgs.Speed.Ref);
	g_MotorPIArgs.Speed.Fbk = fbk_speed_filter;
	
	CNTL_PI_IQ_C( g_MotorPIArgs.Speed );
	
	//speed_out_filter = Low_pass_filter(100,g_MotorPIArgs.Speed.Out,speed_out_filter);
	
	Rte_set_CurrentLoopTargetIq( g_MotorPIArgs.Speed.Out );
}
 
/*
*****************************************************************************************
* name  : foc_check_speed
*
* input : 
*
* return: void
*****************************************************************************************
*/
void foc_check_speed_fbk(void)
{
	static int32_t encoder_state_b = 0;

	real_speed = ((foc_encode1_angle - encoder_state_b)*2000*ENCODE_RANGE)/ENCODE_RANGE;
	encoder_state_b   = foc_encode1_angle;
	fbk_speed_filter = Low_pass_filter(100,real_speed,fbk_speed_filter);
	fbk_speed_filter = Speed_smooth_pass_filter(real_speed_buff,fbk_speed_filter,4);
	//fbk_speed_filter = KalmanFilterSpeed(fbk_speed_filter,0.00025,2.505,fbk_speed_filter);
}

float foc_updata_Fbk_speed_rad(void)
{
	return spd_rpm_to_rad(fbk_speed_filter);
}