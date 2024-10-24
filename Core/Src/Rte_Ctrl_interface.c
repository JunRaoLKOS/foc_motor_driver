#include "Rte_Ctrl_interface.h"
#include "foc_speed_loop.h"
#include "foc_position_loop.h"
#include "foc_pid.h"
#include "foc_states.h"


Rte_Ctrl_Interface Rte_Ctrl;

/* 
 Rte_set_DefaultInit.
*/
void Rte_set_DefaultInit(void)
{
	foc_pid_Init();
	
	Rte_set_KP(28.5);
	Rte_set_KD(3.1);
	
	Rte_set_TargetSpeedRad(0.628);
	Rte_set_TargetPositionRad(0);
	
	Rte_set_CurrentLoopTargetId(0);
	Rte_set_CurrentLoopTargetIq(0);
	Rte_set_TargetSpeedRpm(0);
	
	Rte_set_PositionEncode(180);
	
	Rte_set_MotorEncodeOffset(108);
	
	/* max block current */
	Rte_set_MotorBlockCurrent(500);
	Rte_set_Block_TimeOut(10);
}

/*******************************************************************/
/* 
 set Rte_set_KP.
*/
void Rte_set_KP(float kp)
{
	Rte_Ctrl.KP = kp;
}
/* 
 set Rte_set_KD.
*/
void Rte_set_KD(float kd)
{
	Rte_Ctrl.KD = kd;
}
/* 
 get Rte_get_KP.
*/
float Rte_get_KP(void)
{
	return Rte_Ctrl.KP;
}
/* 
 get Rte_get_KD.
*/
float Rte_get_KD(void)
{
	return Rte_Ctrl.KD;
}
 
/* 
 Rte_set_TargetSpeedRad
*/
void Rte_set_TargetSpeedRad(float spd_rad)
{
	Rte_Ctrl.ref_spd_rad = spd_rad;
	Rte_set_TargetSpeedRpm(spd_rad_to_rpm(spd_rad));
}
/* 
 Rte_set_TargetPositionRad
*/
void Rte_set_TargetPositionRad( float pos_rad )
{
	Rte_set_PositionEncode(pos_rad_to_encode(pos_rad));
}
/* 
 Rte_set_TargetSpeedRad
*/
float Rte_get_TargetSpeedRad(void)
{	
	return Rte_Ctrl.ref_spd_rad; 
}
/* 
 Rte_get_TargetPositionRad
*/
float Rte_get_TargetPositionRad(void)
{
	return pos_encode_to_rad(Rte_get_PositonEncode());
}
/* 
 Rte_get_FbkPositionRad
*/
float Rte_get_FbkPositionRad( void )
{
	extern int foc_position_fbk;
	
	return pos_encode_to_rad(foc_position_fbk);
}
/* 
 Rte_get_FbkSpeedRad
*/
float Rte_get_FbkSpeedRad( void )
{
	return foc_updata_Fbk_speed_rad();
}

/*******************************************************************/

/* 
 set Rte_set_T_Acc.
*/
void Rte_set_T_Acc( uint16_t acc )
{
  Rte_Ctrl.T_Acc = acc;
}
/* 
 set Rte_set_T_Dec.
*/
void Rte_set_T_Dec( uint16_t dec )
{
  Rte_Ctrl.T_Dec = dec;
}

uint16_t Rte_get_T_Acc( void )
{
	return Rte_Ctrl.T_Acc;
}

uint16_t Rte_get_T_Dec( void )
{
	return Rte_Ctrl.T_Dec;
}
/* 
 set current loop target Id.
*/
void Rte_set_CurrentLoopTargetId( int id )
{
	Rte_Ctrl.target_Id = id;
}
/* 
 set current loop target Iq.
*/
void Rte_set_CurrentLoopTargetIq( int iq )
{
	Rte_Ctrl.target_Iq = iq;
}
/* 
 get current loop target Id.
*/
int Rte_get_CurrentLoopTargetId( void )
{
	return Rte_Ctrl.target_Id;
}
/* 
 get Rte_get_CurrentLoop Fbk Id.
*/
int Rte_get_CurrentLoopFbkId( void )
{
	return g_MotorPIArgs.Id.Fbk;
}
/* 
 get current loop target Iq.
*/
int Rte_get_CurrentLoopTargetIq( void )
{
	return Rte_Ctrl.target_Iq;
}
/* 
 get Rte_get_CurrentLoop Fbk Iq.
*/
int Rte_get_CurrentLoopFbkIq( void )
{
	return g_MotorPIArgs.Iq.Fbk;
}

/* 
 set speed loop target speed.
*/ 
void Rte_set_TargetSpeedRpm( int speed )
{
  Rte_Ctrl.target_speed_Rpm = speed;
}
/* 
 get speed loop target speed.
*/
int Rte_get_TargetSpeedRpm( void )
{
	return Rte_Ctrl.target_speed_Rpm;
}
/* 
 set target_position_Encode loop target speed.
*/
void Rte_set_PositionEncode( int pos )
{
	Rte_Ctrl.target_position_Encode = pos;
}
/* 
 get target_position_Encode loop target speed.
*/
int Rte_get_PositonEncode( void )
{
	return Rte_Ctrl.target_position_Encode;
}
/* 
 Rte_set_MotorBlockCurrent
*/
void Rte_set_MotorBlockCurrent(uint32_t cur)
{
	 Rte_Ctrl.StallCurrent = cur;
}

/* 
 Rte_get_MotorBlockCurrent
*/
uint32_t Rte_get_MotorBlockCurrent(void)
{
	 return Rte_Ctrl.StallCurrent;
}
/* 
 Rte_set_Block_TimeOut
*/
void Rte_set_Block_TimeOut(uint16_t timeCnt)
{
	Rte_Ctrl.BlockTimeOut = timeCnt;
}
/* 
 Rte_get_Block_TimeOut
*/
uint16_t Rte_get_Block_TimeOut(void)
{
	return Rte_Ctrl.BlockTimeOut;
}

/* 
 set Rte_set_EncodeOffset.
*/
void Rte_set_MotorEncodeOffset( uint16_t offset )
{
	Rte_Ctrl.encodeOffset = offset;
}

/* 
 get Rte_get_EncodeOffset.
*/
uint16_t Rte_get_MotorEncodeOffset(void)
{
	return Rte_Ctrl.encodeOffset;
}
/* 
 set Rte_set_MotorOff.
*/
void Rte_set_MotorOff(void)
{
	foc_pid_Init();
	Rte_Ctrl.States = FOC_OFF;
}
/* 
 set Rte_set_MotorOn.
*/
void Rte_set_MotorOn(void)
{
	foc_pid_Init();
	Rte_Ctrl.States = FOC_SPEED_MODE;
}
	