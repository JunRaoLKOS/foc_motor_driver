#ifndef _RTE_CTRLINTERFACE_H
#define _RTE_CTRLINTERFACE_H

#include <stdint.h>
#include <math.h>

#define MOTOR_REDUCTION_RATIO         (15.2)
#define MOTOR_POLE_PAIRS              (10)
#define ENCODE_RANGE 				          (16384)
#define ONE_CIRCLE_RADIAN             (6.28318548203f) 


typedef struct
{
	float KP;
	float KD;
	
	int target_Id;
	int target_Iq;
	
	int target_speed_Rpm;
	int target_position_Encode;
	float ref_spd_rad;
	
	uint32_t encodeOffset;
	
	uint16_t T_Acc;
	uint16_t T_Dec;
 
	uint16_t OVTemp;/* 温度保护阈值 */
  uint16_t CanTimeOut;
	
	uint32_t StallCurrent;
	uint16_t BlockTimeOut;
	
	uint8_t States;
	
}Rte_Ctrl_Interface;

extern void Rte_set_DefaultInit(void);

/* set */
extern void Rte_set_KP(float kp);
extern void Rte_set_KD(float kd);
extern void Rte_set_CurrentLoopTargetId( int id );
extern void Rte_set_CurrentLoopTargetIq( int iq );
extern void Rte_set_TargetSpeedRpm( int speed );
extern void Rte_set_T_Acc( uint16_t acc );
extern void Rte_set_T_Dec( uint16_t dec );
extern void Rte_set_MotorEncodeOffset( uint16_t offset );
extern void Rte_set_MotorPosition( uint32_t position );
extern void Rte_set_MotorActualSpeed(int speed);
extern void Rte_set_MotorOff(void);
extern void Rte_set_MotorOn(void);
extern void Rte_set_MotorResetInit(void);
extern void Rte_set_MotorHoldCurrent( int cur );
extern void Rte_set_MotorRunning(void);
extern void Rte_set_MotorError(void);
extern void Rte_set_MotorHolding(void);
extern void Rte_set_MotorHoldCurrent(int hold);
extern void Rte_set_MotorBlockCurrent(uint32_t cur);
extern void Rte_set_MotorOVTemp(uint16_t ovtemp);
extern void Rte_set_CommTimeOut(uint16_t timeCnt);
extern void Rte_set_Block_TimeOut(uint16_t timeCnt);
extern void Rte_set_PositionEncode( int pos );

extern void Rte_set_TargetSpeedRad(float spd_rad);
extern void Rte_set_TargetPositionRad(float pos_rad );


/* get */
extern float Rte_get_KP(void);
extern float Rte_get_KD(void);
extern int Rte_get_CurrentLoopTargetId( void );
extern int Rte_get_CurrentLoopTargetIq( void );
extern int Rte_get_CurrentLoopFbkIq( void );
extern int Rte_get_CurrentLoopFbkId( void );
extern int Rte_get_TargetSpeedRpm( void ); 
extern int Rte_get_MotorActualSpeed( void );
extern int Rte_get_MotorHoldCurrent(void);
extern uint8_t Rte_get_MotorError(void);
extern uint8_t Rte_get_MotorStates( void );
extern uint16_t Rte_get_T_Acc( void );
extern uint16_t Rte_get_T_Dec( void );
extern uint16_t Rte_get_MotorEncodeAngle( void );
extern uint16_t Rte_get_MotorEncodeOffset( void );
extern uint32_t Rte_get_MotorPosition( void );
extern uint32_t Rte_get_MotorBlockCurrent(void);

extern uint16_t Rte_get_MotorOVTemp(void);
extern uint16_t Rte_get_CommTimeOut(void);
extern uint16_t Rte_get_Block_TimeOut(void);
extern int Rte_get_PositonEncode( void );

extern float Rte_get_FbkPositionRad( void );
extern float Rte_get_FbkSpeedRad( void );
extern float Rte_get_TargetPositionRad(void);
extern float Rte_get_TargetSpeedRad(void);

#endif
