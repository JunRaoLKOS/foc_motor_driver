/*
 * pfc_pid.h
 *
 *  Created on: 2018??2??4??
 *      Author: raojun
 */



/**
 * PI control law with programmable output saturation.
 * This macro uses CNTL_PI_IQ_C structures to store coefficients & internal values.
 * The structures should be initialized with the supplied CNTL_PI_IQ_C_INIT macro.
 * Within the structure the Max & Min parameters are the output bounds
 */

#ifndef _FOC_PID_H
#define _FOC_PID_H
/******************************************************************************
 * INCLUDE FILE
 ******************************************************************************/
#include "stdint.h"

/******************************************************************************
 * DEFINE DESCRIPTION
 ******************************************************************************/
#define PID_KC_ENABLE

/******************************************************************************
 * ENUM DESCRIPTION
 ******************************************************************************/

/******************************************************************************
 * STRUCTURE DESCRIPTION
 ******************************************************************************/

//============================================================================
// External References & Function Declarations:
//============================================================================

/******************************************************************************
 * EXTERN CONSTANT DESCRIPTION
 ******************************************************************************/

/******************************************************************************
 * EXTERN VARIABLE DESCRIPTION
 ******************************************************************************/
/******************************************************************************
 * EXTERN FUNCTION PROTOTYPE
 ******************************************************************************/
#define _iq float
#define PID_CONST (1)
#define UPDNLMT(Var,Max,Min)	{(Var)=((Var)>=(Max))?(Max):(Var);(Var)=((Var)<=(Min))?(Min):(Var);}	

/******************************************************************************/
struct PI_IQ_C
{
    _iq  Ref;               // Variable: Referrence
    _iq  Fbk;               // Variable: Feedback
    _iq  Err;               // Variable: Error
    _iq  Kp;                    // Parameter: Proportional gain
    _iq  Ki;                    // Parameter: Integral gain
    _iq  Kc;                    // Parameter: Integral correction gain
    _iq  Up;                // Variable: Proportional output
    _iq  Ui;                    // Variable: Integral output
    _iq  i1;
    _iq  v1;
    _iq  OutPreSat;     // Variable: Pre-saturated output
    _iq  OutMax;        // Parameter: Maximum output
    _iq  OutMin;            // Parameter: Minimum output
		_iq OUTUiMax;
		_iq OUTUiMin;
    _iq  Out;               // Output: PID output
    _iq  SatErr;            // Variable: Saturated difference

};

#define CNTL_PI_IQ_C_INIT(k) {               \
        /* Initialize variables */              \
        k.Ref = 0;                              \
        k.Fbk = 0;                              \
        k.Out = 0;                              \
        k.Kp = 0;                           \
        k.Ki = 0;                           \
        k.OutMax = 0;                              \
        k.OutMin = 0;                              \
        k.Up = 0;                               \
        k.Ui = 0;                       \
        k.Kc = (0);                           \
        k.SatErr = 0;                    \
        k.OutPreSat = 0;}\
/*------------------------------------------------------------------------------
    PI_GRANDO Macro Definition
------------------------------------------------------------------------------*/
#define CNTL_PI_IQ_C(v)  {                                           \
        v.Err       =   v.Ref  - v.Fbk; \
        v.Ui        =   v.Ui + (v.Err*v.Ki)+ (v.SatErr*v.Kc); \
				if(v.Ui > v.OUTUiMax)\
				{\
					v.Ui = v.OUTUiMax;\
				}\
				else if(v.Ui < v.OUTUiMin)\
				{\
					v.Ui = v.OUTUiMin;\
				}\
				v.OutPreSat     =  (v.Err*v.Kp) + v.Ui;   \
				if (v.OutPreSat > v.OutMax) \
				{           \
						v.Out  = v.OutMax/PID_CONST;\
				}           \
				else if (v.OutPreSat  < v.OutMin)\
				{           \
						v.Out = v.OutMin/PID_CONST;\
				}           \
				else        \
				{           \
						v.Out = v.OutPreSat/PID_CONST;\
				}           \
				v.SatErr= v.Out - v.OutPreSat/PID_CONST;}\

		
/*------------------------------------------------------------------------------
    PI_GRANDO Macro Definition
------------------------------------------------------------------------------*/
#define CNTL_PI_IQ_C1(v)  {  \
        v.Err       =   v.Ref  - v.Fbk; \
        v.Ui        =   v.Ui + (v.Err*v.Ki)+ (v.SatErr*v.Kc); \
				if(v.Err == 0)\
				{\
					v.Ui = 0;\
				}\
				else if(v.Ui > v.OUTUiMax)\
				{\
					v.Ui = v.OUTUiMax;\
				}\
				else if(v.Ui < v.OUTUiMin)\
				{\
					v.Ui = v.OUTUiMin;\
				}\
        v.OutPreSat     =  (v.Err*v.Kp) + v.Ui;   \
        if (v.OutPreSat > v.OutMax) \
        {           \
            v.Out  = v.OutMax/PID_CONST;\
        }           \
        else if (v.OutPreSat  < v.OutMin)\
        {           \
            v.Out = v.OutMin/PID_CONST;\
        }           \
        else        \
        {           \
            v.Out = v.OutPreSat/PID_CONST;\
        }           \
        v.SatErr= v.Out - v.OutPreSat/PID_CONST;}\

/*------------------------------------------------------------------------------
    PI_GRANDO Macro Definition
------------------------------------------------------------------------------*/
struct motor_pid_config
{
	struct PI_IQ_C Id;
	struct PI_IQ_C Iq;
	struct PI_IQ_C Speed; 
	struct PI_IQ_C position; 	
};
extern struct motor_pid_config  g_MotorPIArgs;

/*------------------------------------------------------------------------------
    PI_GRANDO Macro Definition
------------------------------------------------------------------------------*/

extern void foc_pid_Init(void);
 
#endif

