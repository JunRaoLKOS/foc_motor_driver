#include "foc_pid.h"

struct motor_pid_config  g_MotorPIArgs;

/*------------------------------------------------------*/
/*------------------------------------------------------*/
void foc_pid_Init(void)
{
	/* current loop id pid */
  CNTL_PI_IQ_C_INIT( g_MotorPIArgs.Id );
	
	g_MotorPIArgs.Id.Kp = 0.002*PID_CONST;
	g_MotorPIArgs.Id.Ki = 0.001*PID_CONST;
	g_MotorPIArgs.Id.Kc = 0.001*PID_CONST;
	g_MotorPIArgs.Id.OutMax = (700*PID_CONST);
	g_MotorPIArgs.Id.OutMin = -(700*PID_CONST);
	g_MotorPIArgs.Id.OUTUiMax = 700*PID_CONST;
	g_MotorPIArgs.Id.OUTUiMin = -700*PID_CONST;
	
	g_MotorPIArgs.Id.Err = 0;
	g_MotorPIArgs.Id.Ref = 0;
	
	/* current loop id pid */
	CNTL_PI_IQ_C_INIT( g_MotorPIArgs.Iq );
	g_MotorPIArgs.Iq.Kp = 0.002*PID_CONST;
	g_MotorPIArgs.Iq.Ki = 9.99999975e-05*PID_CONST;
	g_MotorPIArgs.Iq.Kc = 0.015*PID_CONST;
	g_MotorPIArgs.Iq.OutMax = (700*PID_CONST);
	g_MotorPIArgs.Iq.OutMin = -(700*PID_CONST);
	g_MotorPIArgs.Iq.OUTUiMax = 700*PID_CONST;
	g_MotorPIArgs.Iq.OUTUiMin = -700*PID_CONST;
	
	g_MotorPIArgs.Iq.Err = 0;
	g_MotorPIArgs.Iq.Ref = 0;
	
	/* speed loop pid */
	CNTL_PI_IQ_C_INIT( g_MotorPIArgs.Speed );
	g_MotorPIArgs.Speed.Kp = 1.680*PID_CONST;//0.980,0.012
	g_MotorPIArgs.Speed.Ki = 0.008*PID_CONST;
	g_MotorPIArgs.Speed.Kc = 0.001*PID_CONST;
	g_MotorPIArgs.Speed.OutMax = (60000*PID_CONST);
	g_MotorPIArgs.Speed.OutMin = -(60000*PID_CONST);
	g_MotorPIArgs.Speed.OUTUiMax = 60000*PID_CONST;
	g_MotorPIArgs.Speed.OUTUiMin = -60000*PID_CONST;
	
	g_MotorPIArgs.Speed.Err = 0;
	g_MotorPIArgs.Speed.Ref = 0;
	g_MotorPIArgs.Speed.Fbk = 0;
	
	
	/* position loop pid */
	CNTL_PI_IQ_C_INIT( g_MotorPIArgs.position );
	g_MotorPIArgs.position.Kp = 1.90*PID_CONST;//0.980,0.012
	g_MotorPIArgs.position.Ki = 0.00012*PID_CONST;
	g_MotorPIArgs.position.Kc = 0.01*PID_CONST;
	g_MotorPIArgs.position.OutMax = (60000*PID_CONST);
	g_MotorPIArgs.position.OutMin = -(60000*PID_CONST);
	g_MotorPIArgs.position.OUTUiMax = 600*PID_CONST;
	g_MotorPIArgs.position.OUTUiMin = -600*PID_CONST;
}
void foc_set_speed_pid(uint16_t kp, uint16_t ki)
{
	g_MotorPIArgs.Speed.Kp = kp;
	g_MotorPIArgs.Speed.Ki = ki;
}