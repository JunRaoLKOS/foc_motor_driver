#include "foc_current_loop.h"
#include "foc_sin_table.h"
#include "foc_svpwm.h"
#include "foc_pid.h"
#include "foc_filter.h"
#include "adc.h"
#include "hw_config.h"

#define ADC_RS   (1)   /* 采样电阻5mR  */
#define Amp_Gain (20)  /* 运放放大倍数 */

/****************************************************************************************/
#define Vd_out_limit  (50)
#define Vq_out_limit  (50)

#define Clarke_transform_K  2/3

foc_Iph_AB Iph_Calib;

/*
*****************************************************************************************
* name  : get_run_adc_current
*
* input : 
*
* return: void
*****************************************************************************************
*/

int get_adc_current(unsigned int adcValue)
{ 
   int current = ((int)adcValue-2048)*806/Amp_Gain/ADC_RS;
   return current;
}
/*
*****************************************************************************************
* name  : current loop.
*
* input : 
*
* return: void
*****************************************************************************************
*/	
void Current_loop(int Target_Id, int Target_Iq,int angle) 
{
	int adc_pha = get_adc_current(HAL_ADCEx_InjectedGetValue(&hadc1, 1)) - Iph_Calib.phA;
	int adc_phb = get_adc_current(HAL_ADCEx_InjectedGetValue(&hadc2, 1)) - Iph_Calib.phB;
	
	int Ia_filter = 0;
	int Ib_filter = 0;
	Ia_filter = Low_pass_filter(300 , -adc_pha, Ia_filter);/* if current enter IA:+ , else IA:-*/
	Ib_filter = Low_pass_filter(300 , adc_phb , Ib_filter);

	//Clarke transform
	/* Ialpha = Ia -0.5Ib -0.5Ic = Clarke_transform_K * 3/2 *Ia = Ia; */
	int Ialpha = Ia_filter; 
  int Ibeta = (1000 * (Ia_filter - Ib_filter))/1732;

  //Park transform
  int cos = arm_cos_f32(angle);
  int sin = arm_sin_f32(angle);
  int Id = (cos*Ialpha + sin*Ibeta)/SIN_MAX_VALUE;
  int Iq = (cos*Ibeta  - sin*Ialpha)/SIN_MAX_VALUE;

	/* dq pid */
	g_MotorPIArgs.Id.Fbk = Id;
	g_MotorPIArgs.Id.Ref = Target_Id;
		
	g_MotorPIArgs.Iq.Fbk = Iq;
	g_MotorPIArgs.Iq.Ref = Target_Iq;
		
	CNTL_PI_IQ_C( g_MotorPIArgs.Id );
	CNTL_PI_IQ_C( g_MotorPIArgs.Iq );
		
	int Vd_filter = 0;
	int Vq_filter = 0;
	/* dq filter */
	Vd_filter = Low_pass_filter(300 , g_MotorPIArgs.Id.Out , Vd_filter);
  Vq_filter = Low_pass_filter(300 , g_MotorPIArgs.Iq.Out , Vq_filter);
	
	/* dq out max , min limit */
	UPDNLMT( g_MotorPIArgs.Id.Out ,Vd_out_limit, -Vd_out_limit);
	UPDNLMT( g_MotorPIArgs.Iq.Out ,Vq_out_limit, -Vq_out_limit);

  // Inverse park transform
  int mod_alpha = (cos*g_MotorPIArgs.Id.Out - sin*g_MotorPIArgs.Iq.Out)/SIN_MAX_VALUE;
  int mod_beta  = (cos*g_MotorPIArgs.Iq.Out + sin*g_MotorPIArgs.Id.Out)/SIN_MAX_VALUE;

	mod_alpha = mod_alpha;
	mod_beta  = mod_beta;
		
  //SVM
  Svpwm_calculation( mod_alpha, mod_beta);
}
