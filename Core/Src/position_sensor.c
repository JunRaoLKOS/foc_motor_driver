/*
 * position_sensor.c
 *
 *  Created on: Jul 26, 2020
 *      Author: Ben
 */
#include <stdio.h>
#include <string.h>
#include "position_sensor.h"
#include "math_ops.h"
#include "hw_config.h"
#include "user_config.h"
#include "Rte_Ctrl_interface.h"

/**
  * @brief  AS5047P读取原始数据,不使用CRC校验，直接丢弃校验码
  *    data   [15]   [14]   [13:0]
  *           CRC 1  read 1  ANGLECOM 3FF measured angle with dynamic angle error compensation
  * @retval 返回读取的角度数据
  */
// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle)
{
  float a = fmod(angle, 2*PI_F);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*PI_F);
}

// 计算奇偶函数
uint16_t Parity_bit_Calculate(uint16_t data_2_cal)
{
	uint16_t parity_bit_value=0;
	while(data_2_cal != 0)
	{
		parity_bit_value ^= data_2_cal; 
		data_2_cal >>=1;
	}
	return (parity_bit_value & 0x1); 
}
//SPI发送读取函数
uint16_t SPI2_ReadWrite_OneByte(uint16_t _txdata)
{

	HAL_GPIO_WritePin(ENC_CS2, GPIO_PIN_RESET ); 	// CS low	
	uint16_t rxdata;
	if(HAL_SPI_TransmitReceive(&hspi2,(uint8_t *)&_txdata,(uint8_t *)&rxdata,1,1000) !=HAL_OK)
	rxdata=0;
  HAL_GPIO_WritePin(ENC_CS2, GPIO_PIN_SET ); 	// CS high
	return rxdata;
}
//SPI发送读取函数
uint16_t SPI3_ReadWrite_OneByte(uint16_t _txdata)
{

	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low	
	uint16_t rxdata;
	if(HAL_SPI_TransmitReceive(&hspi3,(uint8_t *)&_txdata,(uint8_t *)&rxdata,1,1000) !=HAL_OK)
	rxdata=0;
  HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
	return rxdata;
}
uint16_t AS5047_read2(uint16_t add)
{
	uint16_t data;
	add |= 0x4000;	//读指令 bit14 置
	if(Parity_bit_Calculate(add)==1) add=add|0x8000; //如果前15位 1的个数位偶数，则Bit15 置1
	SPI2_ReadWrite_OneByte(add);		//发送一条指令，不管读回的数据
	data=SPI2_ReadWrite_OneByte(READ_NOP|0x4000); //发送一条空指令，读取上一次指令返回的数据。
	data &=0x3fff;
	return data;
}

uint16_t AS5047_read1(uint16_t add)
{
	uint16_t data;
	add |= 0x4000;	//读指令 bit14 置
	if(Parity_bit_Calculate(add)==1) add=add|0x8000; //如果前15位 1的个数位偶数，则Bit15 置1
	SPI3_ReadWrite_OneByte(add);		//发送一条指令，不管读回的数据
	data=SPI3_ReadWrite_OneByte(READ_NOP|0x4000); //发送一条空指令，读取上一次指令返回的数据。
	data &=0x3fff;
	
	return data;
}

//AS5047-1
void ps_warmup(EncoderStruct * encoder, int n){
	/* Hall position sensors noisy on startup.  Take a bunch of samples to clear this data */
	for(int i = 0; i<n; i++){
//		encoder->spi_tx_word = 0x0000;
//		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
//		HAL_SPI_TransmitReceive(&ENC_SPI, (uint8_t*)encoder->spi_tx_buff, (uint8_t *)encoder->spi_rx_buff, 1, 100);
//		while( ENC_SPI.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
//		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
			encoder->raw =AS5047_read1(READ_ANGLECOM);
			encoder->angle_singleturn = encoder->raw * 360.f / (float) 16384.0;
			printf("angle_singleturn: %3f\r\n", encoder->angle_singleturn);
	}
}
//AS50471-2
void ps_warmup2(EncoderStruct * encoder, int n){
	/* Hall position sensors noisy on startup.  Take a bunch of samples to clear this data */
	for(int i = 0; i<n; i++){
//		encoder->spi_tx_word = 0x0000;
//		HAL_GPIO_WritePin(ENC_CS2, GPIO_PIN_RESET ); 	// CS low
//		HAL_SPI_TransmitReceive(&ENC_SPI2, (uint8_t*)encoder->spi_tx_buff, (uint8_t *)encoder->spi_rx_buff, 1, 100);
//		while( ENC_SPI2.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
//		HAL_GPIO_WritePin(ENC_CS2, GPIO_PIN_SET ); 	// CS high
		
		encoder->raw =AS5047_read2(READ_ANGLECOM);
		encoder->angle_singleturn = encoder->raw * 360.f / (float) 16384.0;
		printf("angle_singleturn2: %3f\r\n", encoder->angle_singleturn);
	}
}
/* hight encode1 */
void ps_sample(EncoderStruct * encoder, float dt){
	/* updates EncoderStruct encoder with the latest sample
	 * after elapsed time dt */

	/* Shift around previous samples */
	encoder->old_angle = encoder->angle_singleturn;
	for(int i = N_POS_SAMPLES-1; i>0; i--){encoder->angle_multiturn[i] = encoder->angle_multiturn[i-1];}
	//for(int i = N_POS_SAMPLES-1; i>0; i--){encoder->count_buff[i] = encoder->count_buff[i-1];}
	//memmove(&encoder->angle_multiturn[1], &encoder->angle_multiturn[0], (N_POS_SAMPLES-1)*sizeof(float)); // this is much slower for some reason

	/* SPI read/write */
//	encoder->spi_tx_word = ENC_READ_WORD;
//	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
//	HAL_SPI_TransmitReceive(&ENC_SPI, (uint8_t*)encoder->spi_tx_buff, (uint8_t *)encoder->spi_rx_buff, 1, 100);
//	while( ENC_SPI.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
//	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
//	encoder->raw = encoder ->spi_rx_word;
	encoder->raw =AS5047_read1(READ_ANGLECOM);
/* Linearization */
	int off_1 = encoder->offset_lut[(encoder->raw)>>9];				// lookup table lower entry
	int off_2 = encoder->offset_lut[((encoder->raw>>9)+1)%128];		// lookup table higher entry
	int off_interp = off_1 + ((off_2 - off_1)*(encoder->raw - ((encoder->raw>>9)<<9))>>9);     // Interpolate between lookup table entries
	encoder->count = encoder->raw + off_interp;


	/* Real angles in radians */
	encoder->angle_singleturn = ((float)(encoder->count-M_ZERO))/((float)ENC_CPR);
	int int_angle = encoder->angle_singleturn;
	encoder->angle_singleturn = TWO_PI_F*(encoder->angle_singleturn - (float)int_angle);
	//encoder->angle_singleturn = TWO_PI_F*fmodf(((float)(encoder->count-M_ZERO))/((float)ENC_CPR), 1.0f);
	encoder->angle_singleturn = encoder->angle_singleturn<0 ? encoder->angle_singleturn + TWO_PI_F : encoder->angle_singleturn;

	encoder->elec_angle = (encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR);
	int_angle = (int)encoder->elec_angle;
	//encoder->elec_angle = TWO_PI_F*(encoder->elec_angle - (float)int_angle);
	encoder->elec_angle = TWO_PI_F*fmodf((encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR), 1.0f);
	encoder->elec_angle = encoder->elec_angle<0 ? encoder->elec_angle + TWO_PI_F : encoder->elec_angle;	// Add 2*pi to negative numbers
//	/* Linearization */
////	int off_1 = encoder->offset_lut[(encoder->raw)>>9];				// lookup table lower entry
////	int off_2 = encoder->offset_lut[((encoder->raw>>9)+1)%128];		// lookup table higher entry
////	int off_interp = off_1 + ((off_2 - off_1)*(encoder->raw - ((encoder->raw>>9)<<9))>>9);     // Interpolate between lookup table entries
////	encoder->count = encoder->raw + off_interp;
//	encoder->count = encoder->raw ;


//	/* Real angles in radians */
//	encoder->angle_singleturn = ((float)(encoder->count-M_ZERO))/((float)ENC_CPR);
//	//int int_angle = encoder->angle_singleturn;
//	int int_angle = 0;
//	encoder->angle_singleturn = TWO_PI_F*(encoder->angle_singleturn - (float)int_angle);
//	//encoder->angle_singleturn = TWO_PI_F*fmodf(((float)(encoder->count-M_ZERO))/((float)ENC_CPR), 1.0f);
//	encoder->angle_singleturn = encoder->angle_singleturn<0 ? encoder->angle_singleturn + TWO_PI_F : encoder->angle_singleturn;

//	encoder->elec_angle = (encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR);
//	//int_angle = (int)encoder->elec_angle;
//	int_angle = (int)0;
//	//encoder->elec_angle = TWO_PI_F*(encoder->elec_angle - (float)int_angle);
//	encoder->elec_angle = (encoder->ppairs*encoder->angle_singleturn);
//	//encoder->elec_angle = encoder->elec_angle<0 ? encoder->elec_angle + TWO_PI_F : encoder->elec_angle;	// Add 2*pi to negative numbers
//	encoder->elec_angle = _normalizeAngle(encoder->elec_angle);
	/* Rollover */
	float rollover = 0;
	float angle_diff = encoder->angle_singleturn - encoder->old_angle;
//	if(fabs(angle_diff) > (0.8f*2.0f*PI_F) ) rollover += ( angle_diff > 0 ) ? -1 : 1;
	if(angle_diff > PI_F){rollover = -1;}
	else if(angle_diff < -PI_F){rollover = 1;}
	encoder->turns += rollover;
	if(!encoder->first_sample){
		encoder->turns = 0;
//		if(encoder->angle_singleturn > PI_OVER_2_F){encoder->turns = -1;}
//		else if(encoder->angle_singleturn < -PI_OVER_2_F){encoder->turns = 1;}
		encoder->first_sample = 1;
	}



	/* Multi-turn position */
	encoder->angle_multiturn[0] = encoder->angle_singleturn + TWO_PI_F*(float)encoder->turns;

	/* Velocity */
	/*
	// Attempt at a moving least squares.  Wasn't any better
		float m = (float)N_POS_SAMPLES;
		float w = 1.0f/m;
		float q = 12.0f/(m*m*m - m);
		float c1 = 0.0f;
		float ibar = (m - 1.0f)/2.0f;
		for(int i = 0; i<N_POS_SAMPLES; i++){
			c1 += encoder->angle_multiturn[i]*q*(i - ibar);
		}
		encoder->vel2 = -c1/dt;
*/
	//encoder->velocity = vel2
	encoder->velocity = (encoder->angle_multiturn[0] - encoder->angle_multiturn[N_POS_SAMPLES-1])/(dt*(float)(N_POS_SAMPLES-1));
	encoder->elec_velocity = encoder->ppairs*encoder->velocity;

}

/* low encode2 */
void ps_sample2(EncoderStruct * encoder, float dt){
	/* updates EncoderStruct encoder with the latest sample
	 * after elapsed time dt */

	/* Shift around previous samples */
	encoder->old_angle = encoder->angle_singleturn;
	for(int i = N_POS_SAMPLES-1; i>0; i--){encoder->angle_multiturn[i] = encoder->angle_multiturn[i-1];}
	//for(int i = N_POS_SAMPLES-1; i>0; i--){encoder->count_buff[i] = encoder->count_buff[i-1];}
	//memmove(&encoder->angle_multiturn[1], &encoder->angle_multiturn[0], (N_POS_SAMPLES-1)*sizeof(float)); // this is much slower for some reason

	/* SPI read/write */
//	encoder->spi_tx_word = ENC_READ_WORD;
//	HAL_GPIO_WritePin(ENC_CS2, GPIO_PIN_RESET ); 	// CS low
//	HAL_SPI_TransmitReceive(&ENC_SPI2, (uint8_t*)encoder->spi_tx_buff, (uint8_t *)encoder->spi_rx_buff, 1, 100);
//	while( ENC_SPI2.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
//	HAL_GPIO_WritePin(ENC_CS2, GPIO_PIN_SET ); 	// CS high
//	encoder->raw = encoder ->spi_rx_word;
		encoder->raw =AS5047_read2(READ_ANGLECOM);

	/* Linearization */
	int off_1 = encoder->offset_lut[(encoder->raw)>>9];				// lookup table lower entry
	int off_2 = encoder->offset_lut[((encoder->raw>>9)+1)%128];		// lookup table higher entry
	int off_interp = off_1 + ((off_2 - off_1)*(encoder->raw - ((encoder->raw>>9)<<9))>>9);     // Interpolate between lookup table entries
	encoder->count = encoder->raw + off_interp;


	/* Real angles in radians */
	encoder->angle_singleturn = ((float)(encoder->count-M_ZERO))/((float)ENC_CPR);
	int int_angle = encoder->angle_singleturn;
	encoder->angle_singleturn = TWO_PI_F*(encoder->angle_singleturn - (float)int_angle);
	//encoder->angle_singleturn = TWO_PI_F*fmodf(((float)(encoder->count-M_ZERO))/((float)ENC_CPR), 1.0f);
	encoder->angle_singleturn = encoder->angle_singleturn<0 ? encoder->angle_singleturn + TWO_PI_F : encoder->angle_singleturn;

	encoder->elec_angle = (encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR);
	int_angle = (int)encoder->elec_angle;
	//encoder->elec_angle = TWO_PI_F*(encoder->elec_angle - (float)int_angle);
	encoder->elec_angle = TWO_PI_F*fmodf((encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR), 1.0f);
	encoder->elec_angle = encoder->elec_angle<0 ? encoder->elec_angle + TWO_PI_F : encoder->elec_angle;	// Add 2*pi to negative numbers
	/* Rollover */
	float rollover = 0;
	float angle_diff = encoder->angle_singleturn - encoder->old_angle;
	if(angle_diff > PI_F){rollover = -1;}
	else if(angle_diff < -PI_F){rollover = 1;}
	encoder->turns += rollover;
	if(!encoder->first_sample){
		encoder->turns = 0;
//		if(encoder->angle_singleturn > PI_OVER_2_F){encoder->turns = -1;}
//		else if(encoder->angle_singleturn < -PI_OVER_2_F){encoder->turns = 1;}
		encoder->first_sample = 1;
	}



	/* Multi-turn position */
	encoder->angle_multiturn[0] = encoder->angle_singleturn + TWO_PI_F*(float)encoder->turns;

	/* Velocity */
	/*
	// Attempt at a moving least squares.  Wasn't any better
		float m = (float)N_POS_SAMPLES;
		float w = 1.0f/m;
		float q = 12.0f/(m*m*m - m);
		float c1 = 0.0f;
		float ibar = (m - 1.0f)/2.0f;
		for(int i = 0; i<N_POS_SAMPLES; i++){
			c1 += encoder->angle_multiturn[i]*q*(i - ibar);
		}
		encoder->vel2 = -c1/dt;
*/
	//encoder->velocity = vel2
	encoder->velocity = (encoder->angle_multiturn[0] - encoder->angle_multiturn[N_POS_SAMPLES-1])/(dt*(float)(N_POS_SAMPLES-1));
	encoder->elec_velocity = encoder->ppairs*encoder->velocity;

}

void ps_print(EncoderStruct * encoder, int dt_ms){
	printf("Raw: %d", encoder->raw);
	printf("   Linearized Count: %d", encoder->count);
	printf("   Single Turn: %f", encoder->angle_singleturn);
	printf("   Multiturn: %f", encoder->angle_multiturn[0]);
	printf("   Electrical: %f", encoder->elec_angle);
	printf("   Turns:  %f\r\n", encoder->turns);
	//HAL_Delay(dt_ms);
}

float angle_prev=0;
int full_rotations=0; 		// full rotation tracking;
float angle_d;				//GetAngle_Without_Track()的返回值
float angle_cd;				//GetAngle()的返回值
float electricalAngle;
//得到弧度制的角度，范围在0-6.28
float GetAngle_Without_Track(void)
{
	int16_t in_angle;
	in_angle =AS5047_read1(READ_ANGLECOM);
 
	angle_d = (float)in_angle * (TWO_PI_F) / 16384;
//angle_d为弧度制，范围在0-6.28
	return angle_d;
}
 
 
//得到弧度制的带圈数角度
float GetAngle(void)
{
	float val = angle_d;
	float d_angle = val - angle_prev;
	//计算旋转的总圈数
	//通过判断角度变化是否大于80%的一圈(0.8f*6.28318530718f)来判断是否发生了溢出，如果发生了，则将full_rotations增加1（如果d_angle小于0）或减少1（如果d_angle大于0）。
	if(fabs(d_angle) > (0.8f*2.0f*PI_F) ) full_rotations += ( d_angle > 0 ) ? -1 : 1;
	angle_prev = val;

	angle_cd = full_rotations * (2.0f*PI_F) + angle_prev;
	return angle_cd;
}
 
float voltage_limit=12.6;
float voltage_power_supply=12.6;
float shaft_angle=0,open_loop_timestamp=0;
float zero_electric_angle=0,Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0,dc_a=0,dc_b=0,dc_c=0;
int PP=10,DIR=1;

 
float _electricalAngle(void)
{
  return  _normalizeAngle((float)(DIR *  PP) * GetAngle_Without_Track()-zero_electric_angle);
}
 
void Track(void)
{
	GetAngle_Without_Track();
	GetAngle();
	electricalAngle = _electricalAngle();
}
 
 
uint16_t get_encode_val(uint8_t code_id)
{
	uint16_t data = 0;
	
	if(code_id == 1)
	{
		data = AS5047_read1(READ_ANGLECOM);
	}
	else if(code_id == 2)
	{
		data = AS5047_read2(READ_ANGLECOM);
	}
	return data;
}


