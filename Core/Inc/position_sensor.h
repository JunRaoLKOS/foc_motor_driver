/*
 * position_sensor.h
 *
 *  Created on: Jul 26, 2020
 *      Author: Ben
 */

#ifndef INC_POSITION_SENSOR_H_
#define INC_POSITION_SENSOR_H_


//#include "structs.h"
#include "spi.h"
#include <stdint.h>
#define READ_ANGLECOM 0x3FFF
#define READ_NOP      0x0000
#define READ_ERRFL    0x4001

#define N_POS_SAMPLES 20		// Number of position samples to store.  should put this somewhere else...
#define N_LUT 128
#pragma anon_unions
typedef struct{
	union{
		uint8_t spi_tx_buff[2];
		uint16_t spi_tx_word;
	};
	union{
		uint8_t spi_rx_buff[2];
		uint16_t spi_rx_word;
	};
	float angle_singleturn, old_angle, angle_multiturn[N_POS_SAMPLES], elec_angle, velocity, elec_velocity, ppairs, vel2;
	float output_angle_multiturn,turns;
	int raw, count, old_count;
	int count_buff[N_POS_SAMPLES];
	int m_zero, e_zero;
	int offset_lut[N_LUT];
	uint8_t first_sample;
} EncoderStruct;


void ps_warmup(EncoderStruct * encoder, int n);
void ps_warmup2(EncoderStruct * encoder, int n);
void ps_sample2(EncoderStruct * encoder, float dt);
void ps_sample(EncoderStruct * encoder, float dt);
void ps_print(EncoderStruct * encoder, int dt_ms);
void Track(void);
extern uint16_t AS5047_read1(uint16_t add);
extern uint16_t get_encode_val(uint8_t code_id);

#endif /* INC_POSITION_SENSOR_H_ */
