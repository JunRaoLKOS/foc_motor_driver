/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "hw_config.h"
#include "user_config.h"
#include "math_ops.h"
#include "position_sensor.h"
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void can_rx_init(CANRxMessage *msg){
	msg->filter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 	// set fifo assignment
	msg->filter.FilterIdHigh=CAN_ID<<5; 				// CAN ID
	msg->filter.FilterIdLow=0x0;
	msg->filter.FilterMaskIdHigh=0x0;
	msg->filter.FilterMaskIdLow=0;
	msg->filter.FilterMode = CAN_FILTERMODE_IDMASK;
	msg->filter.FilterScale=CAN_FILTERSCALE_32BIT;
	msg->filter.FilterActivation=ENABLE;
	HAL_CAN_ConfigFilter(&CAN_H, &msg->filter);
}

void can_tx_init(CANTxMessage *msg){
	msg->tx_header.DLC = 7; 			// message size of 7 byte
	msg->tx_header.IDE=CAN_ID_STD; 		// set identifier to standard
	msg->tx_header.RTR=CAN_RTR_DATA; 	// set data type to remote transmission request?
	msg->tx_header.StdId = CAN_MASTER;  // recipient CAN ID
}
/*
/ CAN Reply Packet Structure ///
/ 16 bit position, between -4*pi and 4*pi
/ 12 bit velocity, between -30 and + 30 rad/s
/ 12 bit current, between -40 and 40;
/ CAN Packet is 5 8-bit words
/ Formatted as follows.  For each quantity, bit 0 is LSB
/ 0: [position[15-8]]
/ 1: [position[7-0]]
/ 2: [velocity[11-4]]
/ 3: [velocity[3-0], current[11-8]]
/ 4: [current[7-0]]
*/
void pack_reply(CANTxMessage *msg, uint8_t id, float p, float v, float t, float vb){
    int p_int = float_to_uint(p, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
    int t_int = float_to_uint(t, -(I_MAX+SENSE_BUFFER)*KT*GR, (I_MAX+SENSE_BUFFER)*KT*GR, 12);
    int vb_int = float_to_uint(vb, VB_MIN, VB_MAX, 8);
    msg->data[0] = id;
    msg->data[1] = p_int>>8;
    msg->data[2] = p_int&0xFF;
    msg->data[3] = v_int>>4;
    msg->data[4] = ((v_int&0xF)<<4) + (t_int>>8);
    msg->data[5] = t_int&0xFF;
    msg->data[6] = vb_int;
 }

 
void Setpack_reply(CANTxMessage *msg, uint16_t id,uint8_t setcmd ,uint8_t ack)
{

	msg->id = 0x7ff;
	msg->data[0] = (uint8_t)id>>8;
	msg->data[1] = (uint8_t)id>>0;
	msg->data[2] = (uint8_t)ack;
	msg->data[3] = (uint8_t)setcmd;
	 

}
/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]
/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [motorworkmode[0-2]]
//	0x00:torque 
//	0:[kp[3-7]]
//	1:[kp[0-6]]
//	2:[kd[7-7]]
//	2:[kd[0-7]]
//	3:[position[0-7]]
//	4:[position[0-7]]
//	5:[velocity[0-7]]
//	6:[velocity[0-3]]
//	6:[torque[4-7]]
//	7:[torque[0-7]]


HAL_StatusTypeDef unpack_cmd(CANRxMessage msg){
//		int p_int = (msg.data[0]<<8)|msg.data[1];
//		int v_int = (msg.data[2]<<4)|(msg.data[3]>>4);
//		int kp_int = ((msg.data[3]&0xF)<<8)|msg.data[4];
//		int kd_int = (msg.data[5]<<4)|(msg.data[6]>>4);
//		int t_int = ((msg.data[6]&0xF)<<8)|msg.data[7];
//		//controller.

//		commands[0] = uint_to_float(p_int, P_MIN, P_MAX, 16);
//		commands[1] = uint_to_float(v_int, V_MIN, V_MAX, 12);
//		commands[2] = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
//		commands[3] = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
//		commands[4] = uint_to_float(t_int, -I_MAX*KT*GR, I_MAX*KT*GR, 12);
	
			if(msg.data[0] == 0x00)//torque&torque work mode
			{
//				controller->p_des =msg.data[1] << 8 | msg.data[2];
//				controller->v_des = msg.data[3] << 4 | (msg.data[4] & 0xF0) >> 4;
//				controller-> = (msg.data[4] & 0x0F) << 8 | msg.data[5];

//				controller->p_des = uint_to_float(controller->p_des, P_MIN, P_MAX, 16);
//				controller->p_des = uint_to_float(controller->p_des, V_MIN, V_MAX, 12);
//				controller->p_des = uint_to_float(controller->p_des, I_MIN, I_MAX, 12);
//				printf("motor_id_t:%d angle_actual_rad:%f\n ", motor_id_t, rv_motor_msg[motor_id_t].angle_actual_rad);
			}
			else if(msg.data[0] == 0x01)//torque work mode
			{
			
			}
			else if(msg.data[0] == 0x02)//velocity work mode
			{
			
			}
	
	//printf("Received   ");
	//printf("%.3f  %.3f  %.3f  %.3f  %.3f   %.3f", controller->p_des, controller->v_des, controller->kp, controller->kd, controller->t_ff, controller->i_q_ref);
	//printf("\n\r");
	}

/* USER CODE END 1 */