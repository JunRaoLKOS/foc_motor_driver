/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "structs.h"
#include "usart.h"
#include "spi.h"
#include "gpio.h"
#include "adc.h"
#include "can.h"
#include "position_sensor.h"
#include "hw_config.h"
#include "user_config.h"
#include "foc_current_loop.h"
#include "foc_states.h"
#include "Rte_Ctrl_interface.h"
#include "foc_states.h"
/* USER CODE END Includes */

 
/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc3;
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 RX0 interrupt.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  HAL_CAN_GetRxMessage(&CAN_H, CAN_RX_FIFO0, &can_rx.rx_header, can_rx.data);	// Read CAN
  uint32_t TxMailbox;
  //pack_reply(&can_tx, CAN_ID,  comm_encoder.angle_multiturn[0]/GR, comm_encoder.velocity/GR, controller.i_q_filt*KT*GR, controller.v_bus_filt);	// Pack response

  /* Check for special Commands */
	if (can_rx.id == 0x7FF)//Set Motor Ack 01:Auto answer 02:Ask and answer 03:Set zero
	{               
		if ((can_rx.data[1] == CAN_ID) && (can_rx.data[0] == (CAN_ID<< 8))&&(can_rx.data[0] == 0x00))// [data:0 High Can id 8bit] [data:1 Low Can id 8bit]
		{
			switch (can_rx.data[3])
			{
				case 01://Auto answer
					can_rx.ack_mode = AUTO_ANSWER;
					break;
				case 02://Ask and answer
					can_rx.ack_mode = ACK_ANSWER;
					break;
				case 03://Set zero
					 //update_fsm(&state, ZERO_CMD);
					break;
				case 04://Set canID
					CAN_ID = can_rx.data[4]>>8| can_rx.data[5];
					if(!preference_writer_ready(prefs)){ preference_writer_open(&prefs);}
					preference_writer_flush(&prefs);
					preference_writer_close(&prefs);
					preference_writer_load(prefs);
					printf("\n\r  Saved new zero position:  %d\n\r\n\r", CAN_ID);
					break;
				case 0x81://Query comm_mode
					if(can_rx.ack_mode == AUTO_ANSWER){can_tx.data[3] = AUTO_ANSWER;}
					else{can_tx.data[3] = ACK_ANSWER;}
					break;
				
			}
			
			Setpack_reply(&can_tx,CAN_ID,01,can_rx.data[3]);//ack
		}
		else if((can_rx.data[1] == 0xff) && can_rx.data[0] == (0xff))//Query can id
		{
			can_tx.data[0] = 0xff;
			can_tx.data[1] = 0xff;
			can_tx.data[2] = 0x00;
			can_tx.data[3] = (uint8_t)CAN_ID>>8;
			can_tx.data[4] = (uint8_t)CAN_ID>>0;
			
		}
			
	}
  else
	{
	 // unpack_cmd(can_rx, &controller);	// Unpack commands
	  //controller.timeout = 0;					// Reset timeout counter
  }
	
	 HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send response

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_SET  );
	
	foc_states_task();
	
  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  /* USER CODE END ADC_IRQn 1 */
}


/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	HAL_UART_IRQHandler(&huart2);

	char c = Serial2RxBuffer[0];
	//update_fsm(&state, c);

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */
//	controller.adc_mos_temperature_raw = ADCValue[0];
//	controller.adc_coil_temperature_raw = ADCValue[1];
//	controller.adc_vbus_raw= ADCValue[2];
//	controller.v_bus = controller.adc_vbus_raw*V_SCALE;
	HAL_ADC_Start_DMA(&hadc3,(uint32_t *)ADCValue,3);
  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void can_tx_rx(void)
{

	int no_mesage = HAL_CAN_GetRxMessage(&CAN_H, CAN_RX_FIFO0, &can_rx.rx_header, can_rx.data);	// Read CAN
	if(!no_mesage){
		uint32_t TxMailbox;

		//pack_reply(&can_tx, CAN_ID,  comm_encoder.angle_multiturn[0]/GR, comm_encoder.velocity/GR, controller.i_q_filt*KT*GR, controller.v_bus_filt);	// Pack response

		/* Check for special Commands */
		if ((can_rx.rx_header.StdId == 0x7FF)&&(can_rx.rx_header.DLC != 0 ))//Set Motor Ack 01:Auto answer 02:Ask and answer 03:Set zero
		{               
			if ((can_rx.data[1] == CAN_ID) && (can_rx.data[0] == (CAN_ID>> 8))&&(can_rx.data[2] == 0x00))// [data:0 High Can id 8bit] [data:1 Low Can id 8bit]
			{
				switch (can_rx.data[3])
				{
					case 01://Auto answer
						can_rx.ack_mode = AUTO_ANSWER;
						break;
					case 02://Ask and answer
						can_rx.ack_mode = ACK_ANSWER;
						break;
					case 03://Set zero
						 //update_fsm(&state, ZERO_CMD);
						break;
					case 04://Set canID
						CAN_ID = can_rx.data[4]<<8| can_rx.data[5];
						if(!preference_writer_ready(prefs)){ preference_writer_open(&prefs);}
						preference_writer_flush(&prefs);
						preference_writer_close(&prefs);
						preference_writer_load(prefs);
						printf("\n\r  Saved new can id:  %d\n\r\n\r", CAN_ID);
						break;
					case 0x81://Query comm_mode
						if(can_rx.ack_mode == AUTO_ANSWER){can_tx.data[3] = AUTO_ANSWER;}
						else{can_tx.data[3] = ACK_ANSWER;}
						break;
					
				}
				can_tx.tx_header.DLC = 0x04;
				Setpack_reply(&can_tx,CAN_ID,01,can_rx.data[3]);//ack
				HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send response
			}
			else if((can_rx.data[1] == 0xff) && can_rx.data[0] == (0xff))//Query can id
			{
				can_tx.data[0] = 0xff;
				can_tx.data[1] = 0xff;
				can_tx.data[2] = 0x00;
				can_tx.data[3] = (uint8_t)CAN_ID>>8;
				can_tx.data[4] = (uint8_t)CAN_ID>>0;
				can_tx.tx_header.DLC = 0x05;
				HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send response
				
			}
				
		}
		else if(can_rx.rx_header.ExtId == CAN_ID )
		{
//			unpack_cmd(can_rx, &controller);	// Unpack commands
//			controller.timeout = 0;					// Reset timeout counter
			can_tx.tx_header.DLC = 0x08;
			HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send response
		}
		

	}
}


/* USER CODE END 1 */
