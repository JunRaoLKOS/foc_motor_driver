/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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


/// high-bandwidth 3-phase motor control for robots
/// Written by Ben Katz, with much inspiration from Bayley Wang, Nick Kirkby, Shane Colton, David Otten, and others
/// Hardware documentation can be found at build-its.blogspot.com

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "structs.h"
#include <stdio.h>
#include <string.h>

#include "stm32f4xx_flash.h"
#include "flash_writer.h"
#include "position_sensor.h"
#include "preference_writer.h"
#include "hw_config.h"
#include "user_config.h"
#include "drv8323.h"
#include "math_ops.h"
#include "foc_pid.h"
#include "foc_states.h"
#include "foc_current_loop.h"
#include "Rte_Ctrl_interface.h"
#include "foc_speed_loop.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define VERSION_NUM 2.0f


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Flash Registers */
float __float_reg[64];
int __int_reg[256];
PreferenceWriter prefs;

EncoderStruct comm_encoder;
EncoderStruct comm_encoder2;
DRVStruct drv;

CANTxMessage can_tx;
CANRxMessage can_rx;


uint8_t Serial2RxBuffer[1];
uint16_t ADCValue[3]={0};

int test_states = 0;
float test_pos_rad = 0,test_spd_rad=0,fbk_pos_rad=0,fbk_spd_rad=0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* 
 bsp_init
*/
void bsp_init(void)
{
	MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init(TIM1);
 
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
 // MX_ADC3_Init();
  MX_SPI2_Init();

  preference_writer_init(&prefs, 6);
  preference_writer_load(prefs);

  ps_warmup(&comm_encoder, 100);			// clear the noisy data when the encoder first turns on
	
	ps_warmup2(&comm_encoder2, 100);			// clear the noisy data when the encoder first turns on

  //HAL_ADC_Start(&hadc3);
	//HAL_ADC_Start_DMA(&hadc3,(uint32_t *)ADCValue,3);

  /* DRV8323 setup */
  HAL_GPIO_WritePin(DRV_CS, GPIO_PIN_SET ); 	// CS high
  HAL_GPIO_WritePin(ENABLE_PIN, GPIO_PIN_SET );
  HAL_Delay(1);
 
  HAL_Delay(1);
  drv_write_DCR(drv, 0x0, DIS_GDF_EN, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);
  HAL_Delay(1);
  int CSA_GAIN;
  if(I_MAX <= 40.0f){CSA_GAIN = CSA_GAIN_40;}	// Up to 40A use 40X amplifier gain
  else{CSA_GAIN = CSA_GAIN_20;}					// From 40-60A use 20X amplifier gain.  (Make this generic in the future)
  drv_write_CSACR(drv, 0x0, 0x1, 0x0, CSA_GAIN, 0x1, 0x0, 0x0, 0x0, SEN_LVL_1_0);
  HAL_Delay(1);
  
  HAL_Delay(1);
  drv_write_OCPCR(drv, TRETRY_4MS, DEADTIME_400NS, OCP_RETRY, OCP_DEG_8US, VDS_LVL_1_88);
  HAL_Delay(1);
	drv_write_HSR(drv,LOCK_OFF,IDRIVEP_HS_1000MA,IDRIVEN_HS_120MA);
	HAL_Delay(1);
	drv_write_LSR(drv,0x01,TDRIVE_1000NS,IDRIVEP_LS_1000MA,IDRIVEN_LS_120MA);
	HAL_Delay(1);
  drv_disable_gd(drv);
  HAL_Delay(1);
 	
	Rte_set_DefaultInit();

  /* CAN setup */
  can_rx_init(&can_rx);
  can_tx_init(&can_tx);
  HAL_CAN_Start(&CAN_H); //start CAN
  
  /* Set Interrupt Priorities */
  HAL_NVIC_SetPriority(CAN_ISR, 0x01, 0x01);

  /* Turn on interrupts */
  HAL_UART_Receive_IT(&huart2, (uint8_t *)Serial2RxBuffer, 1);
  HAL_TIM_Base_Start_IT(&htim1);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();

  SystemClock_Config();

  bsp_init();
 
	adc_calib_Init();
	
  while (1)
  {
	  HAL_Delay(10);
	  
		foc_states_change(test_states);
		
		Rte_set_TargetSpeedRad(test_spd_rad);
		Rte_set_TargetPositionRad(test_pos_rad);
		
		fbk_pos_rad = Rte_get_FbkPositionRad();
		
		fbk_spd_rad = Rte_get_FbkSpeedRad();
		
		//printf("%d,%d\n",g_MotorPIArgs.Speed.Fbk,g_MotorPIArgs.Speed.Ref);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
