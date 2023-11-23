/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
uint8_t state = 0;
uint16_t table[4];
uint16_t value=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim -> Instance==TIM17){
		  HAL_ADC_Start_IT(&hadc3);
	}


	if(htim-> Instance==TIM16){
		switch(state) {
				case 0:
					clear_digit();
					led_display(table[0]);
					digit_display(0);
					break;
				case 1:
					clear_digit();
					led_display(table[1]);
					digit_display(1);
					break;
				case 2:
					clear_digit();
					led_display(table[2]);
					digit_display(2);
					break;
				case 3:
					clear_digit();
					led_display(table[3]);
					digit_display(3);
					break;
				 }
			    state++;
				if(state>3)
				{
					state=0;
				}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	value=HAL_ADC_GetValue(&hadc3);
	modulo_number(value);
	/*if(value>4095){
		value=0;

	}*/
}




void clear_digit(void)
{

	HAL_GPIO_WritePin(GPIOC, COM1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, COM2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, COM3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, COM4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, SegA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, SegB_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, SegC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, SegD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, SegE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, SegF_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, SegG_Pin, GPIO_PIN_SET);

}

void led_display(uint16_t digit){
	switch(digit){
		case 0:
			HAL_GPIO_WritePin(GPIOC, SegA_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SegB_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SegC_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SegD_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SegE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SegF_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SegG_Pin, GPIO_PIN_SET);
			break;
		case 1:

			HAL_GPIO_WritePin(GPIOC, SegB_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SegC_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SegG_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, SegA_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, SegE_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, SegD_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, SegF_Pin, GPIO_PIN_SET);

			break;
		case 2:
			HAL_GPIO_WritePin(GPIOC, SegA_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SegB_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SegD_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SegE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SegG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SegF_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, SegC_Pin, GPIO_PIN_SET);

			break;
		case 3:
			HAL_GPIO_WritePin(GPIOC, SegA_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SegB_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SegC_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SegD_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SegG_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, SegF_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, SegE_Pin, GPIO_PIN_SET);
			break;
		case 4:
			  HAL_GPIO_WritePin(GPIOC, SegB_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegC_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegF_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegG_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegA_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOC, SegD_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOC, SegE_Pin, GPIO_PIN_SET);
			break;
		case 5:
			  HAL_GPIO_WritePin(GPIOC, SegA_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegC_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegD_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegF_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegG_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegB_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOC, SegE_Pin, GPIO_PIN_SET);
			break;
		case 6:
			  HAL_GPIO_WritePin(GPIOC, SegA_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegC_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegD_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegE_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegF_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegG_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegB_Pin, GPIO_PIN_SET);
			break;
		case 7:
			  HAL_GPIO_WritePin(GPIOC, SegA_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegB_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegC_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegD_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOC, SegE_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOC, SegF_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOC, SegG_Pin, GPIO_PIN_SET);
			break;
		case 8:
			 HAL_GPIO_WritePin(GPIOC, SegA_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(GPIOC, SegB_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(GPIOC, SegC_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(GPIOC, SegD_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(GPIOC, SegE_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(GPIOC, SegF_Pin, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(GPIOC, SegG_Pin, GPIO_PIN_RESET);
			break;
		case 9:
			  HAL_GPIO_WritePin(GPIOC, SegA_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegB_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegC_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegD_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegF_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegG_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, SegE_Pin, GPIO_PIN_SET);
			break;
	}

}

void modulo_number(uint16_t number)
{
	uint16_t helping_num = number;
	uint16_t help_table[4] = {1000, 100, 10, 1};
	for (int i = 0; i<4; i++)
	{
		table[i] = helping_num/help_table[i];
		helping_num = helping_num%help_table[i];

	}
}

void digit_display(uint8_t digit)
{
	switch (digit)
	{
	case 0:
		HAL_GPIO_WritePin(GPIOC, COM1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, COM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, COM4_Pin, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOC, COM2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, COM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, COM4_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOC, COM3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, COM1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, COM4_Pin, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOC, COM4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, COM2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, COM3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, COM1_Pin, GPIO_PIN_RESET);
		break;
	}
}




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_TIM16_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim16);
  HAL_TIM_Base_Start_IT(&htim17);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 1699;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 16999;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 9999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SegG_Pin|SegD_Pin|SegE_Pin|SegC_Pin
                          |SegF_Pin|SegA_Pin|SegB_Pin|COM1_Pin
                          |COM2_Pin|COM3_Pin|COM4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SegG_Pin SegD_Pin SegE_Pin SegC_Pin
                           SegF_Pin SegA_Pin SegB_Pin COM1_Pin
                           COM2_Pin COM3_Pin COM4_Pin */
  GPIO_InitStruct.Pin = SegG_Pin|SegD_Pin|SegE_Pin|SegC_Pin
                          |SegF_Pin|SegA_Pin|SegB_Pin|COM1_Pin
                          |COM2_Pin|COM3_Pin|COM4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
