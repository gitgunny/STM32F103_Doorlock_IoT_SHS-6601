/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t uart_buf;
uint8_t unlock_data[10][7] = {
		{0xAA, 0x00, 0x03, 0x51, 0x48, 0x00, 0x19},
		{0xAA, 0x00, 0x03, 0x51, 0x56, 0x00, 0x07},
		{0xAA, 0x00, 0x03, 0x50, 0x00, 0x00, 0x50},
		{0xAA, 0x00, 0x03, 0x50, 0x09, 0x00, 0x59},
		{0xAA, 0x00, 0x03, 0x50, 0x08, 0x00, 0x58},
		{0xAA, 0x00, 0x03, 0x50, 0x0A, 0x00, 0x5A},
		{0xAA, 0x00, 0x03, 0x50, 0x00, 0x00, 0x50},
		{0xAA, 0x00, 0x03, 0x50, 0x06, 0x00, 0x56},
		{0xAA, 0x00, 0x03, 0x50, 0x02, 0x00, 0x52},
		{0xAA, 0x00, 0x03, 0x50, 0x0A, 0x00, 0x5A}
};
uint8_t status, cnt_1, cnt_2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(Origin_Rx_GPIO_Port, Origin_Rx_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Origin_Tx_GPIO_Port, Origin_Tx_Pin, GPIO_PIN_SET);

  HAL_UART_Receive_IT(&huart1, &uart_buf, 1);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 57600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_2;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Fake_Rx_Pin|Fake_Tx_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Origin_Rx_Pin|Origin_Tx_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Fake_Rx_Pin Fake_Tx_Pin */
  GPIO_InitStruct.Pin = Fake_Rx_Pin|Fake_Tx_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Origin_Rx_Pin Origin_Tx_Pin */
  GPIO_InitStruct.Pin = Origin_Rx_Pin|Origin_Tx_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (cnt_1 == 3 && uart_buf != 0xAA) {
			cnt_1 = 4;

			unlock_data[0][1] = ++uart_buf;
		}
		if (cnt_2 == 3 && uart_buf != 0x5A) {
			cnt_2 = 4;

			TIM2->CCR1 = 4470;
			TIM2->CCR2 = TIM2->CCR1 + 23;
			TIM2->CCR3 = TIM2->CCR2 + 23;
			TIM2->CCR4 = TIM2->CCR3 + 20;
			TIM2->CNT = 0;

			HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
			HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
			HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
			HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4);
		}
		if (uart_buf == 0xAA) cnt_1++;
		if (uart_buf == 0x5A) cnt_2++;

		HAL_UART_Receive_IT(&huart1, &uart_buf, 1);
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			HAL_GPIO_WritePin(Fake_Rx_GPIO_Port, Fake_Rx_Pin, GPIO_PIN_SET);

			HAL_UART_Transmit_IT(&huart1, unlock_data[status], 7);
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			HAL_UART_Transmit_IT(&huart1, unlock_data[status], 7);
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			HAL_UART_Transmit_IT(&huart1, unlock_data[status], 7);
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			if (status == 0) {
				status = 1;

				TIM2->CCR1 = 231;
				TIM2->CCR2 = TIM2->CCR1 + 23;
				TIM2->CCR3 = TIM2->CCR2 + 23;
				TIM2->CCR4 = TIM2->CCR3 + 20;
			} else if (status == 1) {
				status = 2;

				TIM2->CCR1 = 126;
				TIM2->CCR2 = TIM2->CCR1 + 23;
				TIM2->CCR3 = TIM2->CCR2 + 23;
				TIM2->CCR4 = TIM2->CCR3 + 20;
			} else if (status == 2) {
				status = 3;

				TIM2->CCR1 = 22;
				TIM2->CCR2 = TIM2->CCR1 + 23;
				TIM2->CCR3 = TIM2->CCR2 + 23;
				TIM2->CCR4 = TIM2->CCR3 + 20;
			} else if (status == 3) {
				status = 4;

				TIM2->CCR1 = 3384;
				TIM2->CCR2 = TIM2->CCR1 + 23;
				TIM2->CCR3 = TIM2->CCR2 + 23;
				TIM2->CCR4 = TIM2->CCR3 + 20;
			} else if (status == 4) {
				status = 5;

				TIM2->CCR1 = 115;
				TIM2->CCR2 = TIM2->CCR1 + 23;
				TIM2->CCR3 = TIM2->CCR2 + 23;
				TIM2->CCR4 = TIM2->CCR3 + 20;
			} else if (status == 5) {
				status = 6;

				TIM2->CCR1 = 784;
				TIM2->CCR2 = TIM2->CCR1 + 23;
				TIM2->CCR3 = TIM2->CCR2 + 23;
				TIM2->CCR4 = TIM2->CCR3 + 20;
			} else if (status == 6) {
				status = 7;

				TIM2->CCR1 = 26;
				TIM2->CCR2 = TIM2->CCR1 + 23;
				TIM2->CCR3 = TIM2->CCR2 + 23;
				TIM2->CCR4 = TIM2->CCR3 + 20;
			} else if (status == 7) {
				status = 8;

				TIM2->CCR1 = 4450;
				TIM2->CCR2 = TIM2->CCR1 + 23;
				TIM2->CCR3 = TIM2->CCR2 + 23;
				TIM2->CCR4 = TIM2->CCR3 + 20;
			} else if (status == 8) {
				status = 9;

				TIM2->CCR1 = 106;
				TIM2->CCR2 = TIM2->CCR1 + 23;
				TIM2->CCR3 = TIM2->CCR2 + 23;
				TIM2->CCR4 = TIM2->CCR3 + 20;
			} else {
				HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
				HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_2);
				HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
				HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_4);
			}

			TIM2->CNT = 0;
			unlock_data[status][1] = unlock_data[0][1] + status;

			HAL_GPIO_WritePin(Fake_Rx_GPIO_Port, Fake_Rx_Pin, GPIO_PIN_RESET);
		}
	}
}
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
