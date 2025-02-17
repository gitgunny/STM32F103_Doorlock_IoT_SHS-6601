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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* Unlock Signal Start */
uint8_t uartRx_unlock_data_1[7] = 	{0xAA, 0xF2, 0x03, 0x5A, 0x00, 0x00, 0x5A};
uint8_t uartRx_unlock_data_2[7] = 	{0xAA, 0xF3, 0x03, 0x51, 0x48, 0x00, 0x19};
uint8_t uartRx_unlock_data_3[7] = 	{0xAA, 0xF4, 0x03, 0x51, 0x56, 0x00, 0x07};
uint8_t uartRx_unlock_data_4[7] = 	{0xAA, 0xF5, 0x03, 0x50, 0x00, 0x00, 0x50}; // 연속_1
uint8_t uartRx_unlock_data_5[7] = 	{0xAA, 0xF6, 0x03, 0x50, 0x09, 0x00, 0x59}; // 연속_1
uint8_t uartRx_unlock_data_6[7] = 	{0xAA, 0xF7, 0x03, 0x50, 0x08, 0x00, 0x58};
uint8_t uartRx_unlock_data_7[7] = 	{0xAA, 0xF8, 0x03, 0x50, 0x0A, 0x00, 0x5A};
uint8_t uartRx_unlock_data_8[7] = 	{0xAA, 0xF9, 0x03, 0x50, 0x00, 0x00, 0x50};
uint8_t uartRx_unlock_data_9[7] = 	{0xAA, 0xFA, 0x03, 0x50, 0x06, 0x00, 0x56};
uint8_t uartRx_unlock_data_10[7] = 	{0xAA, 0xFB, 0x03, 0x50, 0x02, 0x00, 0x52};
uint8_t uartRx_unlock_data_11[7] = 	{0xAA, 0xFC, 0x03, 0x50, 0x0A, 0x00, 0x5A};
uint8_t uartRx_unlock_data_12[7] = 	{0xAA, 0xFD, 0x03, 0x50, 0x00, 0x00, 0x50}; // 연속_2
uint8_t uartRx_unlock_data_13[7] = 	{0xAA, 0xFE, 0x03, 0x50, 0x10, 0x00, 0x40}; // 연속_2
uint8_t uartRx_unlock_data_14[7] = 	{0xAA, 0xFF, 0x03, 0x50, 0x00, 0x00, 0x50}; // 연속_3
uint8_t uartRx_unlock_data_15[7] = 	{0xAA, 0x00, 0x03, 0x50, 0x00, 0x00, 0x50}; // 연속_3
uint8_t uartRx_unlock_data_16[7] = 	{0xAA, 0x01, 0x03, 0x51, 0x53, 0x00, 0x02};
uint8_t uartRx_unlock_data_17[7] = 	{0xAA, 0x02, 0x03, 0x50, 0x00, 0x00, 0x50}; // 2 block
uint8_t uartRx_unlock_data_18[7] = 	{0xAA, 0x02, 0x03, 0x50, 0x00, 0x00, 0xFC}; // 1 block

uint8_t uartTx_unlock_start[1] = 	{0xE0};
uint8_t uartTx_unlock_data_1[7] = 	{0xAA, 0x53, 0x03, 0x53, 0x4C, 0x00, 0x1F};
uint8_t uartTx_unlock_data_2[7] = 	{0xAA, 0x54, 0x03, 0x48, 0x28, 0x00, 0x60};
uint8_t uartTx_unlock_data_3[7] = 	{0xAA, 0x55, 0x03, 0x56, 0x02, 0x02, 0x56};
uint8_t uartTx_unlock_data_4[7] = 	{0xAA, 0x56, 0x03, 0x53, 0x0C, 0x00, 0x5F};
uint8_t uartTx_unlock_data_5[7] = 	{0xAA, 0x57, 0x03, 0x53, 0x0C, 0x00, 0x5F};
uint8_t uartTx_unlock_end[1] = 		{0xFF};
/* Unlock Signal End */

/* Lock Signal Start */
uint8_t uartRx_lock_data_1[7] = 	{0xAA, 0x03, 0x03, 0x5A, 0x00, 0x00, 0x5A};
uint8_t uartRx_lock_data_2[7] = 	{0xAA, 0x04, 0x03, 0x50, 0x00, 0x00, 0x50}; // 연속_1
uint8_t uartRx_lock_data_3[7] = 	{0xAA, 0x05, 0x03, 0x50, 0x10, 0x00, 0x40}; // 연속_1
uint8_t uartRx_lock_data_4[7] = 	{0xAA, 0x06, 0x03, 0x51, 0x48, 0x00, 0x19};
uint8_t uartRx_lock_data_5[7] = 	{0xAA, 0x07, 0x03, 0x51, 0x56, 0x00, 0x07};
uint8_t uartRx_lock_data_6[7] = 	{0xAA, 0x08, 0x03, 0x50, 0x00, 0x00, 0x50}; // 연속_2
uint8_t uartRx_lock_data_7[7] = 	{0xAA, 0x09, 0x03, 0x50, 0x06, 0x00, 0x56}; // 연속_2
uint8_t uartRx_lock_data_8[7] = 	{0xAA, 0x0A, 0x03, 0x50, 0x02, 0x00, 0x52};
uint8_t uartRx_lock_data_9[7] = 	{0xAA, 0x0B, 0x03, 0x50, 0x0A, 0x00, 0x5A};
uint8_t uartRx_lock_data_10[7] = 	{0xAA, 0x0C, 0x03, 0x50, 0x00, 0x00, 0x50};
uint8_t uartRx_lock_data_11[7] = 	{0xAA, 0x0D, 0x03, 0x50, 0x09, 0x00, 0x59};
uint8_t uartRx_lock_data_12[7] = 	{0xAA, 0x0E, 0x03, 0x50, 0x08, 0x00, 0x58};
uint8_t uartRx_lock_data_13[7] = 	{0xAA, 0x0F, 0x03, 0x50, 0x0A, 0x00, 0x5A};
uint8_t uartRx_lock_data_14[7] = 	{0xAA, 0x10, 0x03, 0x50, 0x00, 0x00, 0x50}; // 연속_3
uint8_t uartRx_lock_data_15[7] = 	{0xAA, 0x11, 0x03, 0x50, 0x20, 0x00, 0x70}; // 연속_3
uint8_t uartRx_lock_data_16[7] = 	{0xAA, 0x12, 0x03, 0x50, 0x00, 0x00, 0x50};
uint8_t uartRx_lock_data_17[7] = 	{0xAA, 0x13, 0x03, 0x51, 0x53, 0x00, 0x02};
uint8_t uartRx_lock_data_18[7] = 	{0xAA, 0x14, 0x03, 0x50, 0x00, 0x00, 0x50}; // 2 block
uint8_t uartRx_lock_data_19[7] = 	{0xAA, 0x14, 0x03, 0x50, 0x00, 0x00, 0xFC}; // 1 block

uint8_t uartTx_lock_start[1] = 		{0xE0};
uint8_t uartTx_lock_data_1[7] = 	{0xAA, 0x58, 0x03, 0x5A, 0x00, 0x00, 0x5A}; // 연속_1
uint8_t uartTx_lock_data_2[7] = 	{0xAA, 0x59, 0x03, 0x53, 0x0E, 0x00, 0x5D}; // 연속_1
uint8_t uartTx_lock_data_3[7] = 	{0xAA, 0x5A, 0x03, 0x48, 0x28, 0x00, 0x60};
uint8_t uartTx_lock_data_4[7] = 	{0xAA, 0x5B, 0x03, 0x56, 0x02, 0x15, 0x41};
uint8_t uartTx_lock_data_5[7] = 	{0xAA, 0x5C, 0x03, 0x53, 0x4E, 0x00, 0x1D};
uint8_t uartTx_lock_data_6[7] = 	{0xAA, 0x5D, 0x03, 0x53, 0x4E, 0x00, 0x1D};
uint8_t uartTx_lock_end[1] = 		{0xFF};
/* Lock Signal End */

_Bool button_flag;
uint8_t rx_start = 0x03;
uint8_t tx_start = 0x58;
uint8_t rx_status, tx_status;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(Origin_Rx_GPIO_Port, Origin_Rx_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Origin_Tx_GPIO_Port, Origin_Tx_Pin, GPIO_PIN_SET);
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 1850;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 4340;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 6720;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 20000;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 2050;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 4220;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 6500;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
	if (GPIO_Pin == B1_Pin && button_flag == 0) {
		HAL_GPIO_WritePin(Origin_Rx_GPIO_Port, Origin_Rx_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Origin_Tx_GPIO_Port, Origin_Tx_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(Fake_Rx_GPIO_Port, Fake_Rx_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Fake_Tx_GPIO_Port, Fake_Tx_Pin, GPIO_PIN_SET);

		HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
		HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
		HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
		HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4);
		HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
		HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_2);
		HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_3);
		HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_4);

		button_flag = 1;
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			HAL_UART_Transmit_IT(&huart1, uartRx_unlock_data_1, 7);
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			HAL_UART_Transmit_IT(&huart1, uartRx_unlock_data_1, 7);
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			HAL_UART_Transmit_IT(&huart1, uartRx_unlock_data_1, 7);
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
			HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_2);
			HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
			HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_4);
			HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
			HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_2);
			HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_3);
			HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_4);

			HAL_GPIO_WritePin(Fake_Rx_GPIO_Port, Fake_Rx_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Fake_Tx_GPIO_Port, Fake_Tx_Pin, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(Origin_Rx_GPIO_Port, Origin_Rx_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(Origin_Tx_GPIO_Port, Origin_Tx_Pin, GPIO_PIN_SET);

			button_flag = 0;
		}
	}
	if (htim->Instance == TIM3) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			HAL_UART_Transmit_IT(&huart3, uartTx_unlock_start, 1);
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			HAL_UART_Transmit_IT(&huart3, uartTx_unlock_data_1, 7);
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			HAL_UART_Transmit_IT(&huart3, uartTx_unlock_data_1, 7);
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			HAL_UART_Transmit_IT(&huart3, uartTx_unlock_data_1, 7);
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
