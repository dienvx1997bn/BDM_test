/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "string.h"
#include "stdio.h"

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
volatile uint32_t bdm_ext[10] = {0};
volatile uint8_t enable_sync = 0;
volatile uint8_t bdm_wait_ack = 0;
volatile uint8_t bdm_wait_rx = 0;
volatile uint32_t bdm_wait_rx_time = 0;

volatile uint32_t sync_time_captured = 0;
volatile uint32_t delay_1TC, delay_4TC, delay_10TC, delay_12TC, delay_end;

uint8_t bdm_data[10] = {0};
volatile uint32_t temp[24];

uint8_t uart_tx_msg[20] = "Hello test Dien!\n";
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
	
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
	static uint8_t count = 0;
	
	if(enable_sync) {
		bdm_ext[count] = __HAL_TIM_GetCounter(&htim2);
		count++;
		
		if(count > 2) {
			sync_time_captured = bdm_ext[2] - bdm_ext[1];
//			sync_time_captured = sync_time_captured/128;
			delay_1TC = sync_time_captured / 128;
			delay_4TC = (sync_time_captured + 16) /32;
			delay_12TC = delay_4TC * 3;
			delay_10TC = delay_4TC * 5 /2;
			delay_end = sync_time_captured /8;
			enable_sync = 0;
			count = 0;
		}
	}
	if(bdm_wait_ack) {	//wait for ack
		if(HAL_GPIO_ReadPin(BDM_PIN_GPIO_Port, GPIO_PIN_8)) {
			bdm_wait_ack = 0;
		}
	}
	if(bdm_wait_rx) {
		bdm_wait_rx_time = __HAL_TIM_GetCounter(&htim2);
		bdm_wait_rx = 0;
	}
}

static __inline void delay4TC() {
	//reset counter
	__HAL_TIM_SetCounter(&htim2, 0);
	while(__HAL_TIM_GetCounter(&htim2) < delay_4TC);
}

static __inline void delay12TC() {
	//reset counter
	__HAL_TIM_SetCounter(&htim2, 0);
	while(__HAL_TIM_GetCounter(&htim2) < delay_12TC);
}

static __inline void delay1024TC() {
	//reset counter
	__HAL_TIM_SetCounter(&htim2, 0);
	while(__HAL_TIM_GetCounter(&htim2) < sync_time_captured * 8);
}

static __inline void bdm_pin_cfg_ext() {
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//	/*Configure GPIO pin : BDM_PIN_Pin */
//	GPIO_InitStruct.Pin = BDM_PIN_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(BDM_PIN_GPIO_Port, &GPIO_InitStruct);
	GPIOC->MODER &= ~(1<<16); 		//PC8
	/* EXTI interrupt init*/
//	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
//	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

static __inline void bdm_pin_cfg_ext_rising() {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/*Configure GPIO pin : BDM_PIN_Pin */
	GPIO_InitStruct.Pin = BDM_PIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BDM_PIN_GPIO_Port, &GPIO_InitStruct);
	
	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

static __inline void bdm_pin_cfg_highZ() {
//	HAL_GPIO_DeInit(BDM_PIN_GPIO_Port, BDM_PIN_Pin);
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = BDM_PIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BDM_PIN_GPIO_Port, &GPIO_InitStruct);
}

static __inline void bdm_pin_cfg_output() {
//	HAL_GPIO_WritePin(BDM_PIN_GPIO_Port, BDM_PIN_Pin, GPIO_PIN_SET);
//	
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//	GPIO_InitStruct.Pin = BDM_PIN_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//	HAL_GPIO_Init(BDM_PIN_GPIO_Port, &GPIO_InitStruct);
//	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	GPIOC->MODER |= (1<<16); 		//PC8
	GPIOC->OTYPER &= ~(1<<8);
	GPIOC->OSPEEDR |= (3<<16);
}

static void bdm_sync(uint8_t sync_enable) {
	bdm_pin_cfg_output();
	//set low for 1ms
	HAL_GPIO_WritePin(BDM_PIN_GPIO_Port, BDM_PIN_Pin, GPIO_PIN_RESET);
	HAL_Delay(2);
	enable_sync = sync_enable;
	
	//reset counter
	__HAL_TIM_SetCounter(&htim2, 0);
	
	//tongle high
	HAL_GPIO_WritePin(BDM_PIN_GPIO_Port, BDM_PIN_Pin, GPIO_PIN_SET);
	//wait sync
	bdm_pin_cfg_ext();
	
	//wait sync success
	if(enable_sync)
		while(enable_sync);
	else
		HAL_Delay(1);
}

static __inline void bdm_tx_1() {
	//pull low
	HAL_GPIO_WritePin(BDM_PIN_GPIO_Port, BDM_PIN_Pin, GPIO_PIN_RESET);
	//reset counter
	__HAL_TIM_SetCounter(&htim2, 0);
	
	//delay 4 TC
	while(__HAL_TIM_GetCounter(&htim2) <= delay_4TC);
	
	//pull high
	HAL_GPIO_WritePin(BDM_PIN_GPIO_Port, BDM_PIN_Pin, GPIO_PIN_SET);
	
	//delay 12 TC
	while(__HAL_TIM_GetCounter(&htim2) <= delay_end);
}

static __inline void bdm_tx_0() {
	//pull low
	HAL_GPIO_WritePin(BDM_PIN_GPIO_Port, BDM_PIN_Pin, GPIO_PIN_RESET);
	//reset counter
	__HAL_TIM_SetCounter(&htim2, 0);
	
	//delay 12 TC
	while(__HAL_TIM_GetCounter(&htim2) <= delay_12TC);
	
	//pull high
	HAL_GPIO_WritePin(BDM_PIN_GPIO_Port, BDM_PIN_Pin, GPIO_PIN_SET);
	
	//delay 4 TC
	while(__HAL_TIM_GetCounter(&htim2) <= delay_end);
}

static __inline uint8_t bdm_rx() {/*Configure GPIO pin : PC9 */
	uint8_t ret = 0;
	//pull low
	//reset counter
	__HAL_TIM_SetCounter(&htim2, 0);
	
	bdm_pin_cfg_output();
	HAL_GPIO_WritePin(BDM_PIN_GPIO_Port, BDM_PIN_Pin, GPIO_PIN_RESET);
	
	//delay 4 TC
	while(__HAL_TIM_GetCounter(&htim2) <= delay_4TC);
	
	//test 1
	bdm_wait_rx = 1;
	bdm_pin_cfg_ext();
	while(bdm_wait_rx == 1);
	
	if(bdm_wait_rx_time < delay_12TC) {
		ret = 1;
	}
	
	//wait to end
	while(__HAL_TIM_GetCounter(&htim2) <= delay_end);
	
	return ret;
}

void bdm_send(uint8_t command, uint8_t ack) {
	/*Configure GPIO pin*/
	bdm_pin_cfg_output();
	
	for(uint8_t i=0; i<8; i++) {	//shift bit
		if(command & 0x80)
            bdm_tx_1();
        else
            bdm_tx_0();
        command = command << 1;
	}
	
	bdm_pin_cfg_ext();
	
	if(ack) {
		bdm_wait_ack = 1;
		while(bdm_wait_ack);
		//delay a bit
		__HAL_TIM_SetCounter(&htim2, 0);
		while(__HAL_TIM_GetCounter(&htim2) <= delay_end);
	}
}

void bdm_read(uint8_t *byte, uint8_t len) {
	memset(byte, 0, len);
	int x, i = 0;
	for(x = 0; x < len; x++) {
		for(i = 0; i <= 7; i++) {
			byte[x] |= (bdm_rx() << (7-i));
		}
	}
}

void bdm_soft_reset() {
	bdm_pin_cfg_output();
	//	HAL_GPIO_WritePin(BDM_PIN_GPIO_Port, BDM_PIN_Pin, GPIO_PIN_RESET);
	HAL_Delay(220);
	
	//tongle high
	HAL_GPIO_WritePin(BDM_PIN_GPIO_Port, BDM_PIN_Pin, GPIO_PIN_SET);
	
	//wait sync
	bdm_pin_cfg_ext();
	HAL_Delay(110);
}

void bdm_start() {
	bdm_sync(1);
	delay1024TC();
	bdm_send(0xD5, 1);	//ACK_ENABLE
	bdm_send(0xE4, 0);	//READ_BD_BYTE
	bdm_send(0xFF, 0);
	bdm_send(0x01, 1);
	bdm_read(bdm_data, 2);
	delay1024TC();
	
	if(bdm_data[1] == 0x80) {
		sprintf((char *)uart_tx_msg, "bdm_start ready\n");
		HAL_UART_Transmit(&huart1, uart_tx_msg, strlen((char *)uart_tx_msg), 1000);
		HAL_Delay(100);
	} else {
		sprintf((char *)uart_tx_msg, "bdm_start fail 0x%02X%02X\n", bdm_data[0], bdm_data[1]);
		HAL_UART_Transmit(&huart1, uart_tx_msg, strlen((char *)uart_tx_msg), 1000);
		HAL_Delay(100);
	}		
}

void bdm_read_address(uint16_t address) {
	delay1024TC();
	bdm_send(0xD5, 1);	//ACK_ENABLE
	bdm_send(0xE8, 0);	//READ_BD_BYTE
	bdm_send(address >> 8, 0);
	bdm_send(address & 0xFF, 1);
	bdm_read(bdm_data, 2);
//	HAL_UART_Transmit_DMA(&huart1, bdm_data, 2);
}

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
	static uint16_t address = 0x00;
	static uint16_t address_e = 0xFFF0;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);

	HAL_UART_Transmit(&huart1, uart_tx_msg, strlen((char *)uart_tx_msg), 100);
	HAL_Delay(10);
	
//	bdm_pin_reset();
	bdm_pin_cfg_output();
	//set low for 1ms
	HAL_GPIO_WritePin(BDM_PIN_GPIO_Port, BDM_PIN_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
		
	bdm_start();
	bdm_soft_reset();
	
	bdm_start();
	HAL_Delay(20);

	while(address <= address_e) {
		sprintf((char *)uart_tx_msg, "addr %04X: ", address);
		HAL_UART_Transmit(&huart1, uart_tx_msg, strlen((char *)uart_tx_msg), 1000);
		bdm_sync(0);
		bdm_read_address(address);
		sprintf((char *)uart_tx_msg, "%02X%02X\n", bdm_data[0], bdm_data[1]);
		HAL_UART_Transmit(&huart1, uart_tx_msg, strlen((char *)uart_tx_msg), 1000);
		address +=2;
	}
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	  HAL_Delay(1);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	  HAL_Delay(1);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BDM_PIN_GPIO_Port, BDM_PIN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BDM_PIN_Pin */
  GPIO_InitStruct.Pin = BDM_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BDM_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
