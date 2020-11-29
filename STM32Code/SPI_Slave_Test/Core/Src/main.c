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
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
// Global variables for light frequencies
uint8_t lightFreqHz1 = 10;
uint8_t lightFreqHz2 = 12; 
uint8_t lightFreqHz3 = 13; 
uint8_t lightFreqHz4 = 14; 
/* For floating point
float lightFreqHz1 = 1;
float lightFreqHz2 = 2; 
float lightFreqHz3 = 3; 
float lightFreqHz4 = 4; 
*/

// Initialize buffer variables to store SPI received data
uint8_t instrReceiv;
uint8_t LEDinfo[4];

// Global delay variable for pulse length of move command [ms]
uint32_t movePulseDelay = 5;

// Surface SPI User Made Instructions (even parity)
//1. Write the LED frequencies 
const uint8_t WriteLEDFreq_instr = 0x55;
//2. Assert a movement command, read if movement was successfull
const uint8_t AssertLeft_instr = 0x65;
const uint8_t AssertRight_instr = 0x66;
const uint8_t AssertMove3_instr = 0x69;
const uint8_t AssertMove4_instr = 0x6A;
//3. Commands send by MCU to indicate if movement was sent to wheelchair
const uint8_t AckYes = 0x95;
const uint8_t AckNo = 0x96;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    // Wait to receive instruction
		HAL_SPI_Receive(&hspi1,&instrReceiv,1,HAL_MAX_DELAY);
		
		// Execute response depending on instruction sent
		switch(instrReceiv)
		{
				case AssertLeft_instr:
				/* In future check to see if moving through Gyro, only move
				if chair is stationary, send ackYes or ackNo depending on whether
				the chair moved	*/ 
				
				// Transmit that you are initiating the turn
				HAL_SPI_Transmit(&hspi1,(uint8_t *)&AckYes,1,HAL_MAX_DELAY);
				// Toggle the left turn pin on for movePulseDelay
				HAL_GPIO_WritePin(LeftTurn_GPIO_Port,LeftTurn_Pin,GPIO_PIN_SET);
				HAL_Delay(movePulseDelay);
				HAL_GPIO_WritePin(LeftTurn_GPIO_Port,LeftTurn_Pin,GPIO_PIN_RESET);
				break;
			case AssertRight_instr:
				/* In future check to see if moving through Gyro, only move
				if chair is stationary, send ackYes or ackNo depending on whether
				the chair moved	*/ 
				
				// Transmit that you are initiating the turn
				HAL_SPI_Transmit(&hspi1,(uint8_t *)&AckNo,1,HAL_MAX_DELAY);
				// Toggle the left turn pin on for movePulseDelay
				HAL_GPIO_WritePin(RightTurn_GPIO_Port,RightTurn_Pin,GPIO_PIN_SET);
				HAL_Delay(movePulseDelay);
				HAL_GPIO_WritePin(RightTurn_GPIO_Port,RightTurn_Pin,GPIO_PIN_RESET);
				break;
			case AssertMove3_instr:
				/* In future check to see if moving through Gyro, only move
				if chair is stationary, send ackYes or ackNo depending on whether
				the chair moved	*/ 
				
				// Transmit that you are initiating the movement
				HAL_SPI_Transmit(&hspi1,(uint8_t *)&AckYes,1,HAL_MAX_DELAY);
				// Toggle the left turn pin on for movePulseDelay
				HAL_GPIO_WritePin(Move3_GPIO_Port,Move3_Pin,GPIO_PIN_SET);
				HAL_Delay(movePulseDelay);
				HAL_GPIO_WritePin(Move3_GPIO_Port,Move3_Pin,GPIO_PIN_RESET);
				break;
			case AssertMove4_instr:
				/* In future check to see if moving through Gyro, only move
				if chair is stationary, send ackYes or ackNo depending on whether
				the chair moved	*/ 
				
				// Transmit that you are initiating the movement
				HAL_SPI_Transmit(&hspi1,(uint8_t *)&AckYes,1,HAL_MAX_DELAY);
				// Toggle the left turn pin on for movePulseDelay
				HAL_GPIO_WritePin(Move4_GPIO_Port,Move4_Pin,GPIO_PIN_SET);
				HAL_Delay(movePulseDelay);
				HAL_GPIO_WritePin(Move4_GPIO_Port,Move4_Pin,GPIO_PIN_RESET);
				break;
			case WriteLEDFreq_instr:
				// Read the incoming data and update light frequency values
				HAL_SPI_Receive(&hspi1,LEDinfo,4,HAL_MAX_DELAY);	
				lightFreqHz1 = LEDinfo[0];
				lightFreqHz2 = LEDinfo[1];
				lightFreqHz3 = LEDinfo[2];
				lightFreqHz4 = LEDinfo[3];
				break;
		}
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RightTurn_Pin|LeftTurn_Pin|LED2_Pin|LED1_Pin
                          |LED4_Pin|Move3_Pin|Move4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RightTurn_Pin LeftTurn_Pin LED2_Pin LED1_Pin
                           LED4_Pin Move3_Pin Move4_Pin */
  GPIO_InitStruct.Pin = RightTurn_Pin|LeftTurn_Pin|LED2_Pin|LED1_Pin
                          |LED4_Pin|Move3_Pin|Move4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
