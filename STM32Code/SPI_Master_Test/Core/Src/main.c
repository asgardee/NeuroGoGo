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
#include "FloatIntConv.h"

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
SPI_HandleTypeDef hspi1;

// Global variables for light frequencies
uint8_t lightFreqHz1 = 1;
uint8_t lightFreqHz2 = 2; 
uint8_t lightFreqHz3 = 3; 
uint8_t lightFreqHz4 = 4; 
/* For floating point
float lightFreqHz1 = 1;
float lightFreqHz2 = 2; 
float lightFreqHz3 = 3; 
float lightFreqHz4 = 4; 
*/

/* USER CODE BEGIN PV */

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
/*
const uint8_t LED1WriteFreq_instr = 0x55;
const uint8_t LED2WriteFreq_instr = 0x56;
const uint8_t LED3WriteFreq_instr = 0x59;
const uint8_t LED4WriteFreq_instr = 0x5A;

const uint8_t LeftRead_instr = 0x65;
const uint8_t RightRead_instr = 0x66;
const uint8_t Move3Read_instr = 0x69;
const uint8_t Move4Read_instr = 0x6A;
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
	// Array that holds status of movement (1 = moving)
	uint8_t SPIMove[4];
	
	// Arrays that hold desired frequencies
	uint8_t SPI_LED[4];
	SPI_LED[0] = lightFreqHz1;
	SPI_LED[1] = lightFreqHz2;
	SPI_LED[2] = lightFreqHz3;
	SPI_LED[3] = lightFreqHz4;
	/* For floating point conversion (if needed)
	uint8_t SPI_LED1[4];
	uint8_t SPI_LED2[4];
	uint8_t SPI_LED3[4];
	uint8_t SPI_LED4[4];
	// Converts the float freq values to the IEEE 754 rep. (4-byte)
	// Note that SPI_LED is an array hence the lack of address reference, &
	float2int(&lightFreqHz1, SPI_LED1);
	float2int(&lightFreqHz2, SPI_LED2);
	float2int(&lightFreqHz3, SPI_LED3);
	float2int(&lightFreqHz4, SPI_LED4);
	*/
	
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
  MX_SPI1_Init();
	
  /* USER CODE BEGIN 2 */
	// Deactivate the slave on startup
	HAL_GPIO_WritePin(NSS_GPIO_Out_GPIO_Port,NSS_GPIO_Out_Pin,GPIO_PIN_SET);
	HAL_Delay(10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
			
		/*
		// Testing if button press works
		if(HAL_GPIO_ReadPin(ButtonPress_GPIO_Port,ButtonPress_Pin) == GPIO_PIN_RESET)
		{
			HAL_GPIO_WritePin(IntLED_GPIO_Port,IntLED_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(IntLED_GPIO_Port,IntLED_Pin, GPIO_PIN_RESET);
		}
		*/
		
		
		// Transmit SPI Instructions on a button press
		if(HAL_GPIO_ReadPin(ButtonPress_GPIO_Port,ButtonPress_Pin) == GPIO_PIN_RESET)
		{
			// Set internal LED high during SPI write/read operations
			HAL_GPIO_WritePin(IntLED_GPIO_Port,IntLED_Pin, GPIO_PIN_RESET);

			// Write Data
			//1. Pull SS Low - Activate
			HAL_GPIO_WritePin(NSS_GPIO_Out_GPIO_Port,NSS_GPIO_Out_Pin,GPIO_PIN_RESET);
			//2. Transmit write data instruction
			HAL_SPI_Transmit(&hspi1,(uint8_t *)&WriteLEDFreq_instr,1,10);
			//3. Write the data over the SPI bus
			HAL_SPI_Transmit(&hspi1,(uint8_t *)&SPI_LED,4,10);
			/* For floating point numbers
			HAL_SPI_Transmit(&hspi1,(uint8_t *)&SPI_LED1,4,10);
			HAL_SPI_Transmit(&hspi1,(uint8_t *)&SPI_LED2,4,10);
			HAL_SPI_Transmit(&hspi1,(uint8_t *)&SPI_LED3,4,10);
			HAL_SPI_Transmit(&hspi1,(uint8_t *)&SPI_LED4,4,10);
			*/
			//4. Pull SS High - Deactivate
			HAL_GPIO_WritePin(NSS_GPIO_Out_GPIO_Port,NSS_GPIO_Out_Pin,GPIO_PIN_SET);
			
			// Read Data (move left)
			//1. Pull SS Low - Activate
			HAL_GPIO_WritePin(NSS_GPIO_Out_GPIO_Port,NSS_GPIO_Out_Pin,GPIO_PIN_RESET);
			//2. Transmit move left assertion
			HAL_SPI_Transmit(&hspi1,(uint8_t *)&AssertLeft_instr,1,10);
			//3. Read the data to the SPIMove array
			HAL_SPI_Receive(&hspi1,(uint8_t *)SPIMove,1,10);
			//4. Pull SS High - Deactivate
			HAL_GPIO_WritePin(NSS_GPIO_Out_GPIO_Port,NSS_GPIO_Out_Pin,GPIO_PIN_SET);	

			// Read Data (move right)
			//1. Pull SS Low - Activate
			HAL_GPIO_WritePin(NSS_GPIO_Out_GPIO_Port,NSS_GPIO_Out_Pin,GPIO_PIN_RESET);
			//2. Transmit move right assertion
			HAL_SPI_Transmit(&hspi1,(uint8_t *)&AssertRight_instr,1,10);
			//3. Read the data to the SPIMove array
			HAL_SPI_Receive(&hspi1,(uint8_t *)SPIMove,1,10);
			//4. Pull SS High - Deactivate
			HAL_GPIO_WritePin(NSS_GPIO_Out_GPIO_Port,NSS_GPIO_Out_Pin,GPIO_PIN_SET);	
			
			// Arbitrary delay 
			HAL_Delay(500);
			// Turn off SPI LED after SPI operations
			HAL_GPIO_WritePin(IntLED_GPIO_Port,IntLED_Pin, GPIO_PIN_SET);
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
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IntLED_GPIO_Port, IntLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSS_GPIO_Out_GPIO_Port, NSS_GPIO_Out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IntLED_Pin */
  GPIO_InitStruct.Pin = IntLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IntLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ButtonPress_Pin */
  GPIO_InitStruct.Pin = ButtonPress_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ButtonPress_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NSS_GPIO_Out_Pin */
  GPIO_InitStruct.Pin = NSS_GPIO_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NSS_GPIO_Out_GPIO_Port, &GPIO_InitStruct);

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
