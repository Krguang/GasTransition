/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define GAS_START_LIMIT 100
#define GAS_LOW_LIMIT 300
#define GAS_HIGH_LIMIT 600

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint8_t messageSendFlag;

uint16_t LocalArray[10];

volatile uint16_t ADC_ConvertedValue[10];
uint32_t ADC_Average[10];

static uint16_t GetCRC16(uint8_t *arr_buff, uint8_t len) {  //CRC校验程序
	uint16_t crc = 0xFFFF;
	uint8_t i, j;
	for (j = 0; j < len; j++) {
		crc = crc ^ *arr_buff++;
		for (i = 0; i < 8; i++) {
			if ((crc & 0x0001) > 0) {
				crc = crc >> 1;
				crc = crc ^ 0xa001;
			}
			else
				crc = crc >> 1;
		}
	}
	return (crc);
}

static void gasCollect() {

	uint16_t gasTemp[7];

	for (uint8_t i = 0; i < 100; i++)
	{
		for (uint8_t j = 0; j < 7; j++) {
			ADC_Average[j] += ADC_ConvertedValue[j];
		}
	}

	for (uint8_t i = 0; i < 7; i++)
	{
		gasTemp[i] = ADC_Average[i] * 10 / 4095;
		ADC_Average[i] = 0;
	}

	if ((gasTemp[0] >= GAS_START_LIMIT) && (gasTemp[0] <= GAS_LOW_LIMIT))		//氧气欠压							
	{
		HAL_GPIO_WritePin(out1Low_GPIO_Port, out1Low_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(out1High_GPIO_Port, out1High_Pin, GPIO_PIN_RESET);

		SET_BIT(LocalArray[0], 1 << 0);
		CLEAR_BIT(LocalArray[0], 1 << 8);
	}
	else if (gasTemp[0] >= GAS_HIGH_LIMIT)							//氧气超压
	{
		HAL_GPIO_WritePin(out1Low_GPIO_Port, out1Low_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(out1High_GPIO_Port, out1High_Pin, GPIO_PIN_SET);

		SET_BIT(LocalArray[0], 1 << 8);
		CLEAR_BIT(LocalArray[0], 1 << 0);
	}
	else												//氧气正常
	{
		HAL_GPIO_WritePin(out1Low_GPIO_Port, out1Low_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(out1High_GPIO_Port, out1High_Pin, GPIO_PIN_RESET);

		CLEAR_BIT(LocalArray[0], 1 << 0);
		CLEAR_BIT(LocalArray[0], 1 << 8);
	}

	if ((gasTemp[1] >= GAS_START_LIMIT) && (gasTemp[1] <= GAS_LOW_LIMIT))		//氩气欠压							
	{
		HAL_GPIO_WritePin(out2Low_GPIO_Port, out2Low_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(out2High_GPIO_Port, out2High_Pin, GPIO_PIN_RESET);

		SET_BIT(LocalArray[0], 1 << 1);
		CLEAR_BIT(LocalArray[0], 1 << 9);
	}
	else if (gasTemp[1] >= GAS_HIGH_LIMIT)							//氩气超压
	{
		HAL_GPIO_WritePin(out2Low_GPIO_Port, out2Low_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(out2High_GPIO_Port, out2High_Pin, GPIO_PIN_SET);
		SET_BIT(LocalArray[0], 1 << 9);
		CLEAR_BIT(LocalArray[0], 1 << 1);
	}
	else												//氩气正常
	{
		HAL_GPIO_WritePin(out2Low_GPIO_Port, out2Low_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(out2High_GPIO_Port, out2High_Pin, GPIO_PIN_RESET);
		CLEAR_BIT(LocalArray[0], 1 << 1);
		CLEAR_BIT(LocalArray[0], 1 << 9);
	}


	if ((gasTemp[2] >= GAS_START_LIMIT) && (gasTemp[2] <= GAS_LOW_LIMIT))		//笑气欠压							
	{
		HAL_GPIO_WritePin(out3Low_GPIO_Port, out3Low_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(out3High_GPIO_Port, out3High_Pin, GPIO_PIN_RESET);
		SET_BIT(LocalArray[0], 1 << 2);
		CLEAR_BIT(LocalArray[0], 1 << 10);
	}
	else if (gasTemp[2] >= GAS_HIGH_LIMIT)							//笑气超压
	{
		HAL_GPIO_WritePin(out3Low_GPIO_Port, out3Low_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(out3High_GPIO_Port, out3High_Pin, GPIO_PIN_SET);
		SET_BIT(LocalArray[0], 1 << 10);
		CLEAR_BIT(LocalArray[0], 1 << 2);
	}
	else												//笑气正常
	{
		HAL_GPIO_WritePin(out3Low_GPIO_Port, out3Low_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(out3High_GPIO_Port, out3High_Pin, GPIO_PIN_RESET);
		CLEAR_BIT(LocalArray[0], 1 << 2);
		CLEAR_BIT(LocalArray[0], 1 << 10);
	}

	if ((gasTemp[3] >= GAS_START_LIMIT) && (gasTemp[3] <= GAS_LOW_LIMIT))		//氮气欠压							
	{
		HAL_GPIO_WritePin(out4Low_GPIO_Port, out4Low_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(out4High_GPIO_Port, out4High_Pin, GPIO_PIN_RESET);
		SET_BIT(LocalArray[0], 1 << 3);
		CLEAR_BIT(LocalArray[0], 1 << 11);
	}
	else if (gasTemp[3] >= GAS_HIGH_LIMIT)							//氮气超压
	{
		HAL_GPIO_WritePin(out4Low_GPIO_Port, out4Low_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(out4High_GPIO_Port, out4High_Pin, GPIO_PIN_SET);
		SET_BIT(LocalArray[0], 1 << 11);
		CLEAR_BIT(LocalArray[0], 1 << 3);
	}
	else												//氮气正常
	{
		HAL_GPIO_WritePin(out4Low_GPIO_Port, out4Low_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(out4High_GPIO_Port, out4High_Pin, GPIO_PIN_RESET);
		CLEAR_BIT(LocalArray[0], 1 << 3);
		CLEAR_BIT(LocalArray[0], 1 << 11);
	}


	if ((gasTemp[4] >= GAS_START_LIMIT) && (gasTemp[4] <= GAS_LOW_LIMIT))		//负压吸引欠压							
	{
		HAL_GPIO_WritePin(out5Low_GPIO_Port, out5Low_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(out5High_GPIO_Port, out5High_Pin, GPIO_PIN_RESET);
		SET_BIT(LocalArray[0], 1 << 4);
		CLEAR_BIT(LocalArray[0], 1 << 12);
	}
	else if (gasTemp[4] >= GAS_HIGH_LIMIT)							//负压吸引超压
	{
		HAL_GPIO_WritePin(out5Low_GPIO_Port, out5Low_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(out5High_GPIO_Port, out5High_Pin, GPIO_PIN_SET);
		SET_BIT(LocalArray[0], 1 << 12);
		CLEAR_BIT(LocalArray[0], 1 << 4);
	}
	else												//负压吸引正常
	{
		HAL_GPIO_WritePin(out5Low_GPIO_Port, out5Low_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(out5High_GPIO_Port, out5High_Pin, GPIO_PIN_RESET);
		CLEAR_BIT(LocalArray[0], 1 << 4);
		CLEAR_BIT(LocalArray[0], 1 << 12);
	}

	if ((gasTemp[5] >= GAS_START_LIMIT) && (gasTemp[5] <= GAS_LOW_LIMIT))		//压缩空气欠压							
	{
		HAL_GPIO_WritePin(out6Low_GPIO_Port, out6Low_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(out6High_GPIO_Port, out6High_Pin, GPIO_PIN_RESET);
		SET_BIT(LocalArray[0], 1 << 5);
		CLEAR_BIT(LocalArray[0], 1 << 13);
	}
	else if (gasTemp[5] >= GAS_HIGH_LIMIT)							//压缩空气超压
	{
		HAL_GPIO_WritePin(out6Low_GPIO_Port, out6Low_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(out6High_GPIO_Port, out6High_Pin, GPIO_PIN_SET);
		SET_BIT(LocalArray[0], 1 << 13);
		CLEAR_BIT(LocalArray[0], 1 << 5);
	}
	else												//压缩空气正常
	{
		HAL_GPIO_WritePin(out6Low_GPIO_Port, out6Low_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(out6High_GPIO_Port, out6High_Pin, GPIO_PIN_RESET);
		CLEAR_BIT(LocalArray[0], 1 << 5);
		CLEAR_BIT(LocalArray[0], 1 << 13);
	}

	if ((gasTemp[6] >= GAS_START_LIMIT) && (gasTemp[6] <= GAS_LOW_LIMIT))		//二氧化碳欠压							
	{
		HAL_GPIO_WritePin(out7Low_GPIO_Port, out7Low_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(out7High_GPIO_Port, out7High_Pin, GPIO_PIN_RESET);
		SET_BIT(LocalArray[0], 1 << 6);
		CLEAR_BIT(LocalArray[0], 1 << 14);
	}
	else if (gasTemp[6] >= GAS_HIGH_LIMIT)							//二氧化碳超压
	{
		HAL_GPIO_WritePin(out7Low_GPIO_Port, out7Low_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(out7High_GPIO_Port, out7High_Pin, GPIO_PIN_SET);
		SET_BIT(LocalArray[0], 1 << 14);
		CLEAR_BIT(LocalArray[0], 1 << 6);
	}
	else												//二氧化碳正常
	{
		HAL_GPIO_WritePin(out7Low_GPIO_Port, out7Low_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(out7High_GPIO_Port, out7High_Pin, GPIO_PIN_RESET);
		CLEAR_BIT(LocalArray[0], 1 << 6);
		CLEAR_BIT(LocalArray[0], 1 << 14);
	}
}

static void sendDataMaster16() {

	uint16_t temp;
	uint8_t i;
	uint8_t txBuf[32];
	uint8_t txCount;

	txBuf[0] = 1;
	txBuf[1] = 0x10;
	txBuf[2] = 0x00;         //数据的起始地址；
	txBuf[3] = 0x00;
	txBuf[4] = 0x00;         //数据的个数；
	txBuf[5] = 0x08;
	txBuf[6] = 0x10;         //数据的字节数；
	for (i = 0; i<txBuf[5]; i++) {
		txBuf[7 + 2 * i] = (uint8_t)(LocalArray[i + txBuf[3]] >> 8);
		txBuf[8 + 2 * i] = (uint8_t)(LocalArray[i + txBuf[3]] & 0xff);
	}
	temp = GetCRC16(txBuf, 2 * txBuf[5] + 7);
	txBuf[7 + 2 * txBuf[5]] = (uint8_t)(temp & 0xff);
	txBuf[8 + 2 * txBuf[5]] = (uint8_t)((temp >> 8) & 0xff);
	txCount = 9 + 2 * txBuf[5];
	HAL_UART_Transmit(&huart1, txBuf, txCount, 0xffff);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_ConvertedValue, 7);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  gasCollect();

	  if (messageSendFlag)
	  {
		  messageSendFlag = 0;
		  sendDataMaster16();
	  }
	  HAL_IWDG_Refresh(&hiwdg);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3) {

			messageSendFlag = 1;
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
