/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BME280.h"
#include "stdio.h"
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
BME280_t bme;
uint8_t hc12_data[100],pms_data[100],f=0,p_flag=0,k=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void ClearBuffer();
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  //NVIC_SystemReset();
  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);

  while(bme.p6!=-7)
  {
	  BME280_Init(&bme, &hi2c2, 0x76);
	  HAL_Delay(1000);
  }

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, hc12_data, 100);
  HAL_GPIO_WritePin(PMS3003_set_GPIO_Port, PMS3003_set_Pin, 0);
  float t,p,h;
  uint32_t Time_ToStart=0,Time_ToSend=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(p_flag)
	  {
		  p_flag=0;
		  Time_ToStart=HAL_GetTick();
		  f=1;
		  ClearBuffer();
		  HAL_GPIO_WritePin(PMS3003_set_GPIO_Port, PMS3003_set_Pin, 1);
		  HAL_UART_Transmit(&huart2, (uint8_t*)"start\n", sizeof("start\n"), 1000);
	  }
	  if(f)
	  {
		  if(HAL_GetTick()-Time_ToStart>90000)
		  {
			  f=0;
			  HAL_UART_Transmit(&huart2, (uint8_t*)"oczekiwanie\n", sizeof("oczekiwanie\n"), 1000);
			  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, pms_data, 100);
		  }
		  if(HAL_GetTick()-Time_ToSend>10000)
		  {
			  uint8_t time=(90000-HAL_GetTick()+Time_ToStart)/1000;
			  uint8_t l=sprintf((char*)hc12_data,"time %d",time);
			  HAL_UART_Transmit(&huart3, hc12_data, l, 1000);
			  HAL_UART_Transmit(&huart2, hc12_data, l, 1000);
			  HAL_UART_Transmit(&huart2, (uint8_t*)"\n", sizeof("\n"), 1000);
			  Time_ToSend=HAL_GetTick();
		  }
	  }
	  if(k)
	  {
		 BME280_Read_all(&bme, &t, &p, &h);
		 k=0;
		 uint8_t l=sprintf((char*)hc12_data,"PM1: %d  PM2.5: %d  PM10: %d  temp: %0.2f  press: %0.2f  hum: %0.2f",pms_data[5],pms_data[7],pms_data[9],t,p,h);
		 HAL_UART_Transmit(&huart2, hc12_data, l, 1000);
		 HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, 1000);
		 uint8_t *j;
		 for(int i=0;i<100;i++)
		 {
			 hc12_data[i]=0;
		 }
		 hc12_data[0]=pms_data[4];
		 hc12_data[1]=pms_data[5];
		 hc12_data[2]=pms_data[6];
		 hc12_data[3]=pms_data[7];
		 hc12_data[4]=pms_data[8];
		 hc12_data[5]=pms_data[9];
		 j=&t;
		 hc12_data[6]=*j;
		 j++;
		 hc12_data[7]=*j;
		 j++;
		 hc12_data[8]=*j;
		 j++;
		 hc12_data[9]=*j;
		 j=&p;
		 hc12_data[10]=*j;
		 j++;
		 hc12_data[11]=*j;
		 j++;
		 hc12_data[12]=*j;
		 j++;
		 hc12_data[13]=*j;
		 j=&h;
		 hc12_data[14]=*j;
		 j++;
		 hc12_data[15]=*j;
		 j++;
		 hc12_data[16]=*j;
		 j++;
		 hc12_data[17]=*j;
		 HAL_UART_Transmit(&huart3, hc12_data, 18, 1000);
	  }
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
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance==USART3)
	{
		p_flag=1;
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, hc12_data, 100);
	}
	else if(huart->Instance==USART1)
	{
		if(pms_data[0]!=0x42)
		{
			ClearBuffer();
			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, pms_data, 100);
		}
		else
		{
			k=1;
			HAL_GPIO_WritePin(PMS3003_set_GPIO_Port, PMS3003_set_Pin, 0);
		}
	}
}
void ClearBuffer()
{
	for(int i=0;i<100;i++)
	{
		pms_data[i]=0;
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
