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
#include "comp.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"
#include "stdio.h"

#include "led.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Definition of ADCx conversions data table size */
#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  32)   /* Size of array aADCxConvertedData[] */

#define TXBUFFERSIZE 128
#define READING_INDEX_LENGTH 4


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Variable containing ADC conversions data */
static uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];
static uint16_t avgPA1;
/* The comparator sets/resets this variable when comparing PA0 & PA1 */
volatile uint8_t comparator = 0;


/* Buffer used for debug UART */
char tx_buffer[TXBUFFERSIZE];
uint8_t msg_id = 0;

uint32_t tx_last;
uint32_t panel_mv = 0;
uint32_t battery_mv = 0;

LED_HandleTypeDef led1;
LED_HandleTypeDef led2;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_ADC_Init();
  MX_COMP1_Init();
  /* USER CODE BEGIN 2 */


  LED_Init(&led1, GPIOC, LED_1_Pin);
  led1.onDuration = 100;
  led1.offDuration = 900;

  LED_Init(&led2, GPIOC, LED_2_Pin);
  led2.offDuration = 500;
  LED_on(&led2);

  if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) !=  HAL_OK)
  {
    Error_Handler();
  }

  HAL_COMP_Start(&hcomp1);

  /* Start conversion in DMA mode ################################# */
  if (HAL_ADC_Start_DMA(&hadc,
                        (uint32_t *)aADCxConvertedData,
                        ADC_CONVERTED_DATA_BUFFER_SIZE
                       ) != HAL_OK)
  {
    Error_Handler();
  }

  htim2.Instance->CCR1 = 0;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);


  // This is the driver pin for connecting the battery to the adc.
  // So the battery can always be connected but present no voltage
  // to the adc until it is powered.
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  uint32_t tmpAvgPA1 = 0;
	  uint16_t i = 0;
	  while (i < ADC_CONVERTED_DATA_BUFFER_SIZE) {
		  /* PA1 is the battery */
		  tmpAvgPA1 += aADCxConvertedData[i];
		  i++;
	  }
	  // Use the average.
 	  avgPA1 = tmpAvgPA1 / ADC_CONVERTED_DATA_BUFFER_SIZE;

	  // 2330 = 13519 = 5.802145923 per bit
	  battery_mv = (avgPA1 * 5802) / 1000;

	  // If the comparator says the panel is a higher voltage than the battery
	  // then charge.
	  if (comparator) {
		  if (battery_mv > 13501) {
			  if (htim2.Instance->CCR1 > 0) {
				  htim2.Instance->CCR1 -= 1;
			  }
		  } else if (battery_mv < 13499){
			  if (htim2.Instance->CCR1 < 1000) {
				  htim2.Instance->CCR1 += 1;
			  }
		  }
	  } else {
		  // Not enough to do charging.
		  htim2.Instance->CCR1 = 0;
	  }


	  /* Set the leds */
	  if (htim2.Instance->CCR1 == 1000) {
		  LED_on(&led2);
	  } else if (htim2.Instance->CCR1 == 0) {
		  LED_off(&led2);
	  } else {
		  led2.onDuration = htim2.Instance->CCR1;
		  LED_flash(&led2);
	  }

	  /* A continual flash to indicate powered on */
	  LED_flash(&led1);


//	  uint32_t now = HAL_GetTick();
//	  if (now - tx_last >= 500) {
//		  tx_last = now;
//		  int tx_len = snprintf(
//				  tx_buffer,
//				  TXBUFFERSIZE,
//				  "msg_id:%d, batt:%lu, panel:%lu, ccr:%lu \n",
//				  msg_id++,
//				  battery_mv,
//				  panel_mv,
//				  htim2.Instance->CCR1
//		  );
//		  HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, tx_len, 500);
//	  }



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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
	if (HAL_COMP_GetOutputLevel(hcomp)) {
		comparator = 1;
	} else {
		comparator = 0;
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
