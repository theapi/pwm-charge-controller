/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"
#include "stdio.h"

/* External ADC */
#include "ads1015.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TXBUFFERSIZE 128
#define READING_INDEX_LENGTH 255

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

/* Buffer used for debug UART */
char tx_buffer[TXBUFFERSIZE];
uint8_t msg_id = 0;
//uint32_t batt = 0;
//uint32_t ccr = 0;
//uint32_t power = 100;
//uint32_t alpha = 70;
//uint32_t movingAverage = 0;


uint32_t reading_index = 0;
uint32_t batt_readings[READING_INDEX_LENGTH] = { 0 };
uint32_t panel_readings[READING_INDEX_LENGTH] = { 0 };
uint32_t tx_last;
uint32_t panel_mv = 0;
uint32_t battery_mv = 0;

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
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */


  HAL_GPIO_WritePin(GPIOC, LED_1_Pin|LED_2_Pin, GPIO_PIN_SET);
  // This is the driver pin for conecting the battery to the adc.
  // So the battery can always be connected but present no voltage
  // to the adc until it is powered.
  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);

  /* Calibrate the ADC */
  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
//  /* Start the adc */
//  HAL_ADC_Start(&hadc);

  /* Start the PWM which is configured for 10.4kHz
   *
   * Smooth with 4.6k & 1uF RC low pass filter.
   */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  htim2.Instance->CCR1 = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_ADC_Start(&hadc);

	  HAL_ADC_PollForConversion(&hadc, 100);
	  uint32_t panel_val = HAL_ADC_GetValue(&hadc);
	  panel_readings[reading_index] = panel_val;
	  uint32_t panel_total = 0;
	  for (uint32_t i = 0; i < READING_INDEX_LENGTH; i++) {
		  panel_total += panel_readings[i];
	  }
	  uint32_t panel = panel_total / READING_INDEX_LENGTH;
	  // 866 = 5039   = 5.818706697 per bit
	  // 2233 = 12995 = 5.819525302 per bit
	  panel_mv = (panel * 5819) / 1000;

	  HAL_ADC_PollForConversion(&hadc, 100);
	  uint32_t batt_val = HAL_ADC_GetValue(&hadc);
	  batt_readings[reading_index] = batt_val;
	  uint32_t batt_total = 0;
	  for (uint32_t i = 0; i < READING_INDEX_LENGTH; i++) {
		  batt_total += batt_readings[i];
	  }
	  uint32_t battery = batt_total / READING_INDEX_LENGTH;

	  // 706 = 4084   = 5.7847025   per bit
	  // 2225 = 12870 = 5.784269663 per bit
	  battery_mv = (battery * 5784) / 1000;

	  if (reading_index < READING_INDEX_LENGTH) {
		  reading_index++;
	  } else {
		  reading_index = 0;
	  }

	  HAL_ADC_Stop(&hadc);



	  if (panel_mv > 12000) {
		  if (battery_mv > 13550) {
			  HAL_GPIO_WritePin(GPIOC, LED_1_Pin, GPIO_PIN_SET);
			  //HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
			  if (htim2.Instance->CCR1 > 0) {
				  htim2.Instance->CCR1 -= 1;
			  }
		  } else if (battery_mv < 13490){
			  HAL_GPIO_WritePin(GPIOC, LED_1_Pin, GPIO_PIN_RESET);
			  //HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
			  if (htim2.Instance->CCR1 < 253) {
				  htim2.Instance->CCR1 += 1;
			  }
		  }
	  } else {
		  // Not enough to do charging.
		  htim2.Instance->CCR1 = 0;
	  }

	  HAL_Delay(250);


	  uint32_t now = HAL_GetTick();
	  if (now - tx_last >= 250) {
		  tx_last = now;
		  int tx_len = snprintf(
				  tx_buffer,
				  TXBUFFERSIZE,
				  "msg_id:%d, batt:%lu, panel:%lu, ccr:%lu \n",
				  msg_id++,
				  battery_mv,
				  panel_mv,
				  htim2.Instance->CCR1
		  );
		  HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, tx_len, 500);
	  }



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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
