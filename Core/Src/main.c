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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "Fan_Controller.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Definice PID konstant (nutno doladit pro tvůj systém)
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
//void ADC_Calibration(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//int16_t i = 0;
uint16_t readValue;
uint16_t readValue2;
uint16_t readValue3;
uint16_t readValue4;
uint16_t rawValues[4];
uint16_t pwmValue;
uint16_t pwmValue1;
uint16_t pwmValue2;
uint16_t pwmValue3;
uint8_t convCompleted=0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        readValue  = rawValues[0];
        readValue2 = rawValues[1];
        readValue3 = rawValues[2];
        readValue4 = rawValues[3];
    }
}


int16_t fan_speed_global = 0;
int16_t fan_speed_global2 = 0;
int caseValue = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Initialise PID controller */
	/*PIDController pid = { PID_KP, PID_KI, PID_KD,
						  PID_TAU,
						  PID_LIM_MIN, PID_LIM_MAX,
						  PID_LIM_MIN_INT, PID_LIM_MAX_INT,
						  SAMPLE_TIME_S };

	PIDController_Init(&pid);*/

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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

    //HAL_ADC_Start(&hadc1);
    //HAL_ADCEx_Calibration_Start(&hadc1);
	/*TIM1 -> CCR1 = 10;
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);*/
	  //HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	/*ADC1 -> CR &= ~ ADC_CR_ADEN;

	ADC1 -> CR|= ADC_CR_ADCAL;
	while((ADC1 -> CR & ADC_CR_ADCAL)!=0)
	{}*/
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  init_fan(&(TIM1->CCR1), &(TIM1->CCR2), &(TIM1->CCR4), &(TIM1->CCR3));

  lcd_init();
  lcd_put_cur(0, 0);
  lcd_send_string("HELLO ");
  lcd_send_string("WORLD ");
  lcd_send_string("FROM");
  HAL_Delay(1000);
  lcd_put_cur(1, 0);
  lcd_send_string("CONTROLLERSTECH");
  HAL_Delay(2000);
  lcd_clear();
  //lcd_clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //ADC_Calibrate();
	 HAL_ADCEx_Calibration_Start(&hadc1);
	 HAL_Delay(10);
	 set_fan_speed(fan_speed_global);
	 set_fan_speed2(fan_speed_global2);
	 //CalculatePWM(&pwmValue, &pwmValue1, &pwmValue2, &pwmValue3);
	 HAL_ADC_Start_DMA(&hadc1, (uint32_t *) rawValues, 4);
	 HAL_Delay(10);  // Krátká prodleva pro stabilizaci


	  if (convCompleted) {
	      readValue = (uint16_t) rawValues[0];
	      readValue2 = (uint16_t) rawValues[1];
	      readValue3 = (uint16_t) rawValues[2];
	      readValue4 = (uint16_t) rawValues[3];
	      convCompleted = 0;  // Resetování flagu pro další měření
	  }
	  pwmValue = 100 - ((readValue * 500) / 4095);  // Inverzní vztah pro pwmValue
	  pwmValue1 = 100 - ((readValue2 * 500) / 4095);  // Inverzní vztah pro pwmValue1
	  pwmValue2 = 100 - ((readValue3 * 500) / 4095);
	  pwmValue3 = 100 - ((readValue4 * 500) / 4095);
	  //Hranice pwm signálu
	  if (pwmValue >60000){
			pwmValue = 1;
		}
	  if (pwmValue1 >60000){
	  			pwmValue1 = 1;
	  		}
	  if (pwmValue2 >60000){
	  			pwmValue2 = 1;
	  		}
	  if (pwmValue3 >60000){
	  			pwmValue3 = 1;
	  		}

	  // Logika otáčení motorků
	  if (readValue > readValue2 && readValue2 > readValue3 && readValue2 > readValue4)
	  {
	  	caseValue = 1;
	  }
	  else if (readValue2 > readValue && readValue2 > readValue3 && readValue2 > readValue4)
	  {
	  	caseValue = 2;
	  }
	  else if (readValue2 > readValue3 && readValue3 > readValue && readValue3 > readValue4)
	  {
	  	caseValue = 3;
	  }
	  else if (readValue3 > readValue2 && readValue2 > readValue && readValue2 > readValue4)
	  {
	  	caseValue = 4;
	  }
	  else if (readValue3 > readValue4 && readValue4 > readValue2 && readValue4 > readValue)
	  {
	  	caseValue = 5;
	  }
	  else if (readValue4 > readValue3 && readValue3 > readValue2 && readValue3 > readValue)
	  {
	  	caseValue = 6;
	  }
	  else if (readValue4 > readValue && readValue > readValue3 && readValue > readValue2)
	  {
	  	caseValue = 7;
	  }
	  else if (readValue > readValue4 && readValue4 > readValue2 && readValue4 > readValue3)
	  {
	  	caseValue = 8;
	  }

	  switch (caseValue) {
	      case 1:
	          fan_speed_global = pwmValue * 10;
	          fan_speed_global2 = pwmValue * 10;
	          break;
	      case 2:
	          fan_speed_global = -pwmValue1 * 10;
	          fan_speed_global2 = -pwmValue1 * 10;
	          break;
	      case 3:
	          fan_speed_global = pwmValue1 * 10;
	          fan_speed_global2 = pwmValue1 * 10;
	          break;
	      case 4:
	          fan_speed_global = -pwmValue2 * 10;
	          fan_speed_global2 = -pwmValue2 * 10;
	          break;
	      case 5:
	          fan_speed_global = pwmValue2 * 10;
	          fan_speed_global2 = pwmValue2 * 10;
	          break;
	      case 6:
	          fan_speed_global = -pwmValue3 * 10;
	          fan_speed_global2 = -pwmValue3 * 10;
	          break;
	      case 7:
	          fan_speed_global = pwmValue3 * 10;
	          fan_speed_global2 = pwmValue3 * 10;
	          break;
	      case 8:
	          fan_speed_global = -pwmValue * 10;
	          fan_speed_global2 = -pwmValue * 10;
	          break;
	      default:
	          fan_speed_global = 0;
	          fan_speed_global2 = 0;
	          break;
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void ADC_Calibrate(void) {
    // Ensure ADC is disabled before calibration
    if (HAL_ADC_DeInit(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    // Start ADC calibration
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    // Reinitialize ADC after calibration
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
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
