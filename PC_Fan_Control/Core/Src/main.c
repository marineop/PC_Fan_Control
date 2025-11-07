/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include <stdbool.h>
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

#define PWM_INCREMENT				(10)
#define PWM_MAX_DUTY_CYCLE			(1000)
#define PRINT_RPM_INTERVAL			(200)			// ms
#define MAX_DEBOUNCING_COUNTER		(10)			// 0.1 ms x 10 = 1 ms
#define MIN_DEBOUNCING_COUNTER		(0)

#define DEBOUNCING_SET_BIT_MASK		(0x200)
#define DEBOUNCING_MASK				(0x3FF)
#define DEBOUNCING_TOTAL			(10)
#define DEBOUNCING_THRESHOLD		(8)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

volatile GPIO_PinState pb0Old = GPIO_PIN_RESET;
volatile GPIO_PinState pb5Old = GPIO_PIN_RESET;

uint32_t pb0History = 0;
uint32_t pb5History = 0;

volatile int32_t dutyCycle = 1000;

volatile bool captured = false;
volatile uint32_t lastCaptureValue = 0;
volatile uint32_t captureValue = 0;

volatile int64_t timeStamp;
int64_t lastPrintTime = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void UpdateSpeed(GPIO_PinState aLast, GPIO_PinState bLast, GPIO_PinState aNew, GPIO_PinState bNew);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim16)
	{
		lastCaptureValue = captureValue;
		captureValue = htim16.Instance->CCR1;
		captured = true;
	}
}

uint32_t UpdatePinHistory(uint32_t history, GPIO_PinState newState)
{
	history >>= 1;
	if(newState == GPIO_PIN_RESET)
	{
		history |= DEBOUNCING_SET_BIT_MASK;
	}
	else
	{
		history &= (~DEBOUNCING_SET_BIT_MASK);
	}

	history &= DEBOUNCING_MASK;

	return history;
}

GPIO_PinState CheckStateChange(GPIO_PinState currentState, uint32_t history)
{
	GPIO_PinState newState;

	if(currentState == GPIO_PIN_RESET)
	{
		if(__builtin_popcount(history) >= DEBOUNCING_THRESHOLD)
		{
			newState = GPIO_PIN_SET;
		}

	}
	else
	{
		if(__builtin_popcount(history) <= (DEBOUNCING_TOTAL - DEBOUNCING_THRESHOLD))
		{
			newState = GPIO_PIN_RESET;
		}
	}

	return newState;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3)
	{
		GPIO_PinState pb0Current = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
		GPIO_PinState pb5Current = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);

		pb0History = UpdatePinHistory(pb0History, pb0Current);
		pb5History = UpdatePinHistory(pb5History, pb5Current);

		GPIO_PinState pb0New = CheckStateChange(pb0Current, pb0History);
		GPIO_PinState pb5New = CheckStateChange(pb5Current, pb5History);;

		if(pb0Old != pb0New || pb5Old != pb5New)
		{
			UpdateSpeed(pb0Old, pb5Old, pb0New, pb5New);

			pb0Old = pb0New;
			pb5Old = pb5New;
		}
	}
}

void UpdateSpeed(GPIO_PinState aLast, GPIO_PinState bLast, GPIO_PinState aNew, GPIO_PinState bNew)
{
	int32_t direction = 0;

	if (aLast == GPIO_PIN_RESET) // 0
	{
		if (bLast == GPIO_PIN_RESET) // 0
		{
			if (aNew == GPIO_PIN_RESET) // 0
			{
				if (bNew == GPIO_PIN_RESET)
				{
				}
				else // 1
				{
					direction = -1; // 0 0 0 1
				}
			}
			else // 1
			{
				if (bNew == GPIO_PIN_RESET) // 0
				{
					direction = 1; // 0 0 1 0
				}
				else
				{
				}
			}
		}
		else // 1
		{
			if (aNew == GPIO_PIN_RESET) // 0
			{
				if (bNew == GPIO_PIN_RESET) // 0
				{
					direction = 1; // 0 1 0 0
				}
				else
				{
				}
			}
			else
			{
				if (bNew == GPIO_PIN_RESET)
				{
				}
				else
				{
					direction = -1; // 0 1 1 1
				}
			}
		}
	}
	else // 1
	{
		if (bLast == GPIO_PIN_RESET) // 0
		{
			if (aNew == GPIO_PIN_RESET) // 0
			{
				if (bNew == GPIO_PIN_RESET)
				{
					direction = -1; // 1 0 0 0
				}
				else
				{
				}
			}
			else // 1
			{
				if (bNew == GPIO_PIN_RESET) // 0
				{
				}
				else
				{
					direction = 1; // 1 0 1 1
				}
			}
		}
		else // 1
		{
			if (aNew == GPIO_PIN_RESET) // 0
			{
				if (bNew == GPIO_PIN_RESET) // 0
				{
				}
				else
				{
					direction = 1; // 1 1 0 1
				}
			}
			else
			{
				if (bNew == GPIO_PIN_RESET)
				{
					direction = -1; // 1 1 1 0
				}
				else
				{
				}
			}
		}
	}

	if (direction > 0)
	{
		dutyCycle += PWM_INCREMENT;
		if (dutyCycle > PWM_MAX_DUTY_CYCLE)
		{
			dutyCycle = PWM_MAX_DUTY_CYCLE;
		}

		htim14.Instance->CCR1 = dutyCycle;
	}
	else if (direction < 0)
	{
		dutyCycle -= PWM_INCREMENT;
		if (dutyCycle < 0)
		{
			dutyCycle = 0;
		}

		htim14.Instance->CCR1 = dutyCycle;
	}
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
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	pb0Old = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	pb5Old = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);

	if(pb0Old == GPIO_PIN_SET)
	{
		pb0History = DEBOUNCING_MASK;
	}

	if(pb5Old == GPIO_PIN_SET)
	{
		pb5History = DEBOUNCING_MASK;
	}

	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1); // Fan PWM
	HAL_TIM_IC_Start_IT(&htim16, TIM_CHANNEL_1); // Tachometer PA6
	HAL_TIM_Base_Start_IT(&htim3); // Encoder PB0, PB5

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if (captured)
		{
			captured = false;

			if (timeStamp - lastPrintTime > PRINT_RPM_INTERVAL)
			{
				lastPrintTime = timeStamp;

				int32_t diff = (int32_t)captureValue - (int32_t)lastCaptureValue;
				if (diff < 0)
				{
					diff += 65536;
				}

				int32_t rpm = (60 / 2 * 10000) / diff;

				printf("RPM: %ld, dutyCycle: %ld\r\n", rpm, dutyCycle);
			}
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2500;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 2500-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim16, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
