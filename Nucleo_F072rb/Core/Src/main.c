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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

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
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

void enableAlarm(void);
void disableAlarm(void);
void sendAlarm(void);
void sendOk(void);
void setGreenColor(void);
void setBlueColor(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t  InputState =  GPIO_PIN_SET;
uint8_t  EnableFlag = 0;
uint8_t  CounterBounce = 0;

char string_alarm[] = "ALARM !!! ALARM !!!\r\n";

char string_ok[] = "OK\r\n";


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
  SystemClock_Config();            // конфигурация системы тактирования всего микроконтроллера

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();       // Инициализация всех пинов используемых в данном проекте
  MX_USART2_UART_Init(); // Инициализация UART2
// MX_USB_DEVICE_Init();
  MX_TIM6_Init();       // Инициализация таймера TIM6
  MX_TIM7_Init();       // Инициализация таймера TIM7
  /* USER CODE BEGIN 2 */
/*
 * старт таймеров для использования в программе,
 * после инициализации обработка событий обоих таймеров производится
 * в функции void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
 *
 */
  HAL_TIM_Base_Start_IT(&htim6);    //  стартуем работу и наступление событий по прерванию от TIM6
  HAL_TIM_Base_Start_IT(&htim7);    //  стартуем работу и наступление событий по прерванию от TIM7


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // в while  отправлем события в UART, все остальные действия выполняем в прерываниях.
  while (1)
  {
	  if(EnableFlag)      // флаг наступления события тревоги, флаг выставляется в обработчике прерывания
	  {

		setGreenColor();  // выставить зеленый цвет для вывода отладочной информации в UART

		sendAlarm();      // отправка события "ALARM" в UART

	    HAL_Delay(1000);  // пауза в 1000 ms
	  }

	  else
	  {

		  setBlueColor();  // выставить голубой цвет для вывода отладочной информации в UART

		  sendOk();        // отправка события "OK" в UART

		  HAL_Delay(2000); // пауза в 2000 ms
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47999;                // делим частоту 48.000.000 на 48000 и получаем 1000 Гц, это тактирование таймера раз 1 ms
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP; // счетчик испульсов считает вверх
  htim6.Init.Period = 9;                       // отсчитываем 10 тактов, то есть 10 ms, и каждые 10 ms выдаем событие
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 47999;              // делим частоту 48.000.000 на 48000 и получаем 1000 Гц, это тактирование таймера  раз 1 ms
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 499;                    // отсчитываем 10 тактов, то есть 500 ms, и каждые 500 ms выдаем событие
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart2.Init.BaudRate = 115200;                 // скорость работы UART 9600
  huart2.Init.WordLength = UART_WORDLENGTH_8B; // длина слова 8 бит
  huart2.Init.StopBits = UART_STOPBITS_1;      // 1 stop bit
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();  //тактирование порта С
  __HAL_RCC_GPIOF_CLK_ENABLE();  //тактирование порта F
  __HAL_RCC_GPIOA_CLK_ENABLE();  //тактирование порта A
  __HAL_RCC_GPIOB_CLK_ENABLE();  //тактирование порта B

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // пин выставлем в LOW

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, USER_PIEZO_ELEMENT_Pin|USER_LED_Pin|POWER_INDICATION_Pin, GPIO_PIN_RESET); // пины выставлем в LOW

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_INPUT_GERKON_Pin */
  GPIO_InitStruct.Pin = USER_INPUT_GERKON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull =  GPIO_PULLUP;   // подтянут к вверху , то есть 3.3 вольта
  HAL_GPIO_Init(USER_INPUT_GERKON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USER_PIEZO_ELEMENT_Pin USER_LED_Pin POWER_INDICATION_Pin */
  GPIO_InitStruct.Pin = USER_PIEZO_ELEMENT_Pin|USER_LED_Pin|POWER_INDICATION_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/**
* @brief  HAL_TIM_PeriodElapsedCallback
* @param  structure of timer
* @retval None
*/


/*
 * данная функция вызывается каждый раз  при наступлении события от
 * таймеров TIM6 и TIM7, в ней производится обработка события, то есть выполняются необходимые нам действия
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /*  timer6 for button
   *  для отслеживания нажатия кнопки и
   *  устранения дребезга, если кнопку
   *  нажимали менее 50 ms- то это дребезг
   *  попадаем в этом в обработчик раз в 10 ms
  */
  if(htim->Instance == TIM6) // попадаем в данный if каждые 10 ms, так настроен таймер TIM6
  {
	  InputState = HAL_GPIO_ReadPin(USER_INPUT_GERKON_GPIO_Port,USER_INPUT_GERKON_Pin);

	   if(InputState==GPIO_PIN_RESET)
	     {
		     CounterBounce++; //  счетчик отрезков времени. считаем кол-во  отрезков времени по 10 ms
	     }

	     else
	     {
		     CounterBounce = 0;  // любой дребезг - сброс счетчика отрезков времени.
		     EnableFlag = 0;
	     }


	  if(CounterBounce > 5) // >=50 ms
	     {
	       EnableFlag = 1;   // выставляем флаг для включения тревоги
	     }

  }

  /*
   *  timer7 for alarm
   *
  */
  else if(htim->Instance == TIM7) // попадаем сюда каждые 500 ms,  так настроен таймер TIM7
  {

	    if(EnableFlag)     // если флага в 1
	      {
		   enableAlarm(); //
	      }
	    else // если флага в 0
	      {
		   disableAlarm();
	      }
   }

}


void enableAlarm(void)
{

   HAL_GPIO_TogglePin(GPIOB,USER_PIEZO_ELEMENT_Pin); // звук

   HAL_GPIO_TogglePin(GPIOB,USER_LED_Pin);           // свет
}

void disableAlarm(void )
{
	HAL_GPIO_WritePin(GPIOB,USER_PIEZO_ELEMENT_Pin ,GPIO_PIN_RESET );

	HAL_GPIO_WritePin(GPIOB,USER_LED_Pin ,GPIO_PIN_RESET);
}

void sendAlarm(void)
{
    HAL_UART_Transmit(&huart2,(uint8_t *)string_alarm, (uint16_t)strlen(string_alarm),200);
}


void sendOk(void)
{
	 HAL_UART_Transmit(&huart2,(uint8_t *)string_ok, (uint16_t)strlen(string_ok),200);
}

void setGreenColor(void)
{
	HAL_UART_Transmit(&huart2,(uint8_t *)"\033[31m\0", (uint16_t)strlen("\033[31m\0"),200);
}

void setBlueColor(void)
{
	  HAL_UART_Transmit(&huart2,(uint8_t *)"\033[34m\0", (uint16_t)strlen("\033[34m\0"),200);
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
