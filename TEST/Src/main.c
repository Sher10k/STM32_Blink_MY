
/* Defined -------------------------------------------------------------------*/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"

/* Private variables ---------------------------------------------------------*/
char click0=1, k=1;
volatile uint32_t ticks_delay = 0;
volatile uint8_t flag_Rx = 1, flag_Tx = 1;
volatile uint8_t data = 1;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
void Delay2(uint32_t t);
void delay_ms(uint32_t milliseconds);

int main(void)
{

	uint8_t str[] = "Hello";

	// Initialization
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART1_UART_Init();

	HAL_UART_Transmit_DMA(&huart1, &data, 1);
	HAL_UART_Receive_DMA(&huart1, &data, 1);
	// For SysTick
	// SysTick
	/*SysTick->LOAD = SystemCoreClock/1000-1;		// Загрузка значения. Нам нужен килогерц
	SysTick->VAL = 0x0;		// Обнуляем таймеры и флаги. Записью, помните?
	SysTick->CTRL =	SysTick_CTRL_CLKSOURCE_Msk |
					SysTick_CTRL_TICKINT_Msk;*/

	//HAL_UART_Receive_IT(&huart1, &data, 1);
	while (1)
	{
		//HAL_UART_Transmit_DMA(&huart1, &data, 1);
		/*if (flag_Rx == 0) HAL_UART_Receive_DMA(&huart1, &data, 1);
		flag_Rx = 1;*/
		if (data == 1) GPIOC->ODR ^= GPIO_ODR_ODR13;

		//HAL_UART_Transmit(&huart1, str, 5, 1000);

		/*if (((GPIOA->IDR & (GPIO_IDR_IDR0)) == 0) && (click0 != 0))
		{
			//GPIOC->ODR ^= GPIO_ODR_ODR13;
			k ^= 1;
		}
		click0 = (GPIOA->IDR & (GPIO_IDR_IDR0));*/

		//GPIOC->ODR ^= GPIO_ODR_ODR13;
		//delay_ms(1000);
		HAL_Delay(1000);

		/*if (huart1.RxXferCount == 0)
		{
			//HAL_UART_Transmit(&huart1, str, 5, 1000);
			HAL_UART_Transmit(&huart1, &data, 1, 1000);
			if (data == 1)
			{
				k = 1;
				//GPIOC->BSRR |= GPIO_BSRR_BR13;
			} else
			{
				k = 0;
				GPIOC->BSRR |= GPIO_BSRR_BS13;
			}
			if (k == 1) GPIOC->ODR ^= GPIO_ODR_ODR13;
			HAL_UART_Receive_IT(&huart1, &data, 1);
		}*/

		/*if (k == 0)
		{
			GPIOC->BSRR |= GPIO_BSRR_BS13;
			Delay2(500000);
			GPIOC->BSRR |= GPIO_BSRR_BR13;
			Delay2(500000);
		}*/
	}
}


void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/* функция задержки */
void Delay2(uint32_t t)
{
  for (uint32_t i = 0; i < t; i++)
  {
    asm("NOP");
  }
}
void delay_ms(uint32_t milliseconds)
{
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;	// Запуск системного счетчика
	uint32_t start = ticks_delay;
	while ((ticks_delay - start) < milliseconds);
	SysTick->CTRL ^= SysTick_CTRL_ENABLE_Msk;	// Остановка системного счетчика
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
