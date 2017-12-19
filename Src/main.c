/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */

#define HAL

#define LED_RED 	TIM_CHANNEL_1
#define LED_GREEN 	TIM_CHANNEL_2
#define LED_BLUE 	TIM_CHANNEL_4

#define FADE_DELAY	25

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t Power = 0;

uint32_t adc[2];

uint32_t color;
uint32_t luminosity;

uint64_t PwmDutyRed;
uint64_t PwmDutyGreen;
uint64_t PwmDutyBlue;

uint32_t test;			// a test variable

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#ifdef HAL




/**
  * @brief  Sets Duty to TIM PWM
  * @param  PulseLength 0 - 2^16-1
  * @param  Channel TIM Channels to be configured
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  * @retval none
  */
void setPwmDuty(uint32_t PulseLength, uint32_t Channel)
{
	HAL_TIM_PWM_Stop(&htim3,Channel);

	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = PulseLength;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, Channel) != HAL_OK)
	{
	  _Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_PWM_Start(&htim3,Channel);
}


/**
  * @brief  Fade-in of a choosen LED color with a time delay
  * @param  TimeDiv		devides the additional PWM pulse to extend the fade-in time (1 - 1000)
  * @param  Led 		chose a led color to select
  *          This parameter can be one of the following values:
  *            @arg LED_RED:   Red   LEDs are selected
  *            @arg LED_GREEN: Green LEDs are selected
  *            @arg LED_BLUE:  Blue  LEDs are selected
  * @retval none
  */
void fadeInLed(uint16_t TimeDiv, uint32_t Led)
{
	uint32_t Channal;
	uint64_t PwmDuty = TimeDiv;

	if(TimeDiv > 1000)
		TimeDiv = 1000;

	Channal = Led;

	while(PwmDuty/TimeDiv < 65535)
	{
		setPwmDuty(PwmDuty/TimeDiv,Channal);
		PwmDuty = (PwmDuty + PwmDuty/TimeDiv);
		HAL_Delay(25);
		}

}


/**
  * @brief  Fade-out of a choosen LED color with a time delay
  * @param  TimeDiv		devides the additional PWM pulse to extend the fade-in time (1 - 1000)
  * @param  Led 		chose a led color to select
  *          This parameter can be one of the following values:
  *            @arg LED_RED:   Red   LEDs are selected
  *            @arg LED_GREEN: Green LEDs are selected
  *            @arg LED_BLUE:  Blue  LEDs are selected
  * @retval none
  */
void fadeOutLed(uint16_t TimeDiv, uint32_t Led)
{
	uint32_t Channal;
	uint64_t PwmDuty = 65535*TimeDiv;

	if(TimeDiv > 1000)
		TimeDiv = 1000;

	Channal = Led;

	while(PwmDuty/TimeDiv > TimeDiv)
	{
		setPwmDuty(PwmDuty/TimeDiv,Channal);
		PwmDuty = (PwmDuty - PwmDuty/TimeDiv);
		HAL_Delay(25);
	}
	setPwmDuty(0,Channal);


}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//Power = 1 - Power;
	switch(GPIO_Pin)
	{
	case GPIO_PIN_8:
		HAL_GPIO_WritePin(PwmRed_Tim3Ch1_GPIO_Port,PwmRed_Tim3Ch1_Pin, 1);
		break;

	case GPIO_PIN_9:
		HAL_GPIO_WritePin(PwmGreen_Tim3Ch2_GPIO_Port, PwmGreen_Tim3Ch2_Pin, 1);
		break;

	case GPIO_PIN_10:
		HAL_GPIO_WritePin(PwmBlue_Tim3Ch4_GPIO_Port,PwmBlue_Tim3Ch4_Pin, 1);
		break;

	default:
		break;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	color = adc[0];
	luminosity = adc[1];
}


#endif

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */



  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc2, adc, 2);

//  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


#ifdef HAL
	  // light effect
	  if(Power == 1)
	  {

		  fadeOutLed(FADE_DELAY,LED_GREEN);
		  fadeInLed(FADE_DELAY,LED_RED);
		  fadeOutLed(FADE_DELAY,LED_BLUE);
		  fadeInLed(FADE_DELAY,LED_GREEN);
		  fadeOutLed(FADE_DELAY,LED_RED);
		  fadeInLed(FADE_DELAY,LED_BLUE);

	  }
	  else
	  {

		  fadeInLed(10,LED_RED);
		  fadeOutLed(10,LED_RED);
		  fadeInLed(10,LED_GREEN);
		  fadeOutLed(10,LED_GREEN);
		  fadeInLed(10,LED_BLUE);
		  fadeOutLed(10,LED_BLUE);

	  }




	  // switch power condition
	  if((color + luminosity) < 600)
	  {
		  Power = 0;
	  }
	  else
	  {
		  Power = 1;
	  }
#endif

#ifdef LL

	  LL_GPIO_SetOutputPin(PwmRed_Tim3Ch1_GPIO_Port,PwmRed_Tim3Ch1_Pin);
	  LL_mDelay(250);
	  LL_GPIO_ResetOutputPin(PwmRed_Tim3Ch1_GPIO_Port,PwmRed_Tim3Ch1_Pin);
	  LL_mDelay(250);
#endif


  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_10B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : UserButton1_Exti8_Pin UserButton2_Exti9_Pin UserButton3_Exti10_Pin */
  GPIO_InitStruct.Pin = UserButton1_Exti8_Pin|UserButton2_Exti9_Pin|UserButton3_Exti10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
