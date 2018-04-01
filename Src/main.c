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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#define HAL

#define LED_RED 	TIM_CHANNEL_1
#define LED_GREEN 	TIM_CHANNEL_2
#define LED_BLUE 	TIM_CHANNEL_4

/* Boarders who define on witch point of color adc value a color maximum is reached */
#define COLOR_MAX_VALUE		20160
#define BOARDER_GREEN		20160
#define BOARDER_BLUE		40320
#define BOARDER_RED			60480 	// and 0
#define BOARDER_WHITE		63000

#define COLOR_TIM3_SCALING_FACTOR			3		// factor to scale form 20160 color value to 16 bit tim3 value
#define COLOR_TIM3_SCALING_FACTOR_OFFSET	5055	// existing offset, when scaling from 20160 color value to 16 bit tim3 value

#define ADC_TIM3_SCALING_FACTOR				16		// factor to scale from 12 bit adc to 16 bit tim3
#define ADC_TIM3_SCALING_FACTOR_OFFSET		15		// existing offset, when scaling 12 bit adc to 16 bit tim3

#define FADE_DELAY			25

#define ADC_ARRAY_LENGTH 	1000

#define TIM3_MAX_VALUE		65535				// maximum value for tim3 (2^16-1)

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t Power = 0;

uint32_t adc[2];

uint32_t RandomNumberAdc = 0;

uint32_t color;
uint32_t colorArray[ADC_ARRAY_LENGTH];
uint32_t luminosity;
uint32_t luminosityArray[ADC_ARRAY_LENGTH];
uint16_t AdcArrayCounter = 0;

uint64_t PwmDutyRed;
uint64_t PwmDutyGreen;
uint64_t PwmDutyBlue;

uint8_t Programm = 0;		// switch for light programs
uint8_t BreakProgram = 0;  // flag can be set to break the actual program (0…do nothing; 1…break program)

uint32_t test;			// a test variable
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#ifdef HAL




/**
  * @brief  Sets Duty to TIM PWM
  * @param  PulseLength 0 - 65535 (2^16)
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

	while(PwmDuty/TimeDiv < TIM3_MAX_VALUE)
	{
		setPwmDuty(PwmDuty/TimeDiv,Channal);
		PwmDuty = (PwmDuty + PwmDuty/TimeDiv);
		HAL_Delay(FADE_DELAY);
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
	uint64_t PwmDuty = TIM3_MAX_VALUE*TimeDiv;

	if(TimeDiv > 1000)
		TimeDiv = 1000;

	Channal = Led;

	while(PwmDuty/TimeDiv > TimeDiv)
	{
		setPwmDuty(PwmDuty/TimeDiv,Channal);
		PwmDuty = (PwmDuty - PwmDuty/TimeDiv);
		HAL_Delay(FADE_DELAY);
	}
	setPwmDuty(0,Channal);


}

/**
  * @brief  Fades the LEDs from red to violett
  * @retval none
  */
void circleColor()
{
	int32_t n;		// counter variable

	setPwmDuty(TIM3_MAX_VALUE, LED_RED);
	setPwmDuty(0, LED_GREEN);
	setPwmDuty(0, LED_BLUE);

	n = 0;
	while(n<=TIM3_MAX_VALUE)
	{
		setPwmDuty(TIM3_MAX_VALUE-n, LED_RED);
		setPwmDuty(n, LED_GREEN);
		n++;
		if(BreakProgram == 1)
			break;
	}

	n = 0;
	while(n<=TIM3_MAX_VALUE)
	{
		setPwmDuty(TIM3_MAX_VALUE-n, LED_GREEN);
		setPwmDuty(n, LED_BLUE);
		n++;
		if(BreakProgram == 1)
			break;
	}

	n = 0;
	while(n<=TIM3_MAX_VALUE)
	{
		setPwmDuty(TIM3_MAX_VALUE-n, LED_BLUE);
		setPwmDuty(n, LED_RED);
		n++;
		if(BreakProgram == 1)
			break;
	}

}

void pulsingRGB()
{
	// turn off all leds
	setPwmDuty(0,LED_RED);
	setPwmDuty(0,LED_GREEN);
	setPwmDuty(0,LED_BLUE);

	// pulsing red, green, blue
	fadeInLed(10,LED_RED);
	fadeOutLed(10,LED_RED);
	fadeInLed(10,LED_GREEN);
	fadeOutLed(10,LED_GREEN);
	fadeInLed(10,LED_BLUE);
	fadeOutLed(10,LED_BLUE);
}


void chooseRainbowColor(uint32_t color)
{
	/* area from red to green */
	if (color >= 0 && color <=BOARDER_GREEN)
	{
		setPwmDuty((BOARDER_GREEN-color)*COLOR_TIM3_SCALING_FACTOR*luminosity/TIM3_MAX_VALUE,LED_RED);
		setPwmDuty(color*COLOR_TIM3_SCALING_FACTOR*luminosity/TIM3_MAX_VALUE,LED_GREEN);
		setPwmDuty(0,LED_BLUE);
	}
	/* area from green to blue */
	if (color > BOARDER_GREEN && color <= BOARDER_BLUE)
	{
		setPwmDuty(0,LED_RED);
		setPwmDuty((BOARDER_GREEN-(color-BOARDER_GREEN))*COLOR_TIM3_SCALING_FACTOR*luminosity/TIM3_MAX_VALUE,LED_GREEN);
		setPwmDuty((color-BOARDER_GREEN)*COLOR_TIM3_SCALING_FACTOR*luminosity/TIM3_MAX_VALUE,LED_BLUE);
	}
	/* area from blue to red */
	if (color > BOARDER_BLUE && color <= BOARDER_RED)
	{
		setPwmDuty((color-BOARDER_BLUE)*COLOR_TIM3_SCALING_FACTOR*luminosity/TIM3_MAX_VALUE,LED_RED);
		setPwmDuty(0,LED_GREEN);
		setPwmDuty((BOARDER_GREEN-(color-BOARDER_BLUE))*COLOR_TIM3_SCALING_FACTOR*luminosity/TIM3_MAX_VALUE,LED_BLUE);
	}
	/* small passage for red only (smooth transition to white */
	if (color > BOARDER_RED && color <= BOARDER_WHITE)
	{
		setPwmDuty(COLOR_MAX_VALUE*COLOR_TIM3_SCALING_FACTOR*luminosity/TIM3_MAX_VALUE,LED_RED);
		setPwmDuty(0,LED_GREEN);
		setPwmDuty(0,LED_BLUE);
	}
	/* small passage for white at the end of the scale */
	if (color > BOARDER_WHITE && color <= TIM3_MAX_VALUE)
	{
		setPwmDuty(luminosity,LED_RED);
		setPwmDuty(luminosity,LED_GREEN);
		setPwmDuty(luminosity,LED_BLUE);
	}
}


void pulseRandomColor()
{

}


/**
  * @brief  This function provides accurate delay (in milliseconds) based
  *         on variable incremented.
  * @note   In the default implementation , SysTick timer is the source of time base.
  *         It is used to generate interrupts at regular time intervals where uwTick
  *         is incremented.
  *         The origin function is overwritten with this implementation.
  * @param  Delay specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /* Add a period to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
     wait++;
  }

  while((HAL_GetTick() - tickstart) < wait)
  {
	  if(BreakProgram == 1)
		  break;
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BreakProgram = 1;		// brake actual program
	//Power = 1 - Power;
	switch(GPIO_Pin)
	{
	case GPIO_PIN_8:
		// HAL_GPIO_WritePin(PwmRed_Tim3Ch1_GPIO_Port,PwmRed_Tim3Ch1_Pin, 1);
		Programm = 1;
		break;

	case GPIO_PIN_9:
		// HAL_GPIO_WritePin(PwmGreen_Tim3Ch2_GPIO_Port, PwmGreen_Tim3Ch2_Pin, 1);
		Programm = 2;
		break;

	case GPIO_PIN_10:
		// HAL_GPIO_WritePin(PwmBlue_Tim3Ch4_GPIO_Port,PwmBlue_Tim3Ch4_Pin, 1);
		Programm = 3;
		break;

	default:
		break;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	colorArray[AdcArrayCounter] = adc[0]*ADC_TIM3_SCALING_FACTOR;
	luminosityArray[AdcArrayCounter]=adc[1]*ADC_TIM3_SCALING_FACTOR+ADC_TIM3_SCALING_FACTOR_OFFSET;			// read adc value (10 bit) and expand to pwm value (32 bit)


	if(AdcArrayCounter < ADC_ARRAY_LENGTH-1)
	{
		AdcArrayCounter++;
	}
	else
	{
		AdcArrayCounter = 0;
	}


	RandomNumberAdc = (RandomNumberAdc << 1) | ((adc[0] & 0x00000001) ^ (adc[1] & 0x00000001));

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

	  /* adc average */
		color = 0;
		for(uint16_t n = 0; n<ADC_ARRAY_LENGTH; n++)
		{
			color=color+colorArray[n];
		}
		color = color/ADC_ARRAY_LENGTH;

		luminosity = 0;
		for(uint16_t n = 0; n<ADC_ARRAY_LENGTH; n++)
		{
			luminosity=luminosity+luminosityArray[n];
		}
		luminosity = luminosity/ADC_ARRAY_LENGTH;



		/* choose programm */
		BreakProgram = 0;				// reset BreakProgram flag
		switch(Programm)
		{

		// light dimmable with luminosity potentiometer and choosable color with color potentiometer (at maximum value color is white)
		case 1:		chooseRainbowColor(color);
					break;

		// pulsing red, green, blue
		case 2:		pulsingRGB();
					break;

		// circle color with linear function
		case 3:		circleColor();
					break;




		default:
					break;
		}

	  // light effect
	  if(Power == 1)
	  {



	  }
	  else
	  {



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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
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
