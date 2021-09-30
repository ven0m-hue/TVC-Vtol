/*
 * main.c
 *
 *  Created on: September 30, 2021
 *      Author: Venom
 */

//Includes
#include "main.h"
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

//Prototypes
void SystemClockConfig(uint8_t clock_freq);
void Error_handler(void);
void UART2_Init(void);
void Timer2_Init(void);
void LSE_Config(void);


// Handle variable of general purpose timer 2 made global
TIM_HandleTypeDef tim2;
UART_HandleTypeDef huart2;
//Pwm handle made global
TIM_OC_InitTypeDef timerPWMconfig;

//Global variables
uint32_t clock_freq;  // for clock config
uint32_t flash_latency;

//Uart variables
uint8_t receivedData;
uint8_t data_buffer[5];
uint32_t count = 0;
bool recepCmplt = false;
uint32_t Data;

void Universal_Inits() {

	HAL_Init();
	SystemClockConfig(SYS_CLOCK_FREQ_50MHz);
	LSE_Config();
	Timer2_Init();
	UART2_Init();
}

int main()
{
	//Inits
	Universal_Inits();
	
	//Transmit to the terminal at start 
	char* user_data = "The VA-U\r\n";
	uint16_t data_len = strlen(user_data);
	HAL_UART_Transmit(&huart2, (uint8_t*)user_data, data_len, HAL_MAX_DELAY);

	//Application code
	/*
	 * 1. Generate a PWM signal using the timer 2 peripheral for the given duty cycle using output compare mode
	 * 2. Init the timer 2 peripheral
	 * 3. Config the PWM driver init
	 * 4. Calculate the period and duty cycle
	 */
	if (HAL_TIM_PWM_Start(&tim2, TIM_CHANNEL_1) != HAL_OK) Error_handler();



	while(1)
	{
		memset(data_buffer, 0, sizeof(data_buffer));
		count = 0;
		while(recepCmplt != true) HAL_UART_Receive_IT(&huart2, &receivedData, 1);

		recepCmplt = false;
	}


	while(1);
	return 0;
}

void SystemClockConfig(uint8_t clock_freq)
{
	/*
				 *  Use the external clock sourced from the stlink onboard debugger mcu's clock
				 *  1. Initialize the oscillator corresponding to their respective regDef
				 *  2. Initialize the oscillator as a bypass, as it is sourced from another mcu
				 *  3. Init the RCC clock config to succefully init the HSE
				 *  4. PLL by sourcing HSE clock
				 */
				RCC_OscInitTypeDef osc_init;
				RCC_ClkInitTypeDef clk_init;

				memset(&osc_init,0, sizeof(osc_init));

				osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;  // For HSE or LSE since we need both
				osc_init.HSEState = RCC_HSE_BYPASS;  // This is for HSE
				osc_init.LSEState = RCC_LSE_ON;      // This is for LSE
				osc_init.PLL.PLLState = RCC_PLL_ON;
				osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;

				switch(clock_freq)
				{
					case SYS_CLOCK_FREQ_50MHz:
					{
						osc_init.PLL.PLLM = 8;
						osc_init.PLL.PLLN = 100;
						osc_init.PLL.PLLP = 2;
						osc_init.PLL.PLLQ = 2;
						osc_init.PLL.PLLR = 2;
						// PLL is  configured
						/*
						 *  In this project configure the following
						 *  1. Config AHB bus clock as 50MHz  /1
						 *  2. Config APB1 bus clock as 25MHz /2 ie divide by 2
						 *  3. Config APB2 bus clock as 25MHz /2 ie divide by 2
						 *  Configured using the RCC config registers
						 */
						clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_HCLK;
						clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
						clk_init.AHBCLKDivider = RCC_CFGR_HPRE_DIV1;
						clk_init.APB1CLKDivider = RCC_CFGR_PPRE1_DIV2;
						clk_init.APB2CLKDivider = RCC_CFGR_PPRE1_DIV2;

						flash_latency = FLASH_LATENCY_1;

						break;

					}
					case SYS_CLOCK_FREQ_80MHz:
					{
						osc_init.PLL.PLLM = 8;
						osc_init.PLL.PLLN = 160;
						osc_init.PLL.PLLP = 2;
						osc_init.PLL.PLLQ = 2;
						osc_init.PLL.PLLR = 2;
						// PLL is  configured
						/*
						 *  In this project configure the following
						 *  1. Config AHB bus clock as 80MHz  /1
						 *  2. Config APB1 bus clock as 40MHz /2 ie divide by 2
						 *  3. Config APB2 bus clock as 80MHz /1 ie divide by 1
						 *  Configured using the RCC config registers
						 */
						clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_HCLK;
						clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
						clk_init.AHBCLKDivider = RCC_CFGR_HPRE_DIV1;
						clk_init.APB1CLKDivider = RCC_CFGR_PPRE1_DIV2;
						clk_init.APB2CLKDivider = RCC_CFGR_PPRE1_DIV1;

						flash_latency = FLASH_LATENCY_2;

						break;
					}
					case SYS_CLOCK_FREQ_120MHz:
					{
						osc_init.PLL.PLLM = 8;
						osc_init.PLL.PLLN = 240;
						osc_init.PLL.PLLP = 2;
						osc_init.PLL.PLLQ = 2;
						osc_init.PLL.PLLR = 2;
						// PLL is  configured
						/*
						 *  In this project configure the following
						 *  1. Config AHB bus clock as 120MHz  /1
						 *  2. Config APB1 bus clock as 40MHz /2 ie divide by 2
						 *  3. Config APB2 bus clock as 60MHz /2 ie divide by 2
						 *  Configured using the RCC config registers
						 */
						clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_HCLK;
						clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
						clk_init.AHBCLKDivider = RCC_CFGR_HPRE_DIV1;
						clk_init.APB1CLKDivider = RCC_CFGR_PPRE1_DIV4;
						clk_init.APB2CLKDivider = RCC_CFGR_PPRE1_DIV2;

						flash_latency = FLASH_LATENCY_3;
						break;
					}
					/*
					 * Side Note: PLLN value must be greater than 50 and lesser than 432
					 */
					case SYS_CLOCK_FREQ_180MHz:
					{
						/*
						 * To drive the mcu to 180 MHz we need to drive power controller register
						 * 1.@Ref Power Controller and Set the regulator voltage scale 1, as per the data sheet
						 * 2.Turn on the Over drive mode
						 */
						__HAL_RCC_PWR_CLK_ENABLE(); // ALways enable the clock for anything

						__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

						__HAL_PWR_OVERDRIVE_ENABLE();

						// osc init
						osc_init.PLL.PLLM = 8;
						osc_init.PLL.PLLN = 360;
						osc_init.PLL.PLLP = 2;   // default
						osc_init.PLL.PLLQ = 2;   // default
						osc_init.PLL.PLLR = 2;   // default
						// PLL is  configured
						/*
						 *  In this project configure the following -> to know more refer @Clock Configuration from the ioc
						 *  1. Config AHB bus clock as 120MHz  /1
						 *  2. Config APB1 bus clock as 40MHz /2 ie divide by 2
						 *  3. Config APB2 bus clock as 60MHz /2 ie divide by 2
						 *  Configured using the RCC config registers
						 */
						clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_HCLK;
						clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
						clk_init.AHBCLKDivider = RCC_CFGR_HPRE_DIV1;
						clk_init.APB1CLKDivider = RCC_CFGR_PPRE1_DIV4;
						clk_init.APB2CLKDivider = RCC_CFGR_PPRE1_DIV2;

						flash_latency = FLASH_LATENCY_5;
					}
					default: return;
				}

			if(HAL_RCC_OscConfig(&osc_init) != HAL_OK) Error_handler();

		    // after this line if everything is okay HSE is succefully turned on
			if (HAL_RCC_ClockConfig(&clk_init, flash_latency) != HAL_OK) Error_handler();

			/*
			 * SYSTICK CONFIG
			 * Since we have changed the clock config from default frequency to the application specific
			 * We need to change the clock config  going into the arm cortex processor. (prcoessor side clock config).
			 */

			HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/ 1000);
			HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK); // There is a pre-scalar @Ref ClockTree
}

void LSE_Config()
{
	// Selects the clock soure to output on any one of the pin
	// MCO1 = PA8 and MCO2 = PA9
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCOSOURCE_LSE, RCC_MCODIV_1);
}


/*
	 * Timer 2 initialization and instantiation

	 * Clk = Peripheral_frequency //50MHz
	 * pre-scaler_counter_clk = clk/(pre-scaler + 1)
	 * time_period = 1/pre-scaler_counter_clk //
	 * Period = time_period * time_delay //
	 * Load the number to the ARR register.(Only 16 bit wide)
	 * side note-> Max ARR value can be upto and not more than 65535 since its a 16 bit register
	 * 
	 * PWM Generation:
		 * Working with the timer2 Output channel for PWM generation, for more info @ref general purpose timer in reference manual
		 * 1. Init the timer Output to Compare the time base
		 * 2. Config  the output channel for PWM
	 *	 
	 *
	 */
void Timer2_Init(void)
{
	//Initialize the timers
	tim2.Instance = TIM2;
	tim2.Init.CounterMode = TIM_COUNTERMODE_UP; // set as up counter
	tim2.Init.Period = 3000 - 1; // for 3ms or 333Hz
	tim2.Init.Prescaler = 50 - 1;
	if(HAL_TIM_PWM_Init(&tim2) != HAL_OK) Error_handler();  //Timer 2 is configured
	
	//PWM Inits
	memset(&timerPWMconfig, 0 , sizeof(timerPWMconfig));
	timerPWMconfig.OCMode = TIM_OCMODE_PWM1;
	timerPWMconfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	
	//Config 10% DutyCycle
	timerPWMconfig.Pulse = tim2.Init.Period * 10/100;
	if(HAL_TIM_PWM_ConfigChannel(&tim2, &timerPWMconfig, TIM_CHANNEL_1) != HAL_OK) Error_handler();

}


//Interrupt service routine 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (receivedData == '\r')
	{
		recepCmplt = true;  //Flag
		Data = atoi((char*)data_buffer);

		//Change the DC on the fly.
		__HAL_TIM_SET_COMPARE(&tim2, TIM_CHANNEL_1, tim2.Init.Period * (DutyCycle(Data))/100);

		data_buffer[count++] = '\r';
		//Send back the revceived data back to the terminal.
		HAL_UART_Transmit(huart, data_buffer, count, HAL_MAX_DELAY);
	}
	else
		data_buffer[count++] = receivedData;

}

void UART2_Init(void)
{
	/*
	 * High level initialization
	 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	//huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;

	if(HAL_UART_Init(&huart2) != HAL_OK) Error_handler();  // If there is a problem

}

void Error_handler(void)
{
	while(1);
}
