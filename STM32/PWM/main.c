/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "main.h"
#include "MPU9250.h"
#include "PID.h"
#include "EKFLite.h"
#include "stdbool.h"
#define SAMPLE_TIME_MS  100

//Peripherals
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef tim2;
TIM_HandleTypeDef tim3;
//TIM_HandleTypeDef tim3;
TIM_OC_InitTypeDef timerPWMconfig1;
TIM_OC_InitTypeDef timerPWMconfig2;


MPU9250_Handle_t imu;
EKF_Handle_t ekfl;

uint8_t receivedData;
uint8_t data_buffer[5];
uint32_t count = 0;
bool recepCmplt = false;
uint32_t Data;

void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void Timer2_Init(void);
void Timer3_Init(void);

void Error_Handler(void);
static void MX_MPU9250_Init(void);
int main(void)
{


   HAL_Init();
   SystemClock_Config();
   Timer2_Init();
   Timer3_Init();
	// MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_MPU9250_Init();

	char* user_data = "The VA-U\r\n";
	uint16_t data_len = strlen(user_data);
	HAL_UART_Transmit(&huart2, (uint8_t*)user_data, data_len, HAL_MAX_DELAY);

  if (HAL_TIM_PWM_Start(&tim3, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

  if (HAL_TIM_PWM_Start(&tim2, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

  EKF_Init(&ekfl);

  uint8_t buf[64];
  memset(buf, 0, sizeof buf);

  while (1)
  {
#if 0
	  memset(data_buffer, 0, sizeof(data_buffer));
	  count = 0;
	  while(recepCmplt != true) HAL_UART_Receive_IT(&huart2, &receivedData, 1);
	  recepCmplt = false;
#endif

	  /* Accelerometer measurements and Kalman update */
	  MPU9250_ReadAccel(&imu);
	  HAL_Delay(1);
	  EKF_Update(&ekfl, imu.acc);

	  /* Gyro measurements and Kalman prediction */
	  MPU9250_ReadGyro(&imu);
	  HAL_Delay(1);
	  EKF_Predict(&ekfl, imu.gyr, 0.001f * SAMPLE_TIME_MS);
	  //Data = (uint32_t)ekfl.roll;
	  __HAL_TIM_SET_COMPARE(&tim2, TIM_CHANNEL_1, tim2.Init.Period * DutyCycleServo((uint32_t)ekfl.roll)/100);
	  __HAL_TIM_SET_COMPARE(&tim3, TIM_CHANNEL_1, tim3.Init.Period * DutyCycleServo((uint32_t)ekfl.pitch)/100);

//	  /* Angles roll and pitch */
//	  sprintf((char*)buf, "RollKF:%f, PitchKF:%f\r\n", ekfl.roll, ekfl.pitch);
//	  HAL_UART_Transmit(&huart2, buf, sizeof(buf), HAL_MAX_DELAY);



	  }
  }



//The ISR()
#if 0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (receivedData == '\r')
	{
		recepCmplt = true;
		Data = atoi((char*)data_buffer);

		//__HAL_TIM_SET_COMPARE(&tim2, TIM_CHANNEL_1, tim2.Init.Period * DutyCycleServo(Data)/100);

		data_buffer[count++] = '\r';
		HAL_UART_Transmit(huart, data_buffer, count, HAL_MAX_DELAY);

	}
	else
	{
		data_buffer[count++] = receivedData;

	}


}
#endif

static void MX_MPU9250_Init(void)
{

	imu.I2Chandle = &hi2c1;
	if(MPU9250_init(&imu) == 0)
	{
	  Error_Handler();
	}
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}



/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{


  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  //huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* Input/output capture and not interrupt capture.
 * Timer initialization and instantiation

 * Clk = Peripheral_frequency //50MHz
 * pre-scaler_counter_clk = clk/(pre-scaler + 1)
 * time_period = 1/pre-scaler_counter_clk //
 * Period = time_period * time_delay //
 * Load the number to the ARR register.(Only 16 bit wide)
 * side note-> Max ARR value can be upto and not more than 65535 since its a 16 bit register
 *
 */

/*
	 * Working with the timer2 Output channel for PWM generation, for more info @ref general purpose timer in reference manual
	 * 1. Init the timer Output to Compare the time base
	 * 2. Config  the output channel for PWM
 */
void Timer2_Init(void)
{

	tim2.Instance = TIM2;
	tim2.Init.CounterMode = TIM_COUNTERMODE_UP; // set as up counter
	tim2.Init.Period = 3000 - 1; // for one milli second
	tim2.Init.Prescaler = 50 - 1;//49;
	if(HAL_TIM_PWM_Init(&tim2) != HAL_OK) Error_Handler();  //Timer 2 is configured

	memset(&timerPWMconfig1, 0 , sizeof(timerPWMconfig1));
	timerPWMconfig1.OCMode = TIM_OCMODE_PWM1;
	timerPWMconfig1.OCPolarity = TIM_OCPOLARITY_HIGH;
	//Initialize PWM for 0% DutyCycle
	timerPWMconfig1.Pulse = tim2.Init.Period * 10/100;
	if(HAL_TIM_PWM_ConfigChannel(&tim2, &timerPWMconfig1, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

}

void Timer3_Init(void)
{

	tim3.Instance = TIM3;
	tim3.Init.CounterMode = TIM_COUNTERMODE_UP; // set as up counter
	tim3.Init.Period = 3000 - 1; // for one milli second
	tim3.Init.Prescaler = 50 - 1;//49;
	if(HAL_TIM_PWM_Init(&tim3) != HAL_OK) Error_Handler();  //Timer 3 is configured

	memset(&timerPWMconfig2, 0 , sizeof(timerPWMconfig2));
	timerPWMconfig2.OCMode = TIM_OCMODE_PWM1;
	timerPWMconfig2.OCPolarity = TIM_OCPOLARITY_HIGH;
	//Initialize PWM for 0% DutyCycle
	timerPWMconfig2.Pulse = tim3.Init.Period * 10/100;
	if(HAL_TIM_PWM_ConfigChannel(&tim3, &timerPWMconfig2, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

}



/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOH_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin : B1_Pin */
//  GPIO_InitStruct.Pin = B1_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : LD2_Pin */
//  GPIO_InitStruct.Pin = LD2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
//
//}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
