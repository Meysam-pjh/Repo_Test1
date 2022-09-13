#include "main.h"
#include "string.h"
#include "stdio.h"
//***************************
I2C_HandleTypeDef hi2c2;
RTC_HandleTypeDef hrtc;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
//********************************
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
//*********************************************
uint16_t Add_Dev = 0X76;						// I2C Address
//'''''''''''''''''''''''''''''''''''''''''''''
uint16_t T_P_Mode_Add = 0X74;
uint8_t T2_P0_Mode1_Val[1];
uint8_t T2_P16_Mode1_Val[1];
uint8_t T4_P16_Mode1_Val[1];
uint8_t T8_P16_Mode1_Val[1];
uint8_t T16_P16_Mode1_Val[1];
//'''''''''''''''''''''''''''''''''''''''''''''
uint16_t Hum_Add = 0X72;
uint8_t Hum1_Val[1];
uint8_t Hum2_Val[1];
uint8_t Hum4_Val[1];
uint8_t Hum8_Val[1];
uint8_t Hum16_Val[1];
//'''''''''''''''''''''''''''''''''''''''''''''
uint16_t Fillter_Add = 0X75;
uint8_t Fillter_0[1];
//'''''''''''''''''''''''''''''''''''''''''''''
uint16_t gas_wait0_Add = 0X64;
uint8_t gas_wait0_Val[1];	// 100ms heat up duration
//'''''''''''''''''''''''''''''''''''''''''''''
uint16_t res_heat0_Add = 0X5A;
uint8_t res_heat0_Val[1];
//'''''''''''''''''''''''''''''''''''''''''''''
uint16_t Ctrl_gas1_Add = 0X71;
uint8_t Ctrl_gas1_Val[1];
//*********************************************
uint16_t T_Data_Add = 0X22;
uint16_t P_Data_Add = 0X1F;
uint16_t H_Data_Add = 0X25;
uint16_t gas_Data_Add = 0X2A;
//'''''''''''''''''''''''''''''''''''''''''''''
uint8_t T_Buffer[3];
uint8_t P_Buffer[2];
uint8_t H_Buffer[2];
uint8_t g_Buffer[2];
uint8_t Add_Data[1];
uint8_t G_MSB , G_LSB , G_range;
//'''''''''''''''''''''''''''''''''''''''''''''
uint32_t TEMP1;
uint16_t HUM , Gas1 , P1;
unsigned int Press;
float HUM1, TEMP2;
double Temperature , Humidity , GAS;
//*********************************************
uint8_t par_g1_Buffer[2] , par_g2_Buffer[2] , par_g3_Buffer[2];
uint8_t Res_Heat_Range[1] , Res_Heat_Val[1] , res_heat_x;
uint16_t par_g2;
double var1 , var2 , var3 , var4 , var5;
uint32_t T_MSB , T_LSB  , T_SLB ;
double TEMP_1_2;
//*********************************************
int main(void)
{
	//''''''''''''''''''''''''''''''
	HAL_Init();
	SystemClock_Config();
	//MX_GPIO_Init();
	MX_RTC_Init();
	MX_SPI1_Init();
	MX_USART2_UART_Init();
	MX_I2C2_Init();
	//*****************************************
	uint16_t Reaset_Add = 0XE0;
	uint8_t Reaset[1];
	Reaset[0] = 0XB6;
	HAL_I2C_Mem_Write(&hi2c2, Add_Dev<<1, Reaset_Add, 1, Reaset, 1, 100);
	//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
	while (1)
	{
		//******************************************************************** Registers Configuration
		T2_P16_Mode1_Val[0] = 0X55;
		//T4_P16_Mode1_Val[0] = 0X75;
		//T8_P16_Mode1_Val[0] = 0X95;
		//T16_P16_Mode1_Val[0] = 0XB5;
		HAL_I2C_Mem_Write(&hi2c2, Add_Dev<<1, T_P_Mode_Add, 1, T2_P16_Mode1_Val, 1, 100);
		//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
		Hum1_Val[0] = 0X01;
		//Hum2_Val[0] = 0X02;
		//Hum4_Val[0] = 0X03;
		//Hum8_Val[0] = 0X04;
		//Hum16_Val[0] = 0X05;
		HAL_I2C_Mem_Write(&hi2c2, Add_Dev<<1, Hum_Add, 1, Hum1_Val, 1, 100);
		//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
		Fillter_0[0] = 0X00;
		HAL_I2C_Mem_Write(&hi2c2, Add_Dev<<1, Fillter_Add, 1, Fillter_0, 1, 100);
		//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
		Ctrl_gas1_Val[0] = 0X10;
		HAL_I2C_Mem_Write(&hi2c2, Add_Dev<<1, Ctrl_gas1_Add, 1, Ctrl_gas1_Val, 1, 100);
		//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
		gas_wait0_Val[0] = 0X59;
		HAL_I2C_Mem_Write(&hi2c2, Add_Dev<<1, gas_wait0_Add, 1, gas_wait0_Val, 1, 100);
		HAL_Delay(30);
		//********************************************************************************************
		HAL_Delay(30);
		//********************************************************************************************
		Add_Data[0] = 0X22;													// Read Temperature
		HAL_I2C_Master_Transmit(&hi2c2, Add_Dev<<1, Add_Data, 1, 100);
		HAL_I2C_Mem_Read(&hi2c2, Add_Dev<<1, T_Data_Add, 1, T_Buffer, 3, 100);
		T_MSB = T_Buffer[0]<<16;
		T_LSB = T_Buffer[1]<<8;
		T_SLB = T_Buffer[2];
		TEMP1 = (T_MSB | T_LSB | T_SLB);
		Temperature = ((TEMP1 / (65536.0 * G_range)) - 2.5);
		//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

		//'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
		Add_Data[0] = 0X1F;													// Read Press
		HAL_I2C_Master_Transmit(&hi2c2, Add_Dev<<1, Add_Data, 1, 100);
		HAL_I2C_Mem_Read(&hi2c2, Add_Dev<<1, P_Data_Add, 1, P_Buffer, 2, 100);
		P1 = (P_Buffer[0]<<8 | P_Buffer[1]);
		Press = ((65536 - P1) / 44);
		//'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
		Add_Data[0] = 0X25;													// Read Humidity
		HAL_I2C_Master_Transmit(&hi2c2, Add_Dev<<1, Add_Data, 1, 100);
		HAL_I2C_Mem_Read(&hi2c2, Add_Dev<<1, H_Data_Add, 1, H_Buffer, 2, 100);
		HUM = (H_Buffer[0]<<8 | H_Buffer[1]);
		HUM1 = (HUM / (1024.0));
		Humidity = (HUM1);
		//*********************************************************************************************
		Add_Data[0] = 0XED;													// Par_g1
		HAL_I2C_Master_Transmit(&hi2c2, Add_Dev<<1, Add_Data, 1, 100);
		HAL_I2C_Mem_Read(&hi2c2, Add_Dev<<1, Add_Data, 1, par_g1_Buffer, 1, 100);
		//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
		Add_Data[0] = 0XEB;													// Par_g2
		HAL_I2C_Master_Transmit(&hi2c2, Add_Dev<<1, Add_Data, 1, 100);
		HAL_I2C_Mem_Read(&hi2c2, Add_Dev<<1, Add_Data, 1, par_g2_Buffer, 2, 100);
		par_g2 = (par_g2_Buffer[0]<<8 | par_g2_Buffer[1]);
		//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
		Add_Data[0] = 0XEE;													// Par_g3
		HAL_I2C_Master_Transmit(&hi2c2, Add_Dev<<1, Add_Data, 1, 100);
		HAL_I2C_Mem_Read(&hi2c2, Add_Dev<<1, Add_Data, 1, par_g3_Buffer, 1, 100);
		//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
		Add_Data[0] = 0X02;													// Reas_Heat_Range
		HAL_I2C_Master_Transmit(&hi2c2, Add_Dev<<1, Add_Data, 1, 100);
		HAL_I2C_Mem_Read(&hi2c2, Add_Dev<<1, Add_Data, 1, Res_Heat_Range, 1, 100);
		//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
		Add_Data[0] = 0X00;													// Reas_Heat_Val
		HAL_I2C_Master_Transmit(&hi2c2, Add_Dev<<1, Add_Data, 1, 100);
		HAL_I2C_Mem_Read(&hi2c2, Add_Dev<<1, Add_Data, 1, Res_Heat_Val, 1, 100);
		//**************************************************************************************
		//**************************************************************************************
		var1 = (par_g1_Buffer[0] / 16.0) + 49.0;
		var2 = ((par_g2 / 32768.0) * 0.0005) + 0.00235;
		var3 = (par_g3_Buffer[0] / 1024.0);
		var4 = var1 * (1.0 + (var2 * 250.0));
		var5 = var4 + (var3 * TEMP1);
		res_heat_x = (uint8_t)(3.4 * (var5 * (4.0 / 4.0 + Res_Heat_Range[0])) * 1.0 / (1.0 + (Res_Heat_Val[0] * 0.002) -25 ));
		//*********************************************************************************************************************
		res_heat0_Val[0] = res_heat_x;
		HAL_I2C_Mem_Write(&hi2c2, Add_Dev<<1, res_heat0_Add, 1, res_heat0_Val, 1, 100);
		//*********************************************************************************************************************
		Add_Data[0] = 0X2A;													// Read Gas
		HAL_I2C_Master_Transmit(&hi2c2, Add_Dev<<1, Add_Data, 1, 100);
		HAL_I2C_Mem_Read(&hi2c2, Add_Dev<<1, gas_Data_Add, 1, g_Buffer, 2, 100);
		G_MSB = g_Buffer[0];
		G_LSB = (g_Buffer[1] & 0XC0);
		G_range = (g_Buffer[1] & 0X0F);
		Gas1 = (G_MSB<<8 | G_LSB);
		GAS = (Gas1 / 1000.0);
  }
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00707CBB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 31;
  hrtc.Init.SynchPrediv = 1023;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PA15_RESERVED_Pin|PA12_RESERVED_Pin|PA1_RESERVED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PC1_RESERVED_Pin|PC0_RESERVED_Pin|PC2_RESERVED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA15_RESERVED_Pin PA12_RESERVED_Pin PA1_RESERVED_Pin */
  GPIO_InitStruct.Pin = PA15_RESERVED_Pin|PA12_RESERVED_Pin|PA1_RESERVED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4_RESERVED_Pin PB1_RESERVED_Pin PB0_RESERVED_Pin */
  GPIO_InitStruct.Pin = PB4_RESERVED_Pin|PB1_RESERVED_Pin|PB0_RESERVED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13_RESERVED_Pin */
  GPIO_InitStruct.Pin = PC13_RESERVED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PC13_RESERVED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1_RESERVED_Pin PC0_RESERVED_Pin PC2_RESERVED_Pin */
  GPIO_InitStruct.Pin = PC1_RESERVED_Pin|PC0_RESERVED_Pin|PC2_RESERVED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

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
