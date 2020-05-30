/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim2;


/* USER CODE BEGIN PV */
uint16_t ADC_Values[2];
__attribute__((__section__(".user_data"))) const uint16_t userConfig[16];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim);
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
  InitUserVariables ();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  Alpha_State_Ptr = *Wait_Reverse;

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_Values, 2);
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE );
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (_sysTickFlagScan) {
			Alpha_State_Ptr();
			Idle_Works();
			if (_10msFlagScan) {
				__NOP();
			}

			_sysTickFlagScan=FALSE;
			_5msFlagScan=FALSE;
			_10msFlagScan=FALSE;
			_100msFlagScan=FALSE;
			_1000msFlagScan=FALSE;
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1600;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  TIM2PERIOD=htim2.Init.Period = 1600;
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// Prevent unused argument(s) compilation warning
	static uint16_t tiktok = 0;

	if (htim->Instance == TIM2) {

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);

		if ((tiktok % 2)) {
			htim->Instance->CCR1 = (htim->Init.Period + 5); /* overall period 1600 0x640*/
			htim->Instance->CCR2 = Duty;
		} else {
			htim->Instance->CCR1 = Duty; /* overall period 1600 0x640*/
			htim->Instance->CCR2 = (htim->Init.Period + 5);
		}

		tiktok++;
	}

}

void Idle_Works(void) {

	static uint8_t ADCSAvgCounter = 0;
	static const char CR[] = { 0x0A };
	static enum PrintMode {
		Intro, Giris, Kayit
	} PrintMode = Intro;
	static uint8_t ParametreCount = 0;

	static char TransmitData[128] = { 0 }, str[10];
	static uint16_t POT1_ConvSum = 0, POT2_ConvSum = 0, Rcv; /*Default timeout*/
	/*ADC Clock is set to 4.5 Mhz, Sampling time 239.5 cycles, +12.5 cycles conversion time two conversion
	 *  : 4.5e6/(2*(71.5+12.5)) app. 10kHz  */

	POT1_ConvSum += ADC_Values[0];
	POT2_ConvSum += ADC_Values[1];
	ADCSAvgCounter++;
	if (ADCSAvgCounter == 16) {
		ADCSAvgCounter = 0;
		Pot1_ConvAvg = POT1_ConvSum >> 4;
		Pot2_ConvAvg = POT2_ConvSum >> 4;
		Pot1_ConvAvg=Pot1_ConvAvg+100;
		Pot2_ConvAvg=Pot2_ConvAvg+100;

		POT1_ConvSum = 0;
		POT2_ConvSum = 0;
	}

	if (check == 1) {
		if (PrintMode == Intro) {
			TransmitData[0] = 0x0A;			// new line
			for (int i = 1; i < 128; i++) {
				TransmitData[i] = 0;
			}

			for (int i = 0; i < NUMBER_OF_PARAMETERS; i++) {
				//	strcpy(Dummy, StrArr[i]);
				strcat(TransmitData, StrArr[i]);
				sprintf(str, "%d", ParamArray[i]);
				strcat(str, CR);
				strcat(TransmitData, str);
			}

			strcat(TransmitData, CR);
			strcat(TransmitData, StrArr[NUMBER_OF_PARAMETERS]);

			CDC_Transmit_FS((uint8_t *) TransmitData, strlen(TransmitData));
			for (int i = 0; i < dataSize; i++) {
				ReceivedData[i] = 0;
			}

			check = 0;
			PrintMode = Giris;
			ParametreCount = 0;
		}

		else if (PrintMode == Giris) {
			for (int i = 0; i < 128; i++) {
				TransmitData[i] = 0;
			}
			Rcv = atoi(ReceivedData);
			for (int i = 0; i < dataSize; i++) {
				ReceivedData[i] = 0;
			}

			if ( ((Rcv <= 100) && ParametreCount<2) || ((Rcv <= 5000) && ParametreCount>=2 ))			 {
				ScaleArray[ParametreCount] = Rcv;

				sprintf(str, "%d", ScaleArray[ParametreCount]);
				strcat(TransmitData, str);

				ParametreCount++;
				strcat(TransmitData, CR);

				if (ParametreCount < NUMBER_OF_PARAMETERS)
					strcat(TransmitData,
							StrArr[NUMBER_OF_PARAMETERS + ParametreCount]);
				else {
					PrintMode = Kayit;
					strcat(TransmitData, CR);
					strcat(TransmitData, StrArr[NUMBER_OF_STRING - 2]);
				}
				CDC_Transmit_FS((uint8_t *) TransmitData, strlen(TransmitData));

			}

			else {
				strcat(TransmitData, CR);
				strcat(TransmitData, StrArr[NUMBER_OF_STRING - 3]);
				strcat(TransmitData, CR);
				strcat(TransmitData, StrArr[NUMBER_OF_PARAMETERS + ParametreCount]);
				CDC_Transmit_FS((uint8_t *) TransmitData, strlen(TransmitData));
			}

			check = 0;
		}

		else if (PrintMode == Kayit) {
			for (int i = 0; i < 128; i++) {
				TransmitData[i] = 0;
			}
			if (ReceivedData[0] == 'y') {
				for (int i = 0; i < dataSize; i++) {
					ReceivedData[i] = 0;
				}
				ParamArray[0] = TIM2PERIOD / 100 * ScaleArray[0];
				ParamArray[1] = TIM2PERIOD / 100 * ScaleArray[1];
				ParamArray[2] = ScaleArray[2];				// ms
				ParamArray[3] = ScaleArray[3];				// ms

				DutyMAX = ParamArray[0];
				DutyMAX_Reverse = ParamArray[1];

				strcat(TransmitData, CR);
				strcat(TransmitData, StrArr[NUMBER_OF_STRING - 1]);
				CDC_Transmit_FS((uint8_t *) TransmitData, strlen(TransmitData));
				SaveandExit(ParamArray);
			}
			PrintMode = Intro;
			check = 0;
		}
	}
}

void Forward (void){

	static bool ForwardFlag = 0;
	static uint16_t Forwardimeout = 100; /*Default timeout*/
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_15);	//Enable Driver Chip

	/* State entry works*/
	if (ForwardFlag) {
		ForwardFlag = FALSE;
		StateSandClock = 0;
		Duty = 0;
		Forwardimeout= (uint16_t) ((float) ParamArray[2] * (float) Pot1_ConvAvg/4096.0f);
	}

	if (_10msFlagScan)	{
		StateSandClock += 10;
		if (Duty < (DutyMAX-(DutyMAX>>5U))) Duty=Duty+(DutyMAX>>5U);
		else Duty=DutyMAX;
	}

	if (StateSandClock > Forwardimeout) {

		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14, GPIO_PIN_RESET);	//Enable Driver Chip
		Alpha_State_Ptr=*Wait_Forward;
		ForwardFlag = TRUE;
		StateSandClock=0;
	}

}

void Wait_Reverse (void){

	static __IO uint32_t tmpccmrx;
	static bool WaitReverseFlag = 0;
	static uint16_t Wait_ReverseTimeout = 100;  /*Default timeout*/

	/* State entry works*/
	if (WaitReverseFlag) {
		WaitReverseFlag = FALSE;
		StateSandClock = 0;
		Wait_ReverseTimeout= (uint16_t) ((float) ParamArray[3] * (float) Pot2_ConvAvg/4096.0f);
	}

	if (_10msFlagScan)	StateSandClock += 10;

	if (StateSandClock > Wait_ReverseTimeout) {

		/*State Exit Works*/
		tmpccmrx = htim2.Instance->CCMR1;
		/* Reset the Output Compare mode and Capture/Compare selection Bits */
		tmpccmrx &= ~TIM_CCMR1_OC1M;
		tmpccmrx &= ~TIM_CCMR1_OC2M;
		tmpccmrx |= (TIM_OCMODE_PWM1);
		tmpccmrx |= (TIM_OCMODE_PWM2 << 8U);
		htim2.Instance->CCMR1 = tmpccmrx;
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14, GPIO_PIN_SET);
		Alpha_State_Ptr = *Forward; /*next state*/
		StateSandClock=0;
		WaitReverseFlag=TRUE;
	}
}

void Reverse (void){

	static bool ReverseFlag = 0;
	static uint16_t ReverseTimeout = 100; /*Default timeout*/

	/* State entry works*/
	if (ReverseFlag) {
		ReverseFlag = FALSE;
		StateSandClock = 0;
		ReverseTimeout = 3000; /* ms, Can be adjusted by UCB Communication*/
	}

	if (_10msFlagScan)	{
		StateSandClock += 10;
		if (Duty < (DutyMAX-(DutyMAX>>5U))) Duty=Duty+(DutyMAX>>5U);
		else Duty=DutyMAX_Reverse;
	}

	if (StateSandClock > ReverseTimeout || 		HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==GPIO_PIN_RESET ) {

		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14, GPIO_PIN_RESET);	//Enable Driver Chip
		Alpha_State_Ptr=*Wait_Reverse;
		ReverseFlag=TRUE;
		StateSandClock=0;
	}

	/*exit reverse state */
}

void Wait_Forward(void) {
	static __IO uint32_t tmpccmrx;
	static bool WaitForwardFlag = 0;
	static uint16_t Wait_ForwardTimeout = 100; /*Default timeout*/

	/* State entry works*/
	if (WaitForwardFlag) {
		WaitForwardFlag = FALSE;
		StateSandClock = 0;
		Wait_ForwardTimeout = 1000; /* ms, Can be adjusted by UCB Communication*/
	}

	if (_10msFlagScan)	StateSandClock += 10;

	if (StateSandClock > Wait_ForwardTimeout) {


		/*State Exit Works*/
		tmpccmrx = htim2.Instance->CCMR1;
		/* Reset the Output Compare mode and Capture/Compare selection Bits */
		tmpccmrx &= ~TIM_CCMR1_OC1M;
		tmpccmrx &= ~TIM_CCMR1_OC2M;
		tmpccmrx |= (TIM_OCMODE_PWM2);
		tmpccmrx |= (TIM_OCMODE_PWM1 << 8U);
		htim2.Instance->CCMR1 = tmpccmrx;

		Duty = 0;
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14, GPIO_PIN_SET);	//Enable Driver Chip
		Alpha_State_Ptr = *Reverse; /*next state*/
		WaitForwardFlag=TRUE;
		StateSandClock=0;
	}
}



//	tick period: uwTickFreq			/*Period in ms */
//called from void SysTick_Handler(void) xx..it.c
void SysTickCountersUpdate(void) {
	//Counter Definition
	static unsigned int _5msCounter = 0;
	static unsigned int _10msCounter = 0;
//	static unsigned int _50msCounter = 0;
	static unsigned int _100msCounter = 0;
//    static unsigned int _500msCounter = 0;

	_sysTickFlagScan = TRUE;

//    CheckButtons();
//    //_msTickCounter++;
//
    _5msCounter++;
    if(_5msCounter >= Tickin_1ms)	{
       _5msFlagScan = TRUE;
        _5msCounter = 0;
    }

    _10msCounter++;
    if(_10msCounter >= Tickin_10ms)	{
       _10msFlagScan = TRUE;
       _10msCounter = 0;
    }

    _100msCounter++;
    if(_100msCounter >= Tickin_100ms)	{
       _100msFlagScan = TRUE;
       _100msCounter = 0;
    }

//
//    _50msCounter++;
//    if(_50msCounter>=_50msTickCount)
//    {
//        _50msFlagScan = TRUE;
//        _50msCounter = 0;
//    }
//
}

void SaveandExit (uint16_t * Parameters)		{
	HAL_FLASH_Unlock();
	FLASH_PageErase(FLASH_PAGE_63);
	HAL_FLASH_Program( FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)userConfig, 0xABCD);
	HAL_FLASH_Program( FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)&userConfig[1], Parameters[0]);
	HAL_FLASH_Program( FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)&userConfig[2], Parameters[1]);
	HAL_FLASH_Program( FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)&userConfig[3], Parameters[2]);
	HAL_FLASH_Program( FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)&userConfig[4], Parameters[3]);
	HAL_FLASH_Lock();
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
