/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INF 10000
#define min(X, Y) (((X) < (Y)) ? (X) : (Y))
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

typedef struct Point {
	int x;
	int y;
} Point;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC4_Init(void);
/* USER CODE BEGIN PFP */
int calc();
void SetAddress(int i);
int isInside(Point polygon[], int n, Point p);
int doIntersect(Point p1, Point q1, Point p2, Point q2);
int orientation(Point p, Point q, Point r);
int onSegment(Point p, Point q, Point r);
int checkSeonsorPosition(float dist);
void saftyBubble();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define width 559
#define length 600
uint32_t ADCValue = 0;
float Vi = (5 / 4096.0) * 1000;
int prevTime = 0;
int elapsed = 0;
int address = 0;
float frontLeft1 = 0;
float frontRight1 = 0;
float left = 0;
float right = 0;
float backLeft = 0;
float backright = 0;
float frontLeft2 = 0;
float frontRight2 = 0;
float distance[8];
int idx = 0;
float Voltage = 0;
float area1, area2, area3, area4, area5, area6, area7, area8, area9, area10,
		area11, area12, area13, area14, area15, area16;
float reading1, reading2, reading3, reading4, reading5, reading6, reading7,
		reading8;
int SensorMaxRange = 600;
int nothing = 8000;
int FBLRCutOff = 559 * (tan(67)); // 67 is the angle of the 1030 sensors
int SLRCutOff = 559;
int Distance[16];
int frontLeftOb, frontOb, frontRightOb, rearOb, rearRightOb, rearLeftOb, leftOb,
		rightOb = 0;

//bubble
int fowardCollisionD = 0;
int sideOffset = 0;
int rightSideCollision;
int leftSideCollision;
int backCollisionD = 0;
int scaleproportion = 0;
int Sensorbubble1, Sensorbubble2, Sensorbubble3, Sensorbubble4, Sensorbubble5,
		Sensorbubble6, Sensorbubble7, Sensorbubble8;
int dir = 0;
int velocity = 0;

///////////////////////////////////
//sensor postioning x,y in mm
int sensorNum = 0;
int sensorXY[6][1] = { { 267, 318 }, { -267, 318 }, { -267, -254 },
		{ 267, -254 }, { -228, -317 }, { 228, -317 } };
uint32_t ADCpot1 = 0;
uint32_t ADCpot2 = 0;
double multipleC = .1;
int middle = 2048;
int deltaDir = 0;
int stop83 = 0;
int stop74 = 0;
/// for display in stm studio
int centerx = 0;
int centery = 0;
bourderx1 = -300;
bordery1 = 300;
bourderx2 = 300;
bordery2 = -300;
int backY, frontY, frontRightY, frontLeftY, backLeftY, backRightY, rightY,
		leftY = 0;
int frontX, backX, frontRightX, frontLeftX, backLeftX, backRightX, rightX,
		leftX = 0;
///
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_RTC_Init();
	MX_TIM2_Init();
	MX_USART3_UART_Init();
	MX_USB_PCD_Init();
	MX_ADC2_Init();
	MX_ADC3_Init();
	MX_ADC4_Init();
	/* USER CODE BEGIN 2 */
	HAL_ADC_Start(&hadc2);
	HAL_ADC_Start(&hadc3);
	HAL_ADC_Start(&hadc4);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		///bubble caluclations
		velocity = HAL_ADC_GetValue(&hadc3);
		dir = HAL_ADC_GetValue(&hadc4);
		deltaDir = abs(dir - middle);

		// more basic algorithm used works better overall rather then other method
		// increase bubble with velocity
		SensorMaxRange = 1000 + velocity;
		Sensorbubble1 = 1000 + velocity;
		Sensorbubble2 = 1000 + velocity;
		Sensorbubble3 = 1000 + velocity;
		Sensorbubble4 = 1000 + velocity;
		Sensorbubble5 = 1000 + velocity;
		Sensorbubble6 = 1000 + velocity;
		Sensorbubble7 = 1000 + velocity;
		Sensorbubble8 = 1000 + velocity;
		stop83 = 0;
		stop74 = 0;
		//change bubble size with direction
		if (dir < 2048) { //left
			Sensorbubble8 = Sensorbubble8 - deltaDir * multipleC;
			Sensorbubble4 = Sensorbubble4 - deltaDir * multipleC;
			Sensorbubble7 = Sensorbubble7 + deltaDir * multipleC;
			Sensorbubble3 = Sensorbubble3 + deltaDir * multipleC;
			Sensorbubble5 = Sensorbubble5 - deltaDir * multipleC;
			Sensorbubble6 = Sensorbubble6 + deltaDir * multipleC;
			if (dir < 30) {
				stop83 = 1;
			} else {
				stop83 = 0;
			}
		}
		if (dir > 2048) { //right
			Sensorbubble8 = Sensorbubble8 + deltaDir * multipleC;
			Sensorbubble4 = Sensorbubble4 + deltaDir * multipleC;
			Sensorbubble7 = Sensorbubble7 - deltaDir * multipleC;
			Sensorbubble3 = Sensorbubble3 - deltaDir * multipleC;
			Sensorbubble5 = Sensorbubble5 + deltaDir * multipleC;
			Sensorbubble6 = Sensorbubble6 - deltaDir * multipleC;
		}
		if (dir > 4000) {
			stop74 = 1;
		} else {
			stop74 = 0;
		}

		SetAddress(idx);
		prevTime = HAL_GetTick();
		elapsed = 0;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

		while (elapsed < 1) { //wait

			elapsed = HAL_GetTick() - prevTime;
		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		prevTime = HAL_GetTick();
		elapsed = 0;
		while (elapsed < 47) { //wait

			elapsed = HAL_GetTick() - prevTime;
		}
		ADCValue = HAL_ADC_GetValue(&hadc2);
		Voltage = ((ADCValue / 4096.0) * 3.3) * 1000;
		distance[idx] = ((5 * (Voltage / Vi)) / 1.27);

		if (idx == 0) {
			frontLeft1 = distance[idx];
			reading1 = distance[idx];
		} else if (idx == 1) {
			frontRight1 = distance[idx];
			reading2 = distance[idx];
		} else if (idx == 2) {
			left = distance[idx];
			reading3 = distance[idx];
		} else if (idx == 3) {
			right = distance[idx];
			reading4 = distance[idx];
		} else if (idx == 4) {
			backLeft = distance[idx];
			reading5 = distance[idx];
		} else if (idx == 5) {
			reading6 = distance[idx];
			backright = distance[idx];
		} else if (idx == 6) {
			reading7 = distance[idx];
			frontRight2 = distance[idx];
		} else if (idx == 7) {
			reading8 = distance[idx];
			frontLeft2 = distance[idx];
		}
		if (idx == 7) {
			backY = 0;
			frontY = 0;
			frontRightY = 0;
			frontLeftY = 0;
			backLeftY = 0;
			backRightY = 0;
			rightY = 0;
			leftY = 0;
			frontX = 0;
			frontY = 0;
			frontRightX = 0;
			frontLeftX = 0;
			backLeftX = 0;
			backRightX = 0;
			rightX = 0;
			leftX = 0;
			frontLeftOb = 0;
			frontOb = 0;
			frontRightOb = 0;
			rearOb = 0;
			rearRightOb = 0;
			rearLeftOb = 0;
			leftOb = 0;
			rightOb = 0;
			calc();

			idx = 0;
		} else {
			idx++;
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB
			| RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC12
			| RCC_PERIPHCLK_ADC34 | RCC_PERIPHCLK_TIM2;
	PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
	PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */
	/** Common config
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.ContinuousConvMode = ENABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc2.Init.LowPowerAutoWait = DISABLE;
	hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void) {

	/* USER CODE BEGIN ADC3_Init 0 */

	/* USER CODE END ADC3_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC3_Init 1 */

	/* USER CODE END ADC3_Init 1 */
	/** Common config
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc3.Init.ContinuousConvMode = ENABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc3.Init.LowPowerAutoWait = DISABLE;
	hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc3) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC3_Init 2 */

	/* USER CODE END ADC3_Init 2 */

}

/**
 * @brief ADC4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC4_Init(void) {

	/* USER CODE BEGIN ADC4_Init 0 */

	/* USER CODE END ADC4_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC4_Init 1 */

	/* USER CODE END ADC4_Init 1 */
	/** Common config
	 */
	hadc4.Instance = ADC4;
	hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc4.Init.Resolution = ADC_RESOLUTION_12B;
	hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc4.Init.ContinuousConvMode = ENABLE;
	hadc4.Init.DiscontinuousConvMode = DISABLE;
	hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc4.Init.NbrOfConversion = 1;
	hadc4.Init.DMAContinuousRequests = DISABLE;
	hadc4.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc4.Init.LowPowerAutoWait = DISABLE;
	hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc4) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC4_Init 2 */

	/* USER CODE END ADC4_Init 2 */

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */
	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 72;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 0xFFFF;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
	sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
	sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 15;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 38400;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief USB Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_PCD_Init(void) {

	/* USER CODE BEGIN USB_Init 0 */

	/* USER CODE END USB_Init 0 */

	/* USER CODE BEGIN USB_Init 1 */

	/* USER CODE END USB_Init 1 */
	hpcd_USB_FS.Instance = USB;
	hpcd_USB_FS.Init.dev_endpoints = 8;
	hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USB_Init 2 */

	/* USER CODE END USB_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOF_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOE_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOG_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	GPIO_PIN_12 | LD3_Pin | SwitchAdr1_Pin | SwitchAdr2_Pin | LD2_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, SwitchAdr0_Pin | LD1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : USER_Btn_Pin */
	GPIO_InitStruct.Pin = USER_Btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PA1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB12 LD3_Pin SwitchAdr1_Pin SwitchAdr2_Pin
	 LD2_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | LD3_Pin | SwitchAdr1_Pin
			| SwitchAdr2_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OverCurrent_Pin */
	GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SwitchAdr0_Pin LD1_Pin */
	GPIO_InitStruct.Pin = SwitchAdr0_Pin | LD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void SetAddress(int i) {
	if (i == 0) {

		HAL_GPIO_WritePin(SwitchAdr0_GPIO_Port, SwitchAdr0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SwitchAdr1_GPIO_Port, SwitchAdr1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SwitchAdr2_GPIO_Port, SwitchAdr2_Pin, GPIO_PIN_RESET);

	} else if (i == 1) {

		HAL_GPIO_WritePin(SwitchAdr0_GPIO_Port, SwitchAdr0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SwitchAdr1_GPIO_Port, SwitchAdr1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SwitchAdr2_GPIO_Port, SwitchAdr2_Pin, GPIO_PIN_RESET);

	} else if (i == 2) {
		HAL_GPIO_WritePin(SwitchAdr0_GPIO_Port, SwitchAdr0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SwitchAdr1_GPIO_Port, SwitchAdr1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SwitchAdr2_GPIO_Port, SwitchAdr2_Pin, GPIO_PIN_RESET);

	} else if (i == 3) {
		HAL_GPIO_WritePin(SwitchAdr0_GPIO_Port, SwitchAdr0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SwitchAdr1_GPIO_Port, SwitchAdr1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SwitchAdr2_GPIO_Port, SwitchAdr2_Pin, GPIO_PIN_RESET);

	} else if (i == 4) {
		HAL_GPIO_WritePin(SwitchAdr0_GPIO_Port, SwitchAdr0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SwitchAdr1_GPIO_Port, SwitchAdr1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SwitchAdr2_GPIO_Port, SwitchAdr2_Pin, GPIO_PIN_SET);

	} else if (i == 5) {
		HAL_GPIO_WritePin(SwitchAdr0_GPIO_Port, SwitchAdr0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SwitchAdr1_GPIO_Port, SwitchAdr1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SwitchAdr2_GPIO_Port, SwitchAdr2_Pin, GPIO_PIN_SET);

	} else if (i == 6) {
		HAL_GPIO_WritePin(SwitchAdr0_GPIO_Port, SwitchAdr0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SwitchAdr1_GPIO_Port, SwitchAdr1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SwitchAdr2_GPIO_Port, SwitchAdr2_Pin, GPIO_PIN_SET);

	} else if (i == 7) {
		HAL_GPIO_WritePin(SwitchAdr0_GPIO_Port, SwitchAdr0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SwitchAdr1_GPIO_Port, SwitchAdr1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SwitchAdr2_GPIO_Port, SwitchAdr2_Pin, GPIO_PIN_SET);


	}

}

int calc() {
	//front outer sensors

	if (reading8 < Sensorbubble8 && stop83 == 0) {
		area16 = reading8;

	} else {
		area16 = nothing;
	}
	if (reading7 < Sensorbubble7 && stop74 == 0) {
		area15 = reading7;

	} else {
		area15 = nothing;
	}

	//side
	if (reading3 < Sensorbubble3 && stop83 == 0) {
		if (reading3 > SLRCutOff) {
			area6 = reading3;
			area13 = nothing;
		} else {
			area13 = reading3;
			area6 = nothing;
		}
	} else {
		area6 = nothing;
		area13 = nothing;

	}
	if (reading4 < Sensorbubble4 && stop74 == 0) {
		if (reading4 > SLRCutOff) {
			area7 = reading4;
			area14 = nothing;
		} else {
			area14 = reading4;
			area7 = nothing;
		}
	} else {

		area14 = nothing;
		area7 = nothing;
	}
	/* old logic
	 if ((reading1 < FBLRCutOff && reading2 < FBLRCutOff)
	 || ((reading1 > FBLRCutOff && reading2 > FBLRCutOff)
	 && ((reading7 < FBLRCutOff) || (area15 == nothing))
	 && ((reading8 < FBLRCutOff) || (area16 == nothing))))

	 */

//front inner sensors
	if (reading1 < Sensorbubble1 || reading2 < Sensorbubble2) { //is something seen
		if (reading1 > FBLRCutOff && reading2 > FBLRCutOff) {
			if (reading1 > reading2) {
				area2 = reading2;
			} else {
				area2 = reading1;
			}
		} else {
			area2 = nothing;
		}

		if (reading1 < FBLRCutOff && reading1 < Sensorbubble1) {
			area4 = reading1;
		} else {
			area4 = nothing;
		}

		if (reading2 < FBLRCutOff && reading2 < Sensorbubble2) {
			area5 = reading2;
		} else {
			area5 = nothing;
		}

		if (reading1 > FBLRCutOff && reading1 < Sensorbubble1
				&& reading8 > FBLRCutOff && reading8 < Sensorbubble8) {
			area3 = reading1;

		} else {

			area3 = nothing;
		}
		if (reading2 > FBLRCutOff && reading2 < Sensorbubble2
				&& reading7 > FBLRCutOff && reading7 < Sensorbubble7) {
			area1 = reading2;

		} else {

			area1 = nothing;
		}
	} else {
		area1 = nothing;
		area2 = nothing;
		area3 = nothing;
		area4 = nothing;
		area5 = nothing;
	}

////back

	if (reading5 < Sensorbubble5 || reading6 < Sensorbubble5) { // is something seen
		if ((reading5 > reading6 - 300 && reading5 < reading6 + 300)) {
					if (reading5 > reading6) {
						area11 = reading6;
					} else {
						area11 = reading5;
					}
				} else {
					area11 = nothing;
				}
		if (reading5 < FBLRCutOff && reading5 < Sensorbubble5) {
			area8 = reading1;
		} else {
			area8 = nothing;
		}

		if (reading6 < FBLRCutOff && reading6 < Sensorbubble6) {
			area9 = reading2;
		} else {
			area9 = nothing;
		}

		if (reading5 > FBLRCutOff && reading5 < Sensorbubble5) {
			area11 = reading5;
			area12 = reading5;
		} else {

			area12 = nothing;
		}
		if (reading6 > FBLRCutOff && reading6 < Sensorbubble6) {

			area11 = reading6;
			area10 = reading6;
		} else {

			area10 = nothing;
		}


		//back right

	} else {
		area8 = nothing;
		area9 = nothing;
		area10 = nothing;
		area11 = nothing;
		area12 = nothing;
	}

//store all areas in an array
	Distance[0] = area1;
	Distance[1] = area2;
	Distance[2] = area3;
	Distance[3] = area4;
	Distance[4] = area5;
	Distance[5] = area6;
	Distance[6] = area7;
	Distance[7] = area8;
	Distance[8] = area9;
	Distance[9] = area10;
	Distance[10] = area11;
	Distance[11] = area12;
	Distance[12] = area13;
	Distance[13] = area14;
	Distance[14] = area15;
	Distance[15] = area16;

	if (Distance[0] != nothing) {
		frontRightOb = 1;
		frontRightX = 100; // all these values are used to display the positions on stmStudio
		frontRightY = 200;
		//printf("Front left obstacle \n");
	}
	if (Distance[1] != nothing) {
		frontY = 200;
		frontX = 0;
		frontOb = 1;
		//printf("Front obstacle \n");
	}
	if (Distance[2] != nothing) {
		frontLeftOb = 1;
		frontLeftX = -100;
		frontLeftY = 200;

		//printf("Front right obstacle \n");
	}
	if (Distance[3] != nothing) {
		frontOb = 1;
		frontY = 200;
		frontX = 0;
		//printf("Front obstacle \n");
	}
	if (Distance[4] != nothing) {
		frontOb = 1;
		frontY = 200;
		frontX = 0;
		//	printf("Front obstacle \n");
	}
	if (Distance[5] != nothing) {
		frontLeftOb = 1;
		frontLeftX = -100;
		frontLeftY = 200;
		//printf("Front  left obstacle \n");
	}
	if (Distance[6] != nothing) {
		frontRightOb = 1;
		frontRightX = 100;
		frontRightY = 200;
		//printf("front right obstacle \n");
	}
	if (Distance[7] != nothing) {
		rearOb = 1;
		backY = -200;
		backX = 0;
		//printf(" rear obstacle \n");
	}
	if (Distance[8] != nothing) {
		rearOb = 1;
		backY = -200;
		backX = 0;
		//printf(" rear obstacle \n");
	}
	if (Distance[9] != nothing) {
		rearLeftOb = 1;
		backLeftY = -200;
		backLeftX = -100;
		//	printf(" rear left obstacle \n");
	}
	if (Distance[10] != nothing) {
		rearOb = 1;
		backY = -200;
		backX = 0;
		//printf(" rear obstacle \n");
	}
	if (Distance[11] != nothing) {
		rearRightOb = 1;
		backRightY = -200;
		backRightX = 100;
		//printf(" rear right obstacle \n");
	}
	if (Distance[12] != nothing) {
		leftOb = 1;
		leftX = -100;
		leftY = 0;
		//printf(" right obstacle \n");
	}
	if (Distance[13] != nothing) {
		rightOb = 1;
		rightX = 100;
		leftY = 0;
		//printf(" left obstacle \n");
	}
	if (Distance[14] != nothing) {
		frontRightOb = 1;
		frontRightX = 100;
		frontRightY = 200;
		//printf(" left obstacle \n");
	}
	if (Distance[15] != nothing) {
		frontLeftOb = 1;
		frontLeftX = -100;
		frontLeftY = 200;
	}
	///////////ended up not using see below////////////Calculations for saftey bubble///////////////////////////
	//void saftyBubble();
	return 0;
}

/////// ended up not goign with this method but could be used in the future for more accuracy found other way worked fine//////////////////taken logic from http://www.dcs.gla.ac.uk/~pat/52233/slides/Geometry1x1.pdf /////////////////////
void saftyBubble() {
	Point polygon1[] = { { 0, 0 }, { 10 + dir, 0 + velocity }, { 10 + dir, 10
			+ velocity }, { 0 + dir, 10 + velocity },
			{ 0 + dir, 10 + velocity }, { 0, 10 } }; // define front polygon
	int n = sizeof(polygon1) / sizeof(polygon1[0]);
	Point p = { 5, 1 };
	int angle1030 = 10; // define sensor angle 1030
	int angle1000 = 0;  //difien snesor angle 1000
	for (int k; k > 16; k++) {
		if (Distance[k] != nothing) {
			sensorNum = checkSeonsorPosition(Distance[k]);
		}
		if (k < 7) {
			p.x = (sensorXY[sensorNum][1] + Distance[k]) * sin(angle1030);
			p.y = (sensorXY[sensorNum][1]) + Distance[k];
		}
		if (k >= 12) {
			p.x = (sensorXY[sensorNum][1] + Distance[k]) * sin(angle1000);
			p.y = (sensorXY[sensorNum][1]) + Distance[k];
		}

	}
	int a = isInside(polygon1, n, p);
	Point polygon2[] = { { 0, 0 }, { 10, 0 }, { 10, 10 }, { 0, 10 } }; // define back polygon
	int n2 = sizeof(polygon1) / sizeof(polygon1[0]);

}
int checkSeonsorPosition(float dist) { // check to see sensor position x and y  return: 0--> front left 1--> front right 2-->left 3-->right 4--> backleft 5 -->back right
	if (dist == reading1) {
		return 0;
	} else if (dist == reading2) {
		return 1;
	} else if (dist == reading3) {
		return 2;
	} else if (dist == reading4) {
		return 3;
	} else if (dist == reading5) {
		return 4;
	} else if (dist == reading6) {
		return 5;
	} else if (dist == reading7) {
		return 0;
	} else if (dist == reading8) {
		return 0;
	} else {
		return 0;
	}

}
int onSegment(Point p, Point q, Point r) {
	if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && q.y <= max(p.y, r.y)
			&& q.y >= min(p.y, r.y))
		return 1;
	return 0;
}
int orientation(Point p, Point q, Point r) {
	int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

	if (val == 0)
		return 0; // colinear
	return (val > 0) ? 1 : 2; // clock or counterclock wise
}

int doIntersect(Point p1, Point q1, Point p2, Point q2) {
	// Find the four orientations needed for general and
	// special cases
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	// General case
	if (o1 != o2 && o3 != o4)
		return 1;

	// Special Cases
	// p1, q1 and p2 are colinear and p2 lies on segment p1q1
	if (o1 == 0 && onSegment(p1, p2, q1))
		return 1;

	// p1, q1 and p2 are colinear and q2 lies on segment p1q1
	if (o2 == 0 && onSegment(p1, q2, q1))
		return 1;

	// p2, q2 and p1 are colinear and p1 lies on segment p2q2
	if (o3 == 0 && onSegment(p2, p1, q2))
		return 1;

	// p2, q2 and q1 are colinear and q1 lies on segment p2q2
	if (o4 == 0 && onSegment(p2, q1, q2))
		return 1;

	return 0; // Doesn't fall in any of the above cases
}
int isInside(Point polygon[], int n, Point p) {
	// There must be at least 3 vertices in polygon[]
	if (n < 3)
		return 0;

	// Create a point for line segment from p to infinite
	Point extreme = { 10000, p.y };

	// Count intersections of the above line with sides of polygon
	int count = 0, i = 0;
	do {
		int next = (i + 1) % n;

		// Check if the line segment from 'p' to 'extreme' intersects
		// with the line segment from 'polygon[i]' to 'polygon[next]'
		if (doIntersect(polygon[i], polygon[next], p, extreme)) {
			// If the point 'p' is colinear with line segment 'i-next',
			// then check if it lies on segment. If it lies, return true,
			// otherwise false
			if (orientation(polygon[i], p, polygon[next]) == 0)
				return onSegment(polygon[i], p, polygon[next]);

			count++;
		}
		i = next;
	} while (i != 0);

	// Return true if count is odd, false otherwise
	return count & 1; // Same as (count%2 == 1)
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
void assert_failed(char *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
