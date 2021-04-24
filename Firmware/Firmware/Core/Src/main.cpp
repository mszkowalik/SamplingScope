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

#include "../DAC/DAC.h"
#include "../DelayLine/DelayLine.h"
#include "../Pin/Pin.h"
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

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;

/* USER CODE BEGIN PV */
Pin* MAIN_DELAY_Q0, *MAIN_DELAY_Q1, *MAIN_DELAY_Q2, *MAIN_DELAY_Q3, *MAIN_DELAY_Q4, *MAIN_DELAY_Q5, *MAIN_DELAY_Q6, *MAIN_DELAY_Q7, *MAIN_DELAY_Q8, *MAIN_DELAY_Q9, *MAIN_DELAY_Q10;
Pin* STEP_GEN_DELAY_Q0 ,*STEP_GEN_DELAY_Q1 , *STEP_GEN_DELAY_Q2, *STEP_GEN_DELAY_Q3, *STEP_GEN_DELAY_Q4, *STEP_GEN_DELAY_Q5, *STEP_GEN_DELAY_Q6, *STEP_GEN_DELAY_Q7, *STEP_GEN_DELAY_Q8, *STEP_GEN_DELAY_Q9, *STEP_GEN_DELAY_Q10;
Pin* CLK_EN_1 , *CLK_EN_0, *CLK_SEL_1, *CLK_SEL_0, *CNTR_RESET;






/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
DAC *VREF1;
DAC *VREF2;
DAC *DELAY_MAIN_DAC;
DAC *DELAY_STEP_DAC;
DelayLine *MAIN_DELAY;
DelayLine *STEP_DELAY;


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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	MAIN_DELAY_Q0 = new Pin(MAIN_DELAY_Q0_Pin, MAIN_DELAY_Q0_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q1 = new Pin(MAIN_DELAY_Q1_Pin, MAIN_DELAY_Q1_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q2 = new Pin(MAIN_DELAY_Q2_Pin, MAIN_DELAY_Q2_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q3 = new Pin(MAIN_DELAY_Q3_Pin, MAIN_DELAY_Q3_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q4 = new Pin(MAIN_DELAY_Q4_Pin, MAIN_DELAY_Q4_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q5 = new Pin(MAIN_DELAY_Q5_Pin, MAIN_DELAY_Q5_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q6 = new Pin(MAIN_DELAY_Q6_Pin, MAIN_DELAY_Q6_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q7 = new Pin(MAIN_DELAY_Q7_Pin, MAIN_DELAY_Q7_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q8 = new Pin(MAIN_DELAY_Q8_Pin, MAIN_DELAY_Q8_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q9 = new Pin(MAIN_DELAY_Q9_Pin, MAIN_DELAY_Q9_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	MAIN_DELAY_Q10 = new Pin(MAIN_DELAY_Q10_Pin, MAIN_DELAY_Q10_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);

	STEP_GEN_DELAY_Q0 = new Pin(STEP_GEN_DELAY_Q0_Pin, STEP_GEN_DELAY_Q0_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q1 = new Pin(STEP_GEN_DELAY_Q1_Pin, STEP_GEN_DELAY_Q1_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q2 = new Pin(STEP_GEN_DELAY_Q2_Pin, STEP_GEN_DELAY_Q2_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q3 = new Pin(STEP_GEN_DELAY_Q3_Pin, STEP_GEN_DELAY_Q3_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q4 = new Pin(STEP_GEN_DELAY_Q4_Pin, STEP_GEN_DELAY_Q4_GPIO_Port, GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q5 = new Pin(STEP_GEN_DELAY_Q5_Pin, STEP_GEN_DELAY_Q5_GPIO_Port,GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q6 = new Pin(STEP_GEN_DELAY_Q6_Pin, STEP_GEN_DELAY_Q6_GPIO_Port,GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q7 = new Pin(STEP_GEN_DELAY_Q7_Pin, STEP_GEN_DELAY_Q7_GPIO_Port,GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q8 = new Pin(STEP_GEN_DELAY_Q8_Pin, STEP_GEN_DELAY_Q8_GPIO_Port,GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q9 = new Pin(STEP_GEN_DELAY_Q9_Pin, STEP_GEN_DELAY_Q9_GPIO_Port,GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);
	STEP_GEN_DELAY_Q10 = new Pin(STEP_GEN_DELAY_Q10_Pin, STEP_GEN_DELAY_Q10_GPIO_Port,GPIO_MODE_OUTPUT_PP,GPIO_PIN_RESET);

	CLK_EN_1 = new Pin(CLK_EN_1_Pin, CLK_EN_1_GPIO_Port,GPIO_MODE_OUTPUT_PP, GPIO_PIN_SET);
	CLK_EN_0 = new Pin(CLK_EN_0_Pin, CLK_EN_0_GPIO_Port,GPIO_MODE_OUTPUT_PP, GPIO_PIN_SET);
	CLK_SEL_1 = new Pin(CLK_SEL_1_Pin, CLK_SEL_1_GPIO_Port,GPIO_MODE_OUTPUT_PP, GPIO_PIN_SET);
	CLK_SEL_0 = new Pin(CLK_SEL_0_Pin, CLK_SEL_0_GPIO_Port,GPIO_MODE_OUTPUT_PP, GPIO_PIN_RESET);
	CNTR_RESET = new Pin(CNTR_RESET_Pin, CNTR_RESET_GPIO_Port,GPIO_MODE_OUTPUT_PP, GPIO_PIN_SET);

	Pin* PROG_DELAY_MAIN [11] = {
			MAIN_DELAY_Q0,
			MAIN_DELAY_Q1,
			MAIN_DELAY_Q2,
			MAIN_DELAY_Q3,
			MAIN_DELAY_Q4,
			MAIN_DELAY_Q5,
			MAIN_DELAY_Q6,
			MAIN_DELAY_Q7,
			MAIN_DELAY_Q8,
			MAIN_DELAY_Q9,
			MAIN_DELAY_Q10
	};

	Pin* PROG_DELAY_STEP_GEN [11] = {
			STEP_GEN_DELAY_Q0,
			STEP_GEN_DELAY_Q1,
			STEP_GEN_DELAY_Q2,
			STEP_GEN_DELAY_Q3,
			STEP_GEN_DELAY_Q4,
			STEP_GEN_DELAY_Q5,
			STEP_GEN_DELAY_Q6,
			STEP_GEN_DELAY_Q7,
			STEP_GEN_DELAY_Q8,
			STEP_GEN_DELAY_Q9,
			STEP_GEN_DELAY_Q10
	};

	VREF1 = new DAC(&hspi1, 2.048f);
	VREF2 = new DAC(&hspi2, 2.048f);
	DELAY_MAIN_DAC = new DAC(&hspi3, 2.048f);
	DELAY_STEP_DAC = new DAC(&hspi4, 2.048f);

	MAIN_DELAY = new DelayLine(PROG_DELAY_MAIN, DELAY_MAIN_DAC);
	STEP_DELAY = new DelayLine(PROG_DELAY_STEP_GEN, DELAY_STEP_DAC);
  //HAL_GPIO_WritePin(CLK_EN_1_GPIO_Port, CLK_EN_1_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(CLK_EN_0_GPIO_Port, CLK_EN_0_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(CLK_SEL_1_GPIO_Port, CLK_SEL_1_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(CLK_SEL_0_GPIO_Port, CLK_SEL_0_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(CNTR_RESET_GPIO_Port, CNTR_RESET_Pin, GPIO_PIN_SET);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //HAL_GPIO_WritePin(INT_CLK_GPIO_Port, INT_CLK_Pin, GPIO_PIN_SET);
	  //HAL_GPIO_WritePin(CLK_SEL_0_GPIO_Port, CLK_SEL_0_Pin, GPIO_PIN_SET);
	  CLK_SEL_0->write(GPIO_PIN_SET);
	  MAIN_DELAY->setDelay(0x7FFFFF);
	  HAL_Delay(3000);
//
//	  HAL_GPIO_WritePin(INT_CLK_GPIO_Port, INT_CLK_Pin, GPIO_PIN_RESET);
//	  HAL_Delay(100);
//
//	  HAL_GPIO_WritePin(CLK_SEL_0_GPIO_Port, CLK_SEL_0_Pin, GPIO_PIN_SET);
	  CLK_SEL_0->write(GPIO_PIN_SET);
	  MAIN_DELAY->setDelay(0x0);
	  HAL_Delay(3000);


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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x60404E72;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SIMO_EN_GPIO_Port, SIMO_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VREF2_LDAC_Pin|MAIN_DELAY_Q0_Pin|USB_PD_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ECL_EN_GPIO_Port, ECL_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, MAIN_DELAY_Q10_Pin|MAIN_DELAY_Q9_Pin|MAIN_DELAY_Q8_Pin|MAIN_DELAY_Q7_Pin
                          |MAIN_DELAY_Q6_Pin|MAIN_DELAY_Q5_Pin|MAIN_DELAY_Q4_Pin|MAIN_DELAY_Q3_Pin
                          |MAIN_DELAY_Q2_Pin|MAIN_DELAY_Q1_Pin|STEP_GEN_DELAY_Q10_Pin|STEP_GEN_DELAY_Q9_Pin
                          |STEP_GEN_DELAY_Q8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PD_MAIN_LDAC_GPIO_Port, PD_MAIN_LDAC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, STEP_GEN_DELAY_Q0_Pin|STEP_GEN_DELAY_Q3_Pin|CNTR_RESET_Pin|PD_STEP_LDAC_Pin
                          |CNTR_Q3_Pin|CNTR_Q2_Pin|CNTR_Q1_Pin|TM_IN1H_Pin
                          |TM_IN1L_Pin|TM_IN2H_Pin|TM_IN2L_Pin|TM_nSLEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, STEP_GEN_DELAY_Q4_Pin|STEP_GEN_DELAY_Q5_Pin|STEP_GEN_DELAY_Q6_Pin|STEP_GEN_DELAY_Q7_Pin
                          |STEP_GEN_DELAY_Q2_Pin|STEP_GEN_DELAY_Q1_Pin|CLK_SEL_0_Pin|INT_CLK_Pin
                          |CLK_SEL_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CLK_EN_1_Pin|CLK_EN_0_Pin|VREF1_LDAC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SIMO_EN_Pin STEP_GEN_DELAY_Q4_Pin STEP_GEN_DELAY_Q5_Pin STEP_GEN_DELAY_Q6_Pin
                           STEP_GEN_DELAY_Q7_Pin STEP_GEN_DELAY_Q2_Pin STEP_GEN_DELAY_Q1_Pin CLK_SEL_0_Pin
                           INT_CLK_Pin CLK_SEL_1_Pin */
  GPIO_InitStruct.Pin = SIMO_EN_Pin|STEP_GEN_DELAY_Q4_Pin|STEP_GEN_DELAY_Q5_Pin|STEP_GEN_DELAY_Q6_Pin
                          |STEP_GEN_DELAY_Q7_Pin|STEP_GEN_DELAY_Q2_Pin|STEP_GEN_DELAY_Q1_Pin|CLK_SEL_0_Pin
                          |INT_CLK_Pin|CLK_SEL_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : VREF2_LDAC_Pin ECL_EN_Pin MAIN_DELAY_Q0_Pin USB_PD_RESET_Pin */
  GPIO_InitStruct.Pin = VREF2_LDAC_Pin|ECL_EN_Pin|MAIN_DELAY_Q0_Pin|USB_PD_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_PD_ALERT_Pin CNTR_Q0_Pin CNTR_Q14_Pin CNTR_Q13_Pin
                           CNTR_Q11_Pin CNTR_Q5_Pin */
  GPIO_InitStruct.Pin = USB_PD_ALERT_Pin|CNTR_Q0_Pin|CNTR_Q14_Pin|CNTR_Q13_Pin
                          |CNTR_Q11_Pin|CNTR_Q5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MAIN_DELAY_Q10_Pin MAIN_DELAY_Q9_Pin MAIN_DELAY_Q8_Pin MAIN_DELAY_Q7_Pin
                           MAIN_DELAY_Q6_Pin MAIN_DELAY_Q5_Pin MAIN_DELAY_Q4_Pin MAIN_DELAY_Q3_Pin
                           MAIN_DELAY_Q2_Pin MAIN_DELAY_Q1_Pin STEP_GEN_DELAY_Q10_Pin STEP_GEN_DELAY_Q9_Pin
                           STEP_GEN_DELAY_Q8_Pin */
  GPIO_InitStruct.Pin = MAIN_DELAY_Q10_Pin|MAIN_DELAY_Q9_Pin|MAIN_DELAY_Q8_Pin|MAIN_DELAY_Q7_Pin
                          |MAIN_DELAY_Q6_Pin|MAIN_DELAY_Q5_Pin|MAIN_DELAY_Q4_Pin|MAIN_DELAY_Q3_Pin
                          |MAIN_DELAY_Q2_Pin|MAIN_DELAY_Q1_Pin|STEP_GEN_DELAY_Q10_Pin|STEP_GEN_DELAY_Q9_Pin
                          |STEP_GEN_DELAY_Q8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : COMP_OUT_P_Pin CNTR_Q12_Pin CNTR_Q4_Pin CNTR_Q15_Pin */
  GPIO_InitStruct.Pin = COMP_OUT_P_Pin|CNTR_Q12_Pin|CNTR_Q4_Pin|CNTR_Q15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD_MAIN_LDAC_Pin */
  GPIO_InitStruct.Pin = PD_MAIN_LDAC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PD_MAIN_LDAC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP_GEN_DELAY_Q0_Pin STEP_GEN_DELAY_Q3_Pin CNTR_RESET_Pin PD_STEP_LDAC_Pin
                           CNTR_Q3_Pin CNTR_Q2_Pin CNTR_Q1_Pin TM_IN1H_Pin
                           TM_IN1L_Pin TM_IN2H_Pin TM_IN2L_Pin TM_nSLEEP_Pin */
  GPIO_InitStruct.Pin = STEP_GEN_DELAY_Q0_Pin|STEP_GEN_DELAY_Q3_Pin|CNTR_RESET_Pin|PD_STEP_LDAC_Pin
                          |CNTR_Q3_Pin|CNTR_Q2_Pin|CNTR_Q1_Pin|TM_IN1H_Pin
                          |TM_IN1L_Pin|TM_IN2H_Pin|TM_IN2L_Pin|TM_nSLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_EN_1_Pin CLK_EN_0_Pin VREF1_LDAC_Pin */
  GPIO_InitStruct.Pin = CLK_EN_1_Pin|CLK_EN_0_Pin|VREF1_LDAC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SAMPLE_CLK_Pin COMP_OUT_N_Pin */
  GPIO_InitStruct.Pin = SAMPLE_CLK_Pin|COMP_OUT_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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
