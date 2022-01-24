/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int kx022_readregs(int addr, uint8_t * data, int len) {
  int status = HAL_I2C_Mem_Read(&hi2c2, KX022_I2C_ADDRESS, addr, I2C_MEMADD_SIZE_8BIT, data, len, 10000);
  if (status != HAL_OK)
    return 1;
  else
    return 0;
}

int kx022_writeregs(int addr, uint8_t * data, int len) {
  int status =  HAL_I2C_Mem_Write(&hi2c2, (uint16_t)KX022_I2C_ADDRESS, addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len, 10000);
  if (status != HAL_OK)
    return 1;
  else
    return 0;
}

int16_t kx022_getAccAxis(uint8_t addr) {
    int16_t acc;
    uint8_t res[2];
 
    kx022_readregs(addr, res, 2);
    acc = ((res[1] << 8) | res[0]);
    return acc;
}

int16_t kx022_getAccX() {
    return (kx022_getAccAxis(KX022_XOUT_L));
}
 
int16_t kx022_getAccY() {
    return (kx022_getAccAxis(KX022_YOUT_L));
}
 
int16_t kx022_getAccZ() {
    return (kx022_getAccAxis(KX022_ZOUT_L));
}


void test_measure_light(void) {
  ADC_ChannelConfTypeDef sConfig = {0};
  char buffer[32] = { 0 };
  
  // Make sure other lights off to not interfere
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);

  // Power on measurement circuit (battery and light)
  HAL_GPIO_WritePin(MEAS_CTRL_PORT, MEAS_CTRL_PIN, GPIO_PIN_SET);
  // Delay to settle down
  HAL_Delay(1);

  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  if (HAL_ADC_Start(&hadc1) != HAL_OK) { Error_Handler(); }
  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) { Error_Handler(); }
  uint16_t uhADCxConvertedValue = HAL_ADC_GetValue(&hadc1);
  HAL_GPIO_WritePin(MEAS_CTRL_PORT, MEAS_CTRL_PIN, GPIO_PIN_RESET);

  sprintf(buffer, "LIGHT %d\r\n", uhADCxConvertedValue);
  HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), HAL_MAX_DELAY);
}

void test_accelerometer(void) {
    char buffer[32] = { 0 };
    int32_t accx = kx022_getAccX();
    int32_t accy = kx022_getAccY();
    int32_t accz = kx022_getAccZ();

    sprintf(buffer, "X/Y/Z %ld/%ld/%ld\r\n", accx, accy, accz);
    HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), HAL_MAX_DELAY);
    HAL_Delay(1);
}

void feature_light_down(void) {
    int32_t accx = kx022_getAccX();
    int32_t accy = kx022_getAccY();
    int32_t accz = kx022_getAccZ();
    int32_t absx;
    if (accx < 0)
        absx = accx * -1;
    else
        absx = accx;
    

    if (accz < -10000 && absx < 10000) {
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 700);
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 700);
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 700);
    } else {
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    }



}

/*
void police() {
    for (int i=0; i<5; i++) {
      HAL_Delay(50);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); //blue
      HAL_Delay(50);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 90); //blue
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); //blue
    
    for (int i=0; i<5; i++) {
      HAL_Delay(50);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
      HAL_Delay(50);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 90);
    }
    
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
}
*/
#define GRADUAL_MAXBRI 600
void test_slow_glow() {
  for (int i=0; i<GRADUAL_MAXBRI; i++) {
    HAL_Delay(5);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, i);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, i);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, i);
  }
  for (int i=0; i<GRADUAL_MAXBRI; i++) {
    HAL_Delay(5);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, GRADUAL_MAXBRI-i);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, GRADUAL_MAXBRI-i);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, GRADUAL_MAXBRI-i);
    
  }
}



void accel_start() {
    unsigned char buf = 0x0;
    unsigned char reg[2];

    int ret = kx022_readregs(KX022_WHO_AM_I, &buf, sizeof(buf));
    if (ret || buf != KX022_WAI_VAL) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1); //red
    }
    
    reg[0] = 0x41;
    kx022_writeregs(KX022_CNTL1, reg, 1);

    //reg[0] = 0x02;
    reg[0] = 0x06; //800hz
    kx022_writeregs(KX022_ODCNTL, reg, 1);

    reg[0] = 0xD8;
    kx022_writeregs(KX022_CNTL3, reg, 1);

    reg[0] = 0x01;
    kx022_writeregs(KX022_TILT_TIMER, reg, 1);

    reg[0] = 0xC1;
    kx022_writeregs(KX022_CNTL1, reg, 1);
    
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  //HAL_Delay(1000);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  MX_ADC1_Init();
  MX_I2C2_Init();
  //MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();

  // Force all LED to off
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); //blue
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); //red
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); //green
  accel_start();

  HAL_UART_Transmit(&huart2, "BOOT\r\n", 6, HAL_MAX_DELAY);
// Test/debug illumination sensor
#if 0
  while (1) { test_measure_light(); }
#endif

// Test/debug accelerometer
#if 0
  while (1) { test_accelerometer(); }
#endif

// Test: slow glow (brightness change)
//#if 0
  while (1) { test_slow_glow(); }
//#endif


// Feature: turn on RGB light only when light looks downwards
#if 0
  while (1) { feature_light_down(); }
#endif

  while (1)
  {
    //float accx = kx022_getAccX();
    //float accy = kx022_getAccY();
    //float accz = kx022_getAccZ();
    /*
    if (accx != 0.0) {
      if (test) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1); //red
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); //green
        test=0;
      } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); //red
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1); //green
        test=1;
      }
    }
    
    
    if (accx > 0) {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (int)(accx*10.0));
    } else {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (int)(accx*-10.0));
    }
    if (accy > 0) {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (int)(accy*10.0));
    } else {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (int)(accy*-10.0));
    }
    if (accz > 0) {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (int)(accz*10.0));
    } else {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (int)(accz*-10.0));
    }
    */

    //gradual();
    // Measure light
    //meas_light();
    //HAL_Delay(250);
    //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
    
    //police();
    /* USER CODE END WHILE */
    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  //RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableLSECSS();
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  //hi2c2.Init.Timing = 0x10707DBC; // 100khz
  hi2c2.Init.Timing = 0x00602173; // 400khz
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

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
  HAL_TIM_Base_Start(&htim3);
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }
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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB7 PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

