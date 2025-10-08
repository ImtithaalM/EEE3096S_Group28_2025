/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include "stm32f4xx.h"
#include "lcd_stm32f4.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables
#define NS         128    // Number of samples in LUT
#define TIM2CLK    16000000// STM Clock frequency: Hint You might want to check the ioc file
#define F_SIGNAL   1000    // Frequency of output analog signal

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs
uint32_t Sin_LUT[128] = {
    2048, 2148, 2248, 2348, 2447, 2545, 2642, 2737,
    2831, 2923, 3013, 3100, 3185, 3267, 3346, 3423,
    3495, 3565, 3630, 3692, 3750, 3804, 3853, 3898,
    3939, 3975, 4007, 4034, 4056, 4073, 4085, 4093,
    4095, 4093, 4085, 4073, 4056, 4034, 4007, 3975,
    3939, 3898, 3853, 3804, 3750, 3692, 3630, 3565,
    3495, 3423, 3346, 3267, 3185, 3100, 3013, 2923,
    2831, 2737, 2642, 2545, 2447, 2348, 2248, 2148,
    2048, 1947, 1847, 1747, 1648, 1550, 1453, 1358,
    1264, 1172, 1082,  995,  910,  828,  749,  672,
     600,  530,  465,  403,  345,  291,  242,  197,
     156,  120,   88,   61,   39,   22,   10,    2,
       0,    2,   10,   22,   39,   61,   88,  120,
     156,  197,  242,  291,  345,  403,  465,  530,
     600,  672,  749,  828,  910,  995, 1082, 1172,
    1264, 1358, 1453, 1550, 1648, 1747, 1847, 1947
};

uint32_t Saw_LUT[128] = {
       0,   32,   64,   96,  128,  160,  192,  224,
     256,  288,  320,  352,  384,  416,  448,  480,
     512,  544,  576,  608,  640,  672,  704,  736,
     768,  800,  832,  864,  896,  928,  960,  992,
    1024, 1056, 1088, 1120, 1152, 1184, 1216, 1248,
    1280, 1312, 1344, 1376, 1408, 1440, 1472, 1504,
    1536, 1568, 1600, 1632, 1664, 1696, 1728, 1760,
    1792, 1824, 1856, 1888, 1920, 1952, 1984, 2016,
    2048, 2079, 2111, 2143, 2175, 2207, 2239, 2271,
    2303, 2335, 2367, 2399, 2431, 2463, 2495, 2527,
    2559, 2591, 2623, 2655, 2687, 2719, 2751, 2783,
    2815, 2847, 2879, 2911, 2943, 2975, 3007, 3039,
    3071, 3103, 3135, 3167, 3199, 3231, 3263, 3295,
    3327, 3359, 3391, 3423, 3455, 3487, 3519, 3551,
    3583, 3615, 3647, 3679, 3711, 3743, 3775, 3807,
    3839, 3871, 3903, 3935, 3967, 3999, 4031, 4063
};

uint32_t Triangle_LUT[128] = {
       0,   64,  128,  192,  256,  320,  384,  448,
     512,  576,  640,  704,  768,  832,  896,  960,
    1024, 1088, 1152, 1216, 1280, 1344, 1408, 1472,
    1536, 1600, 1664, 1728, 1792, 1856, 1920, 1984,
    2048, 2111, 2175, 2239, 2303, 2367, 2431, 2495,
    2559, 2623, 2687, 2751, 2815, 2879, 2943, 3007,
    3071, 3135, 3199, 3263, 3327, 3391, 3455, 3519,
    3583, 3647, 3711, 3775, 3839, 3903, 3967, 4031,
    4095, 4031, 3967, 3903, 3839, 3775, 3711, 3647,
    3583, 3519, 3455, 3391, 3327, 3263, 3199, 3135,
    3071, 3007, 2943, 2879, 2815, 2751, 2687, 2623,
    2559, 2495, 2431, 2367, 2303, 2239, 2175, 2111,
    2048, 1984, 1920, 1856, 1792, 1728, 1664, 1600,
    1536, 1472, 1408, 1344, 1280, 1216, 1152, 1088,
    1024,  960,  896,  832,  768,  704,  640,  576,
     512,  448,  384,  320,  256,  192,  128,   64
};

uint32_t Piano_LUT[128] = {
    1901, 2047, 1834, 1956, 1916, 1948, 1892, 2020,
    1954, 2015, 1939, 1891, 1887, 2208, 1914, 1599,
    1881, 1899, 1831, 1909, 1919, 1905, 1404, 1876,
    2167, 1907, 2798, 1521, 1592, 1678, 2164, 2057,
    1348, 2026, 1817, 1917, 1834, 1946, 1565, 1799,
    1780, 1959, 2078, 1778, 2129, 1775, 2511, 1883,
    1950, 1916, 1847, 1918, 1877, 1890, 2663, 1801,
    2231, 1700, 1850, 1796, 1285, 1767, 1521, 1889,
    1613, 1926, 2004, 1900, 1871, 1871, 1549, 2014,
    1745, 1943, 1620, 2373, 2049, 1885, 2237, 2038,
    1884, 1895, 1852, 1928, 1887, 1894, 2047, 1908,
    1961, 1774, 1682, 1887, 1862, 2094, 1852, 1963,
    2222, 1907, 1933, 1892, 1928, 1875, 1588, 1767,
    1843, 1848, 2044, 1929, 1889, 1849, 1676, 2012,
    1861, 1936, 1844, 1928, 1884, 1915, 1790, 1717,
    1433, 1615, 1933, 1910, 1856, 1812, 1920, 1901

};

uint32_t Guitar_LUT[128] = {
    1970, 2303, 2295, 2049, 2029, 1777, 1748, 1721,
    1411, 2017, 2005, 1977, 2011, 2000, 1821, 1959,
    1966, 1950, 1615, 2442, 1821, 1803, 2405, 2119,
    1732, 1976, 1937, 1985, 1958, 1915, 1879, 1977,
    1976, 2025, 1781, 2076, 1778, 2056, 1924, 1662,
    2161, 1947, 1980, 1975, 1988, 1852, 2068, 1963,
    1793, 1913, 2121, 1931, 2119, 2070, 1808, 1926,
    2034, 2065, 2020, 1757, 2156, 2022, 2661, 2183,
    2031, 2160, 2209, 2178, 2065, 2056, 1392, 1595,
    1993, 1966, 2004, 1987, 2220, 1967, 1954, 1979,
    2092, 2108, 1722, 2072, 2004, 1643, 2006, 1797,
    2012, 2025, 1969, 1956, 1848, 1806, 2040, 1968,
    2204, 1891, 2177, 1906, 1952, 1923, 2483, 2362,
    1933, 1975, 1957, 1964, 2040, 1851, 1968, 1974,
    1844, 1950, 2076, 1816, 2231, 1863, 2067, 1562,
    2031, 1754, 2324, 1823, 1555, 2565, 1852, 1971

};

uint32_t Drum_LUT[128] = {
    2048, 3379, 1865, 2110, 2064, 3426, 1893, 1934,
    2086, 2130, 2036, 2031, 2039, 1937, 1414, 1849,
    2095, 2021, 2045, 2059, 2048, 2072, 2044, 2021,
    2128, 1817, 2073, 2045, 2131, 2042, 3236, 1276,
    1437, 2190, 2436, 2105, 1774, 2082, 2029, 2032,
    4094, 1963, 2036, 2042, 2438, 2273, 1820, 1996,
    2104, 2071, 2018, 2052,  793, 2419, 2076, 2069,
      55, 2240, 2043, 2024, 2161, 2049, 2903, 2387,
    4067, 2013, 2286, 2039, 2536, 2711, 2048, 2096,
    1760, 2003, 2086, 2043, 2638, 1937, 2062, 2076,
    2014, 2057, 2043, 2050, 2212, 2048, 2066, 2048,
    2921, 2071, 2018, 2081, 2007, 2049, 1831, 2014,
     855, 2302, 1978, 2041, 2037, 2037, 2107, 2043,
    1641, 2057, 2067, 2032, 2045, 2439, 2145, 2218,
    2118, 2061, 2039, 1782, 2166, 2427, 2133, 4095,
    2154, 2011, 2036, 1615, 1958,  297, 2525, 2048

};


// TODO: Equation to calculate TIM2_Ticks
//uint32_t TIM2_Ticks = 0; // How often to write new LUT value

uint32_t TIM2_Ticks = TIM2CLK / (NS * F_SIGNAL);

uint32_t DestAddress = (uint32_t) &(TIM3->CCR3); // Write LUT TO TIM3->CCR3 to modify PWM duty cycle


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void EXTI0_IRQHandler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // TODO: Start TIM3 in PWM mode on channel 3

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // TODO: Start TIM2 in Output Compare (OC) mode on channel 1

  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);

  // TODO: Start DMA in IT mode on TIM2->CH1. Source is LUT and Dest is TIM3->CCR3; start with Sine LUT

  HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)Sin_LUT, (uint32_t)&TIM3->CCR3, NS);

  // TODO: Write current waveform to LCD(Sine is the first waveform)

  lcd_clear();
  lcd_putstring("Sine");

  // TODO: Enable DMA (start transfer from LUT to CCR)

  HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM2_Ticks - 1;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* TIM2_CH1 DMA Init */
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_tim2_ch1.Instance = DMA1_Stream5;
  hdma_tim2_ch1.Init.Channel = DMA_CHANNEL_3;         // TIM2_CH1 is on channel 3
  hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH; // Memory -> TIM3->CCR3
  hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;    // Peripheral address fixed
  hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;        // Memory address increments
  hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_tim2_ch1.Init.Mode = DMA_CIRCULAR;            // Repeat LUT automatically
  hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_tim2_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK)
  {
      Error_Handler();
  }

  /* Link DMA handle to TIM2 handle */
  __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);
  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // -------------------------------
  // LCD pins configuration
  // -------------------------------
  // Configure PC14 (RS) and PC15 (E) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Configure PB8 (D4) and PB9 (D5) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure PA12 (D6) and PA15 (D7) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Set all LCD pins LOW initially
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_RESET);


  // -------------------------------
  // Button0 configuration (PA0)
  // -------------------------------
  GPIO_InitStruct.Pin = Button0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Interrupt on rising edge
  GPIO_InitStruct.Pull = GPIO_PULLUP;         // Use pull-up resistor
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Enable and set EXTI line 0 interrupt priority
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

// Declare waveform switching variables
uint8_t waveform_index = 0;
uint32_t *current_LUT = Sin_LUT;
uint32_t last_button_press = 0;

/* USER CODE BEGIN 4 */
void EXTI0_IRQHandler(void){

	// TODO: Debounce using HAL_GetTick()

	static uint32_t last_button_press = 0;
	    uint32_t now = HAL_GetTick();
	    if (now - last_button_press < 200) {
	        HAL_GPIO_EXTI_IRQHandler(Button0_Pin);
	        return; // Debounce: ignore if pressed too soon
	    }
	    last_button_press = now;


	// TODO: Disable DMA transfer and abort IT, then start DMA in IT mode with new LUT and re-enable transfer
	// HINT: Consider using C's "switch" function to handle LUT changes

	static uint8_t waveform_index = 0;
	waveform_index = (waveform_index + 1) % 6;

	__HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
	HAL_DMA_Abort_IT(&hdma_tim2_ch1);

		uint32_t *current_LUT;
	    	switch (waveform_index) {
	    		case 0:
	    			current_LUT = Sin_LUT;
	    	        lcd_clear();
	    	        lcd_putstring("Sine");
	    	        break;
	    	    case 1:
	    	        current_LUT = Saw_LUT;
	    	        lcd_clear();
	    	        lcd_putstring("Sawtooth");
	    	        break;
	    	    case 2:
	    	        current_LUT = Triangle_LUT;
	    	        lcd_clear();
	    	        lcd_putstring("Triangle");
	    	        break;
	    	    case 3:
	    	        current_LUT = Piano_LUT;
	    	        lcd_clear();
	    	        lcd_putstring("Piano");
	    	        break;
	    	    case 4:
	    	        current_LUT = Guitar_LUT;
	    	        lcd_clear();
	    	        lcd_putstring("Guitar");
	    	        break;
	    	    case 5:
	    	        current_LUT = Drum_LUT;
	    	        lcd_clear();
	    	        lcd_putstring("Drum");
	                break;
	    	        }

	    	        HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)current_LUT, (uint32_t)&TIM3->CCR3, NS);
	    	        __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

	    	        HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
	    	    }


	    	HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
