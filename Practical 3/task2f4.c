/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Mandelbrot benchmark for STM32F4)
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
#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_ITER        100
#define NUM_TESTS       5
#define CPU_FREQ_HZ     120000000UL   /* 120 MHz as configured in SystemClock_Config */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//TODO: Define variables you think you might need
// - Performance timing variables (e.g execution time, throughput, pixels per second, clock cycles)

volatile uint32_t start_ms = 0;
volatile uint32_t end_ms = 0;
volatile uint32_t elapsed_ms = 0;

volatile uint32_t start_cycles = 0;
volatile uint32_t end_cycles = 0;
volatile uint32_t elapsed_cycles = 0;

/* Image sizes (square) */
//const int image_sizes[NUM_TESTS] = {128, 160, 192, 224, 256};

const int widths[]  = {128, 256, 512, 800, 1280, 1920};
const int heights[] = {128, 256, 512, 600,  720, 1080};
#define NUM_SIZES (sizeof(widths)/sizeof(widths[0]))



/* Results arrays */
volatile uint32_t exec_time_fixed_ms[NUM_TESTS] = {0};
volatile uint32_t exec_time_double_ms[NUM_TESTS] = {0};
volatile uint32_t exec_cycles_fixed[NUM_TESTS] = {0};
volatile uint32_t exec_cycles_double[NUM_TESTS] = {0};
volatile uint64_t checksum_fixed[NUM_TESTS] = {0};
volatile uint64_t checksum_double[NUM_TESTS] = {0};
volatile uint32_t checksum[NUM_TESTS] = {0};
volatile uint32_t execution_time[NUM_TESTS] = {0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
//TODO: Define any function prototypes you might need such as the calculate Mandelbrot function among others

/* Mandelbrot function prototypes */
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);

/* DWT / cycle counter helpers */
static inline void dwt_enable_cycle_counter(void);
static inline uint32_t dwt_get_cycle_count(void);

/* Helper to compute pixels/sec from cycles */
static inline uint32_t compute_pixels_per_sec(uint32_t pixels, uint32_t cycles);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Enable DWT cycle counter (Cortex-M4) */
static inline void dwt_enable_cycle_counter(void)
{
  /* Enable trace and debug block */
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }
  /* Reset and enable the cycle counter */
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Read cycle counter */
static inline uint32_t dwt_get_cycle_count(void)
{
  return DWT->CYCCNT;
}

/* compute pixels/sec given cycles and known CPU freq */
static inline uint32_t compute_pixels_per_sec(uint32_t pixels, uint32_t cycles)
{
  if (cycles == 0) return 0;
  /* time_seconds = cycles / CPU_FREQ_HZ; pixels/sec = pixels / time_seconds */
  /* = pixels * CPU_FREQ_HZ / cycles */
  uint64_t val = (uint64_t)pixels * (uint64_t)CPU_FREQ_HZ;
  val = val / (uint64_t)cycles;
  if (val > 0xFFFFFFFFULL) return 0xFFFFFFFFU;
  return (uint32_t)val;
}

/* Fixed-point Mandelbrot (using 16-bit fractional part, Q16.16) */
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations)
{
    uint64_t mandelbrot_sum = 0;

    const int32_t scale_shift = 16;          /* Q16.16 */
    const int32_t scale_factor = (1 << scale_shift);
    const int32_t escape_limit = 4 << scale_shift;

    /* Precompute scales in fixed point:
       map x in [0,width-1] -> real in [-2.5, 1.0] span 3.5
       map y in [0,height-1] -> imag in [-1.0, 1.0] span 2.0
    */
    int32_t x_step = (int32_t)((3.5 * (double)scale_factor) / (double)width + 0.5);
    int32_t y_step = (int32_t)((2.0 * (double)scale_factor) / (double)height + 0.5);
    int32_t x_offset = - (int32_t)(2.5 * (double)scale_factor + 0.5);
    int32_t y_offset = - (int32_t)(1.0 * (double)scale_factor + 0.5);

    for (int y = 0; y < height; y++)
    {
        int32_t y0 = y * y_step + y_offset;
        for (int x = 0; x < width; x++)
        {
            int32_t x0 = x * x_step + x_offset;
            int32_t xi = 0;
            int32_t yi = 0;
            int iteration = 0;

            while (iteration < max_iterations)
            {
                /* xi_sq = (xi * xi) >> scale_shift */
                int64_t xi_sq = ((int64_t)xi * (int64_t)xi) >> scale_shift;
                int64_t yi_sq = ((int64_t)yi * (int64_t)yi) >> scale_shift;

                if ((xi_sq + yi_sq) > (int64_t)escape_limit) break;

                int32_t xi_yi = (int32_t)(((int64_t)xi * (int64_t)yi) >> (scale_shift - 1)); /* 2*xi*yi */

                int32_t xi_new = (int32_t)(xi_sq - yi_sq) + x0;
                int32_t yi_new = xi_yi + y0;

                xi = xi_new;
                yi = yi_new;

                iteration++;
            }

            mandelbrot_sum += (uint64_t)iteration;
        }
    }
    return mandelbrot_sum;
}

/* Double-precision Mandelbrot */
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations)
{
    uint64_t mandelbrot_sum = 0;

    const double x_step = 3.5 / (double)width;
    const double y_step = 2.0 / (double)height;

    for (int y = 0; y < height; y++)
    {
        double y0 = y * y_step - 1.0;
        for (int x = 0; x < width; x++)
        {
            double x0 = x * x_step - 2.5;
            double xi = 0.0;
            double yi = 0.0;
            int iteration = 0;

            while (iteration < max_iterations)
            {
                double xi_sq = xi * xi;
                double yi_sq = yi * yi;
                if ((xi_sq + yi_sq) > 4.0) break;

                double yi_new = 2.0 * xi * yi + y0;
                double xi_new = xi_sq - yi_sq + x0;

                xi = xi_new;
                yi = yi_new;

                iteration++;
            }
            mandelbrot_sum += (uint64_t)iteration;
        }
    }
    return mandelbrot_sum;
}

/**
 * @brief Mandelbrot with tiling (splitting the image vertically into chunks).
 * @param width Image width
 * @param height Image height
 * @param max_iterations Iteration limit
 * @param chunk_height Number of rows per chunk to process at once
 * @retval Total checksum across the full image
 */
uint64_t calculate_mandelbrot_tiled(int width, int height, int max_iterations, int chunk_height)
{
    uint64_t total_checksum = 0;

    const double x_step = 3.5 / (double)width;
    const double y_step = 2.0 / (double)height;

    for (int y_chunk_start = 0; y_chunk_start < height; y_chunk_start += chunk_height)
    {
        int y_chunk_end = y_chunk_start + chunk_height;
        if (y_chunk_end > height) {
            y_chunk_end = height; // handle last partial chunk
        }

        /* Process this chunk of rows */
        for (int y = y_chunk_start; y < y_chunk_end; y++)
        {
            double y0 = y * y_step - 1.0;
            for (int x = 0; x < width; x++)
            {
                double x0 = x * x_step - 2.5;
                double xi = 0.0;
                double yi = 0.0;
                int iteration = 0;

                while (iteration < max_iterations)
                {
                    double xi_sq = xi * xi;
                    double yi_sq = yi * yi;
                    if ((xi_sq + yi_sq) > 4.0) break;

                    double yi_new = 2.0 * xi * yi + y0;
                    double xi_new = xi_sq - yi_sq + x0;

                    xi = xi_new;
                    yi = yi_new;

                    iteration++;
                }
                total_checksum += (uint64_t)iteration;
            }
        }
    }

    return total_checksum;
}


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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //TODO: Visual indicator: Turn on LED0 to signal processing start

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	  //TODO: Benchmark and Profile Performance

	    /* Loop through test sizes */
	    for (int i = 0; i < NUM_TESTS; i++)
	    {
	      int sz = image_sizes[i];
	      uint32_t pixels = (uint32_t)sz * (uint32_t)sz;

	      /* --- FIXED-POINT TEST --- */
	      start_ms = HAL_GetTick();
	      start_cycles = dwt_get_cycle_count();

	      checksum_fixed[i] = calculate_mandelbrot_fixed_point_arithmetic(sz, sz, MAX_ITER);

	      end_cycles = dwt_get_cycle_count();
	      end_ms = HAL_GetTick();

	      elapsed_cycles = end_cycles - start_cycles;
	      elapsed_ms = end_ms - start_ms;

	      exec_time_fixed_ms[i] = elapsed_ms;
	      exec_cycles_fixed[i] = elapsed_cycles;

	      /* small delay between tests */
	      HAL_Delay(20);

	      /* --- DOUBLE-PRECISION TEST --- */
	      start_ms = HAL_GetTick();
	      start_cycles = dwt_get_cycle_count();

	      checksum_double[i] = calculate_mandelbrot_double(sz, sz, MAX_ITER);

	      end_cycles = dwt_get_cycle_count();
	      end_ms = HAL_GetTick();

	      elapsed_cycles = end_cycles - start_cycles;
	      elapsed_ms = end_ms - start_ms;

	      exec_time_double_ms[i] = elapsed_ms;
	      exec_cycles_double[i] = elapsed_cycles;

	      /* small delay */
	      HAL_Delay(50);
	    }


	  //TODO: Visual indicator: Turn on LED1 to signal processing start

	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

	  //TODO: Keep the LEDs ON for 2s

	    HAL_Delay(2000);

	  //TODO: Turn OFF LEDs

	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

	        /* --- Optional: insert a long wait or loop-break for step-by-step manual inspection --- */
	        HAL_Delay(2000);


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//TODO: Function signatures you defined previously , implement them here

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
