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
#include <stdint.h>
#include <stdlib.h>
#include <stdint.h>
#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_rcc.h"

/* Private define ------------------------------------------------------------*/
#define MAX_ITER 100

/* Private variables ---------------------------------------------------------*/
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint32_t execution_time[5] = {0};
volatile uint64_t checksum[5] = {0};
volatile uint32_t CyclesElapsed[5] = {0};
double time_secs[5] = {0};
double pixPerSec[5] = {0};

// Width and height arrays
uint64_t width[5] = {128, 160, 192, 224, 256};
uint64_t height[5] = {128, 160, 192, 224, 256};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    /* --- Enable TIM2 as free-running cycle counter --- */
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->PSC = 0;       // Prescaler = 0 -> 48 MHz tick
    TIM2->CNT = 0;       // Clear counter
    TIM2->CR1 |= TIM_CR1_CEN; // Start timer

    const int num_sizes = sizeof(width)/sizeof(width[0]);

    while(1)
    {
        // Turn on LED0 to indicate processing start
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

        for (int i = 0; i < num_sizes; i++)
        {
            uint32_t start_cycles = TIM2->CNT;  // Start cycle count
            start_time = HAL_GetTick();          // Start time in ms

            checksum[i] = calculate_mandelbrot_double(width[i], height[i], MAX_ITER);
            // Or use fixed-point version:
            // checksum[i] = calculate_mandelbrot_fixed_point_arithmetic(width[i], height[i], MAX_ITER);

            end_time = HAL_GetTick();            // End time in ms
            execution_time[i] = end_time - start_time;

            uint32_t stop_cycles = TIM2->CNT;    // Stop cycle count
            CyclesElapsed[i] = stop_cycles - start_cycles;

            // Convert to seconds
            time_secs[i] = (double)execution_time[i] / 1000.0;

            // Compute pixels/sec with 4 decimal precision
            pixPerSec[i] = (float)(width[i]*height[i]) / ((float)execution_time[i]/1000.0f);
        }

        // Keep LEDs on for 2s
        HAL_Delay(2000);

        // Turn off LED0
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    }
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) Error_Handler();
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                            |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                        |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* Mandelbrot Functions */
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations){
    uint64_t mandelbrot_sum = 0;
    int32_t sFactor = 100000; // scaling factor
    int32_t x0, y0, xi, yi, temp;
    int iteration;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Map pixel to fixed-point coordinates
            x0 = (x * sFactor / width) * 35 / 10 - 25 * sFactor / 10;
            y0 = (y * sFactor / height) * 2 - sFactor;
            xi = yi = 0;
            iteration = 0;

            while ((iteration < max_iterations) && (xi*xi + yi*yi <= 4*sFactor*sFactor)) {
                temp = (xi*xi - yi*yi)/sFactor;
                yi = (2*xi*yi)/sFactor + y0;
                xi = temp + x0;
                iteration++;
            }

            mandelbrot_sum += iteration;
        }
    }

    return mandelbrot_sum;
}

uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations)
{
    uint64_t mandelbrot_sum = 0;
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            double x0 = ((double)x / width)*3.5 - 2.5;
            double y0 = ((double)y / height)*2.0 - 1.0;
            double xi = 0, yi = 0;
            int iter = 0;

            while (iter < max_iterations)
            {
                double xi2 = xi*xi;
                double yi2 = yi*yi;
                if (xi2 + yi2 > 4.0) break;

                double temp = xi2 - yi2;
                yi = 2.0*xi*yi + y0;
                xi = temp + x0;
                iter++;
            }

            mandelbrot_sum += iter;
        }
    }
    return mandelbrot_sum;
}

/* Error Handler */
void Error_Handler(void)
{
    __disable_irq();
    while(1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
