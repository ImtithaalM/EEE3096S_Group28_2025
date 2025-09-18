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
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"

/* Private define ------------------------------------------------------------*/
#define MAX_ITER 100

/* Private variables ---------------------------------------------------------*/
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint32_t execution_time[5] = {0};
volatile uint64_t checksum[5] = {0};
uint32_t cyclesElapsed[5] = {0};
double pixPerSec[5] = {0};
uint64_t width[5]  = {128,160,192,224,256};
uint64_t height[5] = {128,160,192,224,256};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);
void enableCycleCounter(void);
uint32_t getCycleCount(void);

/* Private user code ---------------------------------------------------------*/
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations){
    uint64_t mandelbrot_sum = 0;
    int16_t sFactor = 10000; // scaling factor
    int16_t x0, y0, xi, yi, temp;
    int iteration;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            x0 = (x * sFactor / width) * 3.5 - 2.5 * sFactor;
            y0 = (y * sFactor / height) * 2 - 1 * sFactor;
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

uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations){
    uint64_t mandelbrot_sum = 0;
    double x0, y0, xi, yi, iteration, temp;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            x0 = ((double)x / (double)width) * 3.5 - 2.5;
            y0 = ((double)y / (double)height) * 2 - 1;
            xi = yi = iteration = 0;

            while ((iteration < max_iterations) && (xi*xi + yi*yi <= 4)) {
                temp = xi*xi - yi*yi;
                yi = 2*xi*yi + y0;
                xi = temp + x0;
                iteration++;
            }
            mandelbrot_sum += iteration;
        }
    }
    return mandelbrot_sum;
}

/* Main function -------------------------------------------------------------*/
int main(void){
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    for (int i = 0; i < 5; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // LED start

        // Start timing
        start_time = HAL_GetTick();
        enableCycleCounter();
        uint32_t startCycles = getCycleCount();

        // Call Mandelbrot (fixed-point)
        checksum[i] = calculate_mandelbrot_fixed_point_arithmetic(width[i], height[i], MAX_ITER);
        // To use double: checksum[i] = calculate_mandelbrot_double(width[i], height[i], MAX_ITER);

        // End timing
        end_time = HAL_GetTick();
        uint32_t endCycles = getCycleCount();

        execution_time[i] = end_time - start_time;
        cyclesElapsed[i] = endCycles - startCycles;
        pixPerSec[i] = (float)(width[i]*height[i]) / ((float)execution_time[i]/1000.0);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // LED done
        HAL_Delay(1000); // hold LEDs
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    }

    while(1) {
        // nothing to do, execution already measured
    }
}

/* System Clock Configuration */
void SystemClock_Config(void){
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 15;
    RCC_OscInitStruct.PLL.PLLN = 144;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) Error_Handler();
}

/* GPIO Initialization */
static void MX_GPIO_Init(void){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                            |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                        |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* DWT Cycle Counter functions */
void enableCycleCounter(void){
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t getCycleCount(void){
    return DWT->CYCCNT;
}

/* Error Handler */
void Error_Handler(void){
    __disable_irq();
    while(1){}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){}
#endif
