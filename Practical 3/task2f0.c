/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file : main.c
  * @brief : Main program body
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
#include "stm32f0xx.h"
#include "lcd_stm32f0.c"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_rcc.h"

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

/* USER CODE BEGIN PV */
//TODO: Define and initialise the global variables required

#define MAX_ITER 100

// image dimensions
uint64_t height[5] = {128, 160, 192, 224, 256};
uint64_t width[5] = {128, 160, 192, 224, 256};

//global variables
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;

volatile uint32_t execution_time[5];
volatile uint64_t checksum[5];




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);

void ourname(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief The application entry point.
  * @retval int
  */

void ourname(void) {

init_LCD();
lcd_command(CLEAR);
lcd_putstring("Group 28") ;
lcd_command(LINE_TWO);
lcd_putstring("Practical 1b");

}

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

  ourname(); // display on LCD

  /* USER CODE BEGIN 2 */
  //TODO: Turn on LED 0 to signify the start of the operation

   // Turn on LED 0 to signify start
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);


	  //TODO: Record the start time
	  start_time = HAL_GetTick();


	  //TODO: Call the Mandelbrot Function and store the output in the checksum variable defined initially
	  for (int j = 0; j < 5; j++)
	  {
		  checksum[j] = calculate_mandelbrot_fixed_point_arithmetic(width[j], height[j], MAX_ITER); // call arithmetic and call doubles by changing the function

	  //TODO: Record the end time
		  end_time = HAL_GetTick();

	  //TODO: Calculate the execution time
		  uint32_t elapsed = end_time - start_time;
		  execution_time[j] = elapsed;

	  //TODO: Turn on LED 1 to signify the end of the operation
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

	  //TODO: Hold the LEDs on for a 1s delay
	  	  HAL_Delay(1000);

	  //TODO: Turn off the LEDs
	  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	  }

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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//TODO: Mandelbroat using variable type integers and fixed point arithmetic

uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations) {

    //define local variables for fixed point arithmetic
    uint64_t mandelbrot_checksum = 0;
    int64_t scale_factor = 1000000;
    int64_t x0;
    int64_t y0;
    int64_t xi;
    int64_t yi;
    int64_t iteration;
    int64_t xi_temp;

    for (int y = 0; y <= height - 1; y++) {
        for (int x = 0; x <= width - 1; x++) {
            x0 = (x * scale_factor / width) * 35 / 10 - 25 * scale_factor / 10;
            y0 = (y * scale_factor / height) * 2 - scale_factor;

            xi = 0;
            yi = 0;
            iteration = 0;

            while ((iteration < max_iterations) && (xi * xi + yi * yi <= 4 * scale_factor * scale_factor)) {
                xi_temp = (xi * xi - yi * yi) / scale_factor;
                yi = (2 * xi * yi) / scale_factor + y0;
                xi = xi_temp + x0;
                ++iteration;
            }

            mandelbrot_checksum += iteration;
        }
    }
    return mandelbrot_checksum;
}

uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations) {


    uint64_t mandelbrot_checksum = 0;

    //define variables as double
    double x0;
    double y0;
    double xi;
    double yi;
    double iteration;
    double xi_temp;

    for (int y = 0; y <= height - 1; y++) {
        for (int x = 0; x <= width - 1; x++) {
            x0 = ((double)x / (double)width) * 3.5 - 2.5;
            y0 = ((double)y / (double)height) * 2.0 - 1.0;

            xi = 0;
            yi = 0;
            iteration = 0;

            while ((iteration < max_iterations) && (xi * xi + yi * yi <= 4.0)) {
                xi_temp = xi * xi - yi * yi;
                yi = 2.0 * xi * yi + y0;
                xi = xi_temp + x0;
                ++iteration;
            }

            mandelbrot_checksum += iteration;
        }
    }
    return mandelbrot_checksum;
}
/* USER CODE END 4 */

/**
  * @brief This function is executed in case of error occurrence.
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
  * @brief Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
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
