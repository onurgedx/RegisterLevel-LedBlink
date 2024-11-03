/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// All needed configuration for registers
void Configurations(void);

// RCC| Register configurations
void RCC_Config(void);

//  GPIOD Configurations + Enables GPIOD
void GPIO_Config(void);

// Turns on Lights :)
void TurnonLights(void);

// Turns off Lights :(
void TurnoffLights(void);

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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	Configurations();
	int counter=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  for(counter =0;counter<268000;counter++);
	  TurnonLights();

	  for(counter=0; counter<268000;counter++);
	  TurnoffLights();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

/* USER CODE BEGIN 4 */


void Configurations(void)
{
	RCC_Config();
	SystemCoreClockUpdate();
	GPIO_Config();
}

void RCC_Config(void)
{
	RCC->CR =  0x00000083; // RCC Reset
	RCC->CR &= ~(1<<0); // HSI OFF

	RCC->CR |= 1<<16;  // HSE ON
	while(!(RCC->CR & (1<<17))); // Wait HSE active

	RCC->CR |= 1 << 19; // CSS enabled

	RCC->PLLCFGR = 0x24003010 ; // PLLCFGR Reset
	RCC->PLLCFGR |= (1<<22); // PLL SOURCE HSE
	RCC->PLLCFGR |= (8<<0); // PLL M = 4
	RCC->PLLCFGR |= (168<<6); // PLL N = 168
	RCC->PLLCFGR &= ~(1<<16); 	RCC->PLLCFGR &= ~(1<<17); // PLL P = 2
	RCC->CR |= 1<<24; //PLL ON

	while(!(RCC->CR & (1<<25))); // Wait For PLL ON

	RCC->CFGR = 0x00000000; // Reset
	RCC->CFGR &= ~(1<<0); RCC->CFGR |= (1<<1); // PLL Selected as system clock

	while(!(RCC->CFGR& (1<<1)));
}

void GPIO_Config(void)
{

	RCC->AHB1ENR =0x00100000;
	RCC->AHB1ENR |= 1<<3; // GPIOD Clock enable

	GPIOD->MODER |= 1<<24;	 GPIOD->MODER &= ~(1<<25); 	  // 01  12nd pin output mode

	GPIOD->MODER |= 1<<26;  GPIOD->MODER &= ~(1<<27); 	// 01  13rd pin output  mode

	GPIOD->MODER |= 1<<28;  GPIOD->MODER &= ~(1<<29); // 01  14th pin output  mode

	GPIOD->MODER |= 1<<30;	GPIOD->MODER &= ~(1<<31); 	 // 01  15rd pin output  mode

	//GPIOD->OTYPER

	GPIOD->OSPEEDR |= 0xFF000000;
	//GPIOD->OSPEEDR |= FF<<24; // same
}

void TurnoffLights(void)
{
	GPIOD->ODR |= 1<<12;
	GPIOD->ODR |= 1<<13;
	GPIOD->ODR |= 1<<14;
	GPIOD->ODR |= 1<<15;
}

void TurnonLights(void)
{
	GPIOD->ODR &= ~(1<<12);
	GPIOD->ODR &= ~(1<<13);
	GPIOD->ODR &= ~(1<<14);
	GPIOD->ODR &= ~(1<<15);
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
