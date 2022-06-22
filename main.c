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
void init_clock(void);
void init_port(void);
void init_timer(void);
void duty_cycle(long);
void delay1(void);

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
  /* USER CODE BEGIN 2 */
    init_clock();
    init_port();
    init_timer();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  for(int i=20;i<125;i++){
		  duty_cycle(i);
		  delay1();
	  }

	  for(int j=124;j>=20;j--){
	 		  duty_cycle(j);
	 		  delay1();
	 }

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void init_clock(){
	  volatile long *RCC_AHB1;
	  RCC_AHB1 = (long*)0x40023830;
	  *RCC_AHB1 = 0x00000008UL; //Enable GPIOD CLOCK

	  volatile long *RCC_APB1;
	  RCC_APB1 = (long*)0x40023840;
	  *RCC_APB1 = 0x00000004UL; // Enable TIMER4 CLOCK
}

void init_port(){
	volatile long *GPIOD_MODE;
	GPIOD_MODE = (long*)0x40020C00;
	*GPIOD_MODE = 0x80000000UL; //pin15 as alternate function

	volatile long *GPIOD_AFN;
	GPIOD_AFN = (long*)0x40020C24;
	*GPIOD_AFN = 0x20000000UL; //Pin 15 as TIMER4 and channel
}

void init_timer(){
	   volatile long *TIM4_CCER;
	   TIM4_CCER = (long*)0x40000820;
	   *TIM4_CCER = 0x00001000UL; //capture and compare 4 polarity

	   volatile long *TIM4_CR1;
	   TIM4_CR1 = (long*)0x40000800;
	   *TIM4_CR1 = 0x00000081UL; //counter_enable and auto pre-load enable

	   volatile long *TIM4_CCMR2;
	   TIM4_CCMR2 = (long*)0x4000081C;
	   *TIM4_CCMR2 = 0x00006800UL;//channel is active as long as TIMx_CNT<TIMx_CCR4 and Output compare 4 preload enable

	   volatile long *TIM4_PSC;
	   TIM4_PSC = (long*)0x40000828;
	   *TIM4_PSC = 318UL; //pre-scaler

	   volatile long *TIM4_ARR;
	   TIM4_ARR = (long*)0x4000082C;
	   *TIM4_ARR = 1000UL; //auto-reload

	   volatile long *TIM4_EGR;
	   TIM4_EGR = (long*)0x40000814;
	   *TIM4_EGR = 0x00000001UL; //update and Re-initialze counter
}

void duty_cycle(long data){
	volatile long *TIM4_CCR4;
    TIM4_CCR4 = (long*)0x40000840;
    *TIM4_CCR4 = data;
}

void delay1(){
	volatile long i;
	for(i=0;i<50000;i++);
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
