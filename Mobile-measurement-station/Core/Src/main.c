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
#include "tim.h"
#include "gpio.h"
#include "spi.h"
#include "nRF24.h"
#include "radioControl.h"
#include "bme280.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define nRF24_TRANSMITER 0
#define nRF24_RECEIVER 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
__IO uint32_t Tick;
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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SysTick_Config(4000);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */
  MX_TIM3_Init();
  PWM_GPIO_init();
  MX_I2C1_Init();

  //Init_OLED();
  init_ControlerButtons();
  BME280_init();

  nRF24_InitGPIO();
  nRF24_Init(nRF24_RECEIVER);
    nRF24_SetRXAddress(0, (uint8_t *)"Odb",nRF24_RECEIVER);
    nRF24_SetTXAddress((uint8_t *)"Nad",nRF24_RECEIVER);
    nRF24_RX_Mode(nRF24_RECEIVER);
  /*  uint8_t stat1 = nRF24_ReadStatus(nRF24_TRANSMITER);*/
    uint8_t stat2 = nRF24_ReadStatus(nRF24_RECEIVER);
  	  uint8_t output = 0;
  	  uint8_t tablica[100];
  	  for (int j = 0; j < 100; j++){
  		  tablica[j] = 0;
  	  }
  	  uint8_t size = 1;

  	  uint8_t i = 0;
  	  uint8_t k = 0;

/*  nRF24_Init(nRF24_RECEIVER);
  nRF24_SetRXAddress(0, (uint8_t *)"Odb",nRF24_RECEIVER);
  nRF24_SetTXAddress((uint8_t *)"Nad",nRF24_RECEIVER);
  nRF24_RX_Mode(nRF24_RECEIVER);*/

  /* USER CODE END 2 */
  	  //Oled_test();
  	  float test2 = BME280_read_temp();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(nRF24_RXAvailible(nRF24_RECEIVER))
	  	  	  	  {
		  i++;
		nRF24_ReadRXPaylaod(&tablica[i], &size, nRF24_RECEIVER);
	  }
	  if ( i != 0 && i != k){
		  if (tablica[k] == 1){
			  Robot_Drive_Forward();
		  }
		  if (tablica[k] == 2){
			  Robot_Turn_Left();
		  }
		  if (tablica[k] == 3){
			  Robot_Turn_Right();
		  }
		  if (tablica[k] == 4){
			  Robot_Stop();
		  }
		  LL_mDelay(1);
		  k++;
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
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  while (LL_PWR_IsActiveFlag_VOS() != 0)
  {
  }
  LL_RCC_MSI_Enable();

   /* Wait till MSI is ready */
  while(LL_RCC_MSI_IsReady() != 1)
  {

  }
  LL_RCC_MSI_EnableRangeSelection();
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6);
  LL_RCC_MSI_SetCalibTrimming(0);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_Init1msTick(4000000);

  LL_SetSystemCoreClock(4000000);
}

/* USER CODE BEGIN 4 */
void Delay(uint32_t Delay_ms)
{

	    uint32_t StartTime = Tick;
	    while(Tick < (StartTime + Delay_ms))
	    {
	        // Just wait
	    }
}

void SysTick_Handler(void)
{
	    Tick++; // Increase system timer
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
