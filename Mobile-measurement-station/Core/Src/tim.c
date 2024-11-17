/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"
#include <math.h>
#include "main.h"
/* USER CODE BEGIN 0 */
  uint32_t pwm_duty;
  volatile uint32_t counter = 0;
  extern uint32_t time;
/* USER CODE END 0 *//* TIM2 init function */


  /**
    * @brief TIM4 Initialization Function
    * @param None
    * @retval None
    */
  void MX_TIM5_Init(void)
 {

   /* USER CODE BEGIN TIM5_Init 0 */

   /* USER CODE END TIM5_Init 0 */

   LL_TIM_InitTypeDef TIM_InitStruct = {0};

   /* Peripheral clock enable */
   LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);

   /* USER CODE BEGIN TIM5_Init 1 */

   /* USER CODE END TIM5_Init 1 */
   TIM_InitStruct.Prescaler = 1;
   TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
   TIM_InitStruct.Autoreload = 4;
   TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
   LL_TIM_Init(TIM5, &TIM_InitStruct);
   LL_TIM_DisableARRPreload(TIM5);
   LL_TIM_SetClockSource(TIM5, LL_TIM_CLOCKSOURCE_INTERNAL);
   LL_TIM_SetTriggerOutput(TIM5, LL_TIM_TRGO_RESET);
   LL_TIM_DisableMasterSlaveMode(TIM5);

   NVIC_SetPriority(TIM5_IRQn, 0);
   NVIC_EnableIRQ(TIM5_IRQn);
   LL_TIM_EnableIT_UPDATE(TIM5);
   LL_TIM_EnableCounter(TIM5);

   /* USER CODE BEGIN TIM5_Init 2 */

   /* USER CODE END TIM5_Init 2 */

 }

  void TIM5_IRQHandler(void) {
      // Sprawdzenie flagi przerwania
      if (LL_TIM_IsActiveFlag_UPDATE(TIM5)) {
          LL_TIM_ClearFlag_UPDATE(TIM5); // Wyczyść flagę przerwania

          // Sprawdzenie stanu GPIO (PA0)
          if (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_8)) {
              counter = 0; // Wyzeruj licznik, jeśli stan wysoki
          } else {
              counter++;   // Inkrementuj licznik, jeśli stan niski
          }
      }
  }
  void MX_TIM4_Init(void)
  {

    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

    /* USER CODE BEGIN TIM4_Init 1 */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    /* USER CODE END TIM4_Init 1 */
    LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
     LL_TIM_SetCounterMode(TIM4, LL_TIM_COUNTERMODE_UP);
     LL_TIM_SetPrescaler(TIM4, 40-1);
     LL_TIM_SetAutoReload(TIM4, RANGE-1);
     LL_TIM_GenerateEvent_UPDATE(TIM4);
     LL_TIM_ClearFlag_UPDATE(TIM4);

     pwm_duty = 0;
     LL_TIM_OC_SetCompareCH2(TIM4, 1);

     LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
     LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);

     LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
     LL_TIM_EnableCounter(TIM4);

    /* USER CODE END TIM4_Init 2 */
    /**TIM4 GPIO Configuration
    PB7   ------> TIM4_CH2
    */

  }
  void MX_TIM2_Init(void)
  {

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    LL_APB1_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_GPIO_SetPinPull(TIM2_CH1_OC_Port, TIM2_CH1_OC_Pin, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinSpeed(TIM2_CH1_OC_Port, TIM2_CH1_OC_Pin, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetAFPin_0_7(TIM2_CH1_OC_Port, TIM2_CH1_OC_Pin, LL_GPIO_AF_1);
  LL_GPIO_SetPinMode(TIM2_CH1_OC_Port, TIM2_CH1_OC_Pin, LL_GPIO_MODE_ALTERNATE);

    LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetPrescaler(TIM2, 4000-1);
    LL_TIM_SetAutoReload(TIM2, RANGE-1);
    LL_TIM_GenerateEvent_UPDATE(TIM2);
    LL_TIM_ClearFlag_UPDATE(TIM2);

    pwm_duty = 0;
    LL_TIM_OC_SetCompareCH1(TIM2, pwm_duty);

    LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);

    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableCounter(TIM2);

  }

/* USER CODE BEGIN 1 */

void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
	 LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE END TIM2_Init 0 */

  /* Peripheral clock enable */
	 LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);
  LL_TIM_SetPrescaler(TIM3, 4000-1);
  LL_TIM_SetAutoReload(TIM3, RANGE-1);
  LL_TIM_GenerateEvent_UPDATE(TIM3);
  LL_TIM_ClearFlag_UPDATE(TIM3);

  pwm_duty = 0;
  LL_TIM_OC_SetCompareCH1(TIM3, pwm_duty);

  LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);

  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableCounter(TIM3);

}
void PWM_GPIO_init(void)
{
	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};


	  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

	  GPIO_InitStruct.Pin = PWM_LOGIC2_Pin|PWM_LOGIC1_Pin;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(PWM_LOGIC2_Port, &GPIO_InitStruct);

}

void Generate_PWM(uint8_t fulfillment,TIM_TypeDef *TIMx)
{
	pwm_duty = (int) round(RANGE*fulfillment*0.01);
	LL_TIM_OC_SetCompareCH1(TIMx, pwm_duty);
}

void Robot_Drive_Forward(void)
{
	Generate_PWM(80,TIM2);
	Generate_PWM(80,TIM3);

	LL_GPIO_ResetOutputPin(PWM_LOGIC1_Pin, PWM_LOGIC1_Port);
	LL_GPIO_ResetOutputPin(PWM_LOGIC2_Pin, PWM_LOGIC2_Port);
}

void Robot_Stop(void)
{
	Generate_PWM(0,TIM2);
	Generate_PWM(0,TIM3);

	LL_GPIO_ResetOutputPin(PWM_LOGIC1_Pin, PWM_LOGIC1_Port);
	LL_GPIO_ResetOutputPin(PWM_LOGIC2_Pin, PWM_LOGIC2_Port);
}

void Robot_Turn_Left(void)
{
	Generate_PWM(80,TIM2);
	Generate_PWM(0,TIM3);

	LL_GPIO_ResetOutputPin(PWM_LOGIC1_Pin, PWM_LOGIC1_Port);
	LL_GPIO_ResetOutputPin(PWM_LOGIC2_Pin, PWM_LOGIC2_Port);
}

void Robot_Turn_Right(void)
{
	Generate_PWM(0,TIM2);
	Generate_PWM(80,TIM3);

	LL_GPIO_ResetOutputPin(PWM_LOGIC1_Pin, PWM_LOGIC1_Port);
	LL_GPIO_ResetOutputPin(PWM_LOGIC2_Pin, PWM_LOGIC2_Port);
}

void Robot_Emote(void)
{
	Generate_PWM(80,TIM2);
	Generate_PWM(80,TIM3);

	LL_GPIO_ResetOutputPin(PWM_LOGIC1_Pin, PWM_LOGIC1_Port);
	LL_GPIO_SetOutputPin(PWM_LOGIC2_Pin, PWM_LOGIC2_Port);
}
/* USER CODE END 1 */
