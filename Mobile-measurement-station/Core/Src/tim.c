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
/* USER CODE BEGIN 0 */
  uint32_t pwm_duty;
/* USER CODE END 0 */

/* TIM2 init function */
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
void PWM_GPIO_init(void)
{
	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};


	  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

	  GPIO_InitStruct.Pin = PWM_LOGIC2_Pin;
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


/* USER CODE END 1 */
