/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define TIM2_CH1_OC_Pin LL_GPIO_PIN_0
#define TIM2_CH1_OC_Port GPIOA

#define PWM_LOGIC1_Pin LL_GPIO_PIN_1
#define PWM_LOGIC1_Port GPIOA

#define PWM_LOGIC2_Pin LL_GPIO_PIN_5
#define PWM_LOGIC2_Port GPIOA

#define RANGE 250
/* USER CODE END Private defines */
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM4_Init(void);
/* USER CODE BEGIN Prototypes */
void MX_TIM3_Init(void);
void PWM_GPIO_init(void);
void Generate_PWM(uint8_t fulfillment,TIM_TypeDef *TIMx);
void Robot_Drive_Forward(void);
void Robot_Stop(void);
void Robot_Turn_Left(void);
void Robot_Turn_Right(void);
void Robot_Emote(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

