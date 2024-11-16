/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define ADC_VREF_MV		3300
#define ADC_MAX_VALUE	4096
#define CONVERT_ADC_TO_MV(x) ((ADC_VREF_MV * x) / (ADC_MAX_VALUE - 1))
/* USER CODE END Private defines */

void ADC_Init(void);

/* USER CODE BEGIN Prototypes */

/*
 * Function used to read data from ADC channels
 * returns voltage in mV
 * data is returned in order: .... rank 3 -> rank 2 -> rank 1
 *
 * example:
 * #define CHANNELS 2
 * uint32_t voltage[CHANNELS]
 * read_adc(voltage};
 */
void read_adc(uint32_t *data);
void adc_conversion_complete_callback(void);
void adc_sequence_complete_callback(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

