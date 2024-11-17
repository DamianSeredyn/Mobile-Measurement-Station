/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#define ADC_RESOLUTION 4096.0  // 12-bit ADC
#define VREF 3300          // Napięcie referencyjne ADC
#define N 100                // Liczba próbek dla RMS

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* ADC1 init function */
void ADC_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_PLLSAI1);

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  /**ADC1 GPIO Configuration
  PC4   ------> ADC1_IN13
  PC5   ------> ADC1_IN14
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  LL_GPIO_EnablePinAnalogControl(GPIOC, LL_GPIO_PIN_4|LL_GPIO_PIN_5);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV1;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_NONE);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC1);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_13);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_13, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_13, LL_ADC_SINGLE_ENDED);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_14);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_14, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_14, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC1_Init 2 */
  NVIC_SetPriority(ADC1_2_IRQn, 1);
  NVIC_EnableIRQ(ADC1_2_IRQn);
  LL_ADC_Enable(ADC1);
  LL_ADC_EnableIT_EOC(ADC1);
  LL_ADC_EnableIT_EOS(ADC1);

  LL_ADC_REG_StartConversion(ADC1);
  /* USER CODE END ADC1_Init 2 */

}

/* USER CODE BEGIN 1 */
volatile uint8_t conv_cnt = 0;
volatile uint32_t *data_buf = 0;
volatile bool adc_conversion_done = 0;

void read_adc(uint32_t *data){
	adc_conversion_done = 0;
	data_buf = data;
	LL_ADC_REG_StartConversion(ADC1);

}


// Funkcja do obliczenia RMS
double calculate_rms(uint32_t channel) {
    uint32_t data[2];
    double sum_squared = 0.0;

    for (int i = 0; i < N; i++) {
        read_adc(data); // Odczytaj próbki do tablicy
        double voltage = (data[channel] / ADC_RESOLUTION) * VREF; // Przeskaluj ADC do napięcia
        sum_squared += voltage * voltage; // Sumuj kwadraty napięć
    }

    return sqrt(sum_squared / N); // Oblicz RMS
}

void write_adc_table(uint32_t *output){

	for (int i = 0; i < 50; i++){
	uint32_t data[2];
	read_adc(data);

	output[i] = data[1];
	Delay(10);
	}
}

uint32_t decybeloza (double calculated_rms){
	// potem wepnij to w jedną funkcję i nie zapomnij bo jesteś za spany
	float reference = 1.1666;
    double difference = fabs(calculated_rms - reference); // śmieszna funkcja do różnicy napięć
    return 20.0 * log10(difference / reference);

}

uint32_t adc_to_dB(void){
  	double rms_voltage = calculate_rms(1);
  	uint32_t dB_value = decybeloza (rms_voltage);

  	return dB_value;
}


void adc_conversion_complete_callback(void)
{
	  data_buf[conv_cnt] = CONVERT_ADC_TO_MV(LL_ADC_REG_ReadConversionData12(ADC1));

	  conv_cnt++;
}

void adc_sequence_complete_callback(void)
{
	conv_cnt = 0;
	adc_conversion_done = 1;
}

void ADC1_2_IRQHandler(void)
{
	if(LL_ADC_IsActiveFlag_EOC(ADC1) != 0)
		{
			adc_conversion_complete_callback();

		}

		if(LL_ADC_IsActiveFlag_EOS(ADC1) != 0)
		{
			adc_sequence_complete_callback();

			LL_ADC_ClearFlag_EOS(ADC1);
		}
}

/* USER CODE END 1 */
