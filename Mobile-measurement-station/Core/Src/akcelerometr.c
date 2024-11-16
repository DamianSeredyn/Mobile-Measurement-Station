/*
 * akcelerometr.c
 *
 *  Created on: Nov 16, 2024
 *      Author: ciast
 */
#include "akcelerometr.h"
#include "main.h"
#include "i2c.h"

void init_Akcelerometr(void){

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);


	LL_GPIO_SetPinPull(GPIO_AKCELEROMETR_PRZERWANIE, GPIO_AKCELEROMETR_PRZERWANIE_PIN, LL_GPIO_PULL_DOWN);
	LL_GPIO_SetPinSpeed(GPIO_AKCELEROMETR_PRZERWANIE, GPIO_AKCELEROMETR_PRZERWANIE_PIN, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetPinMode(GPIO_AKCELEROMETR_PRZERWANIE, GPIO_AKCELEROMETR_PRZERWANIE_PIN, LL_GPIO_MODE_INPUT);

	LL_EXTI_SetEXTISource(GPIO_AKCELEROMETR_PRZERWANIE, GPIO_AKCELEROMETR_PRZERWANIE_PIN);

	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_6);

	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_6);

	NVIC_SetPriority(EXTI9_5_IRQn, 0);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void EXTI4_15_IRQHandler(void)
{
	if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_6) != RESET)
	{
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_6);
	}
}

void check_accelerometr_alive(void){
	uint8_t output = 0;
	I2C1_reg_read_it(0x3A, 0x0F, &output, 1);
	if (output == 0x41){
		return 0;
	}else{
		return 1;
	}
}




