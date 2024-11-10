/*
 * radioControl.c
 *
 *  Created on: Nov 10, 2024
 *      Author: szymo
 */
#include "main.h"
#include "radioControl.h"
#include "nRF24.h"

void init_ControlerButtons(void){
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

	LL_GPIO_SetPinPull(GPIOA, FORWARD_BUTTON_Pin, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinSpeed(GPIOA, FORWARD_BUTTON_Pin, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetPinMode(GPIOA, FORWARD_BUTTON_Pin, LL_GPIO_MODE_INPUT);

	LL_GPIO_SetPinPull(GPIOA, LEFT_BUTTON_Pin, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinSpeed(GPIOA, LEFT_BUTTON_Pin, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetPinMode(GPIOA, LEFT_BUTTON_Pin, LL_GPIO_MODE_INPUT);

	LL_GPIO_SetPinPull(GPIOA, RIGHT_BUTTON_Pin, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinSpeed(GPIOA, RIGHT_BUTTON_Pin, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetPinMode(GPIOA, RIGHT_BUTTON_Pin, LL_GPIO_MODE_INPUT);
}

uint8_t robotComandsSend_test(uint8_t transmitter){
	if (!LL_GPIO_IsInputPinSet(GPIOA, FORWARD_BUTTON_Pin))
	{
		  uint8_t input = 1;
		  uint8_t output = 0;
		  uint8_t size = 1;

		  LL_mDelay(50);
		  nRF24_SendPacket(&input, size, transmitter);
		  LL_mDelay(50);

		  if(nRF24_RXAvailible(!transmitter))
		  {
		  		  nRF24_ReadRXPaylaod(&output, &size, !transmitter);
		  		  output = output + 1;
		  }
		  return output;
	}
}


void robotComandsSend(uint8_t transmitter)
{
	uint8_t input = 0;
	uint8_t size = 1;

	if (!LL_GPIO_IsInputPinSet(GPIOA, FORWARD_BUTTON_Pin))
	{
		input = 1;  // robot forward
	}
	else if (!LL_GPIO_IsInputPinSet(GPIOA, LEFT_BUTTON_Pin))
	{
		input = 2;  // robot turn left
	}
	else if (!LL_GPIO_IsInputPinSet(GPIOA, RIGHT_BUTTON_Pin))
	{
		input = 3;  // robot turn right
	}

	if (input != 0)
	{
		LL_mDelay(50);
		nRF24_SendPacket(&input, size, transmitter);
		LL_mDelay(50);
	}

}

