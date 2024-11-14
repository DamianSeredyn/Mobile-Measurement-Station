/*
 * receiver.c
 *
 *  Created on: Nov 14, 2024
 *      Author: ciast
 */
#include "main.h"
#include "receiver.h"
#include "nRF24.h"

uint8_t robotComandReceve(uint8_t transmitter) {
	  uint8_t input = 1;
	  uint8_t size = 1;

	  LL_mDelay(50);
	  nRF24_SendPacket(&input, size, transmitter);

	  return input;
}
