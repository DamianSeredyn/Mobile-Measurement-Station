/*
 * uart.c
 *
 *  Created on: Nov 16, 2024
 *      Author: szymo
 */
#include "uart.h"
#include <stdio.h>

#define BUFFER_SIZE 128

ringbuf_u8_s ringbuffer;
uint8_t buffer[BUFFER_SIZE];
uint8_t command_buffer[BUFFER_SIZE];
uint32_t command_length = 0;

void InitRingbuffer(void){
	ringbuf_u8_init(&ringbuffer, buffer, BUFFER_SIZE);
}

void TransmitData(void)
{
	uint8_t tx_data;
	if(ringbuf_u8_dequeue(&ringbuffer, &tx_data) != 1){
		LL_USART_DisableIT_TXE(USART2);
	}else{
		LL_USART_TransmitData8(USART2, tx_data);
	}
}

void ReciveData(void) {
    uint8_t rx_data = LL_USART_ReceiveData8(USART2);

    if (rx_data == '\r' || rx_data == '\n')
    {
        command_buffer[command_length] = '\0';

        ProcessCommand(command_buffer);

        command_length = 0;
    }
    else
    {
        if (command_length < BUFFER_SIZE - 1)
        {
            command_buffer[command_length++] = rx_data;
        }
    }
}

void ProcessCommand(uint8_t* cmd) {
    char response[BUFFER_SIZE];

    	snprintf(response, BUFFER_SIZE, "RECIEVED: %s\n\r", cmd);

    ringbuf_u8_queue_array(&ringbuffer, (uint8_t*)response, strlen(response));

    LL_USART_EnableIT_TXE(USART2);

}


void USART2_IRQHandler(void)
{
	if(LL_USART_IsEnabledIT_TXE(USART2) && LL_USART_IsActiveFlag_TXE(USART2))
	{
		TransmitData();
	}
	if(LL_USART_IsEnabledIT_RXNE(USART2) && LL_USART_IsActiveFlag_RXNE(USART2))
	{
		ReciveData();
	}
	if(LL_USART_IsEnabledIT_IDLE(USART2) && LL_USART_IsActiveFlag_IDLE(USART2))
	{
		LL_USART_ClearFlag_IDLE(USART2);

	}
}

void InitUART(void) {

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_7); // AF7
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_NO);

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF_7); // AF7
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, LL_GPIO_PULL_NO);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

    LL_USART_ConfigCharacter(USART2, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
    LL_USART_SetBaudRate(USART2, SystemCoreClock, LL_USART_OVERSAMPLING_16, 115200);
    LL_USART_ConfigAsyncMode(USART2);

    NVIC_SetPriority(USART2_IRQn, 0);
    NVIC_EnableIRQ(USART2_IRQn);

    LL_USART_EnableDirectionTx(USART2);
    LL_USART_EnableDirectionRx(USART2);

    LL_USART_Enable(USART2);

    LL_USART_ClearFlag_IDLE(USART2);
    LL_USART_EnableIT_RXNE(USART2);
    LL_USART_EnableIT_TXE(USART2);
    LL_USART_EnableIT_IDLE(USART2);

}


void SendString(const uint8_t *str)
{
	const uint8_t *recieved = str;

	ringbuf_u8_queue_array(&ringbuffer, recieved, strlen((const char *)recieved));

    LL_USART_EnableIT_TXE(USART2);
}
