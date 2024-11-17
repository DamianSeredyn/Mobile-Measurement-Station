/*
 * uart.h
 *
 *  Created on: Nov 16, 2024
 *      Author: szymo
 */

#include "main.h"
#include "ringbuf_u8.h"



#ifndef INC_UART_H_
#define INC_UART_H_

#define BUFFER_SIZE	128

typedef struct
{
	uint8_t data[BUFFER_SIZE];
	uint32_t count;
}
buffer_uart;

#define USART_TX_Pin 2
#define USART_RX_Pin 3




void InitUART(void);

#endif /* INC_UART_H_ */
