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

void ReciveData(void);
void TransmitData(void);
void InitUART(void);
void InitRingbuffer(void);
void ProcessCommand(uint8_t* cmd);


#endif /* INC_UART_H_ */
