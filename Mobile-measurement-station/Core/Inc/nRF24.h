/*
 * nRF24.h
 *
 *  Created on: Aug 6, 2024
 *      Author: Damian
 */
#include "main.h"
#ifndef INC_NRF24_H_
#define INC_NRF24_H_

//
//	Configuration
//
#define NRF24_DYNAMIC_PAYLOAD	0
#define NRF24_INTERRUPT_MODE	0

//
// Enums
//
typedef enum
{
	NRF24_RECEIVED_PACKET,		// 0
	NRF24_NO_RECEIVED_PACKET,	// 1
} nRF24_RX_Status;

typedef enum
{
	NRF24_TRANSMITTED_PACKET,		// 0
	NRF24_NO_TRANSMITTED_PACKET,	// 1
} nRF24_TX_Status;


#define NRF24_CONFIG		0x00
#define NRF24_EN_AA		0x01
#define NRF24_EN_RXADDR	0x02
#define NRF24_SETUP_AW	0x03
#define NRF24_SETUP_RETR	0x04
#define NRF24_RF_CH		0x05
#define NRF24_RF_SETUP	0x06
#define NRF24_STATUS		0x07
#define NRF24_OBSERVE_TX	0x08
#define NRF24_CD			0x09
#define NRF24_RX_ADDR_P0	0x0A
#define NRF24_RX_ADDR_P1	0x0B
#define NRF24_RX_ADDR_P2	0x0C
#define NRF24_RX_ADDR_P3	0x0D
#define NRF24_RX_ADDR_P4	0x0E
#define NRF24_RX_ADDR_P5	0x0F
#define NRF24_TX_ADDR		0x10
#define NRF24_RX_PW_P0	0x11
#define NRF24_RX_PW_P1	0x12
#define NRF24_RX_PW_P2	0x13
#define NRF24_RX_PW_P3	0x14
#define NRF24_RX_PW_P4	0x15
#define NRF24_RX_PW_P5	0x16
#define NRF24_FIFO_STATUS	0x17
#define NRF24_DYNPD		0x1C
#define NRF24_FEATURE		0x1D


#define NRF24_CMD_R_REGISTER			0x00
#define NRF24_CMD_W_REGISTER			0x20
#define NRF24_CMD_R_RX_PAYLOAD		0x61
#define NRF24_CMD_W_TX_PAYLOAD		0xA0
#define NRF24_CMD_FLUSH_TX			0xE1
#define NRF24_CMD_FLUSH_RX			0xE2
#define NRF24_CMD_REUSE_TX_PL			0xE3
#define NRF24_CMD_ACTIVATE			0x50
#define NRF24_CMD_R_RX_PL_WID			0x60
#define NRF24_CMD_W_ACK_PAYLOAD		0xA8
#define NRF24_CMD_W_TX_PAYLOAD_NOACK	0xB0
#define NRF24_CMD_NOP					0xFF


#define NRF24_MASK_RX_DR  6
#define NRF24_MASK_TX_DS  5
#define NRF24_MASK_MAX_RT 4
#define NRF24_EN_CRC      3
#define NRF24_CRCO        2
#define NRF24_PWR_UP      1
#define NRF24_PRIM_RX     0
#define NRF24_ENAA_P5     5
#define NRF24_ENAA_P4     4
#define NRF24_ENAA_P3     3
#define NRF24_ENAA_P2     2
#define NRF24_ENAA_P1     1
#define NRF24_ENAA_P0     0
#define NRF24_ERX_P5      5
#define NRF24_ERX_P4      4
#define NRF24_ERX_P3      3
#define NRF24_ERX_P2      2
#define NRF24_ERX_P1      1
#define NRF24_ERX_P0      0
#define NRF24_AW          0
#define NRF24_ARD         4
#define NRF24_ARC         0
#define NRF24_PLL_LOCK    4
#define NRF24_RF_DR_HIGH  3
#define NRF24_RF_DR_LOW	  5
#define NRF24_RF_PWR      1
#define NRF24_LNA_HCURR   0
#define NRF24_RX_DR       6
#define NRF24_TX_DS       5
#define NRF24_MAX_RT      4
#define NRF24_RX_P_NO     1
#define NRF24_TX_FULL     0
#define NRF24_PLOS_CNT    4
#define NRF24_ARC_CNT     0
#define NRF24_TX_REUSE    6
#define NRF24_FIFO_FULL   5
#define NRF24_TX_EMPTY    4
#define NRF24_RX_FULL     1
#define NRF24_RX_EMPTY    0
#define NRF24_RPD         0x09
#define NRF24_EN_DPL      2

#define NRF24_CRC_WIDTH_1B   0
#define NRF24_CRC_WIDTH_2B  1

#define NRF24_PAYLOAD_SIZE 1
#define NRF24_ADDR_SIZE	3

#define NRF24_RF_DR_250KBPS   2
#define NRF24_RF_DR_1MBPS  0
#define NRF24_RF_DR_2MBPS  1

#define NRF24_PA_PWR_M18dBM  0
#define NRF24_PA_PWR_M12dBM 1
#define NRF24_PA_PWR_M6dBM  2
#define NRF24_PA_PWR_0dBM 3

void nRF24_Init(uint8_t device);
void nRF24_InitGPIO(void);
void nRF24_Delay(uint8_t time);
void nRF24_SetPALevel(uint8_t lev,uint8_t device);
void nRF24_SetDataRate(uint8_t dr,uint8_t device);
void nRF24_EnableCRC(uint8_t onoff,uint8_t device);
uint8_t nRF24_ReadConfig(uint8_t device);
void nRF24_WriteConfig(uint8_t conf,uint8_t device);
void nRF24_SetCRCLength(uint8_t crcl,uint8_t device);
void nRF24_SetRetries(uint8_t ard, uint8_t arc,uint8_t device);
void nRF24_SetPayloadSize(uint8_t pipe, uint8_t size,uint8_t device);
void nRF24_SetRFChannel(uint8_t channel,uint8_t device);
void nRF24_EnablePipe(uint8_t pipe, uint8_t onoff,uint8_t device);
void nRF24_AutoACK(uint8_t pipe, uint8_t onoff,uint8_t device);
void nRF24_SetAddressWidth(uint8_t size,uint8_t device);
void nRF24_EnableRXDataReadyIRQ(uint8_t onoff,uint8_t device);
void nRF24_EnableTXDataSentIRQ(uint8_t onoff,uint8_t device);
void nRF24_EnableMaxRetransmitIRQ(uint8_t onoff,uint8_t device);
void nRF24_ClearInterrupts(uint8_t device);
uint8_t nRF24_ReadStatus(uint8_t device);
void nRF24_WriteStatus(uint8_t st,uint8_t device);
void nRF24_FlushRX(uint8_t device);
void nRF24_FlushTX(uint8_t device);
nRF24_TX_Status nRF24_SendPacket(uint8_t* Data, uint8_t Size,uint8_t device);
void nRF24_SetTXAddress(uint8_t* address,uint8_t device);
void nRF24_TX_Mode(uint8_t device);
void nRF24_WriteTXPayload(uint8_t * data, uint8_t size,uint8_t device);
void nRF24_WaitTX(uint8_t device);
void nRF24_SetRXAddress(uint8_t pipe, uint8_t* address,uint8_t device);
void nRF24_RX_Mode(uint8_t device);
uint8_t nRF24_ReadFifoStatus(uint8_t device);
void nRF24_WriteFifoStatus(uint8_t st,uint8_t device);
uint8_t nRF24_IsBitSetInFifoStatus(uint8_t Bit,uint8_t device);
uint8_t nRF24_IsTxReuse(uint8_t device);
uint8_t nRF24_IsTxFull(uint8_t device);
uint8_t nRF24_IsTxEmpty(uint8_t device);
uint8_t nRF24_IsRxFull(uint8_t device);
uint8_t nRF24_IsRxEmpty(uint8_t device);
uint8_t nRF24_GetDynamicPayloadSize(uint8_t device);
nRF24_RX_Status nRF24_ReceivePacket(uint8_t* Data, uint8_t *Size,uint8_t device);
uint8_t nRF24_RXAvailible(uint8_t device);
void nRF24_ReadRXPaylaod(uint8_t *data, uint8_t *size,uint8_t device);
uint8_t nRF24_Test(uint8_t transmitter);

#endif /* INC_NRF24_H_ */
