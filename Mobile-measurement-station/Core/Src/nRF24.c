/*
 * nRF24.c
 *
 *  Created on: Nov 10, 2024
 *      Author: szymo
 */
/*
 * nRF24.c
 *
 *  Created on: Aug 6, 2024
 *      Author: Damian
 */

#include "nRF24.h"
#include "spi.h"



#define NRF24_CE_HIGH		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1)
#define NRF24_CE_LOW		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1)

#define NRF24_CE2_HIGH		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2)
#define NRF24_CE2_LOW		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2)

static uint8_t addr_p0_backup[NRF24_ADDR_SIZE];
static uint8_t nrf24_rx_flag;

void nRF24_Delay(uint8_t time)
{
	LL_mDelay(time);
}
static uint8_t nRF24_ReadRegister(uint8_t reg,uint8_t device)
{
	uint8_t result;

	reg = NRF24_CMD_R_REGISTER | reg;

	if(device == 0)
		spi_cs_set_low();
	else if(device == 1)
		spi_cs2_set_low();

	spi_write_data(&reg, 1);
	spi_read_data(&result, 1);
	if(device == 0)
		spi_cs_set_high();
	else if(device == 1)
		spi_cs2_set_high();

	return result;
}

static void nRF24_ReadRegisters(uint8_t reg, uint8_t* ret, uint8_t len,uint8_t device)
{

	if(device == 0)
		spi_cs_set_low();
	else if(device == 1)
		spi_cs2_set_low();
	reg = NRF24_CMD_R_REGISTER | reg;
	spi_write_data(&reg, 1);
	spi_read_data(ret, len);
	if(device == 0)
		spi_cs_set_high();
	else if(device == 1)
		spi_cs2_set_high();

}

static void nRF24_WriteRegister(uint8_t reg, uint8_t val,uint8_t device)
{

	if(device == 0)
		spi_cs_set_low();
	else if(device == 1)
		spi_cs2_set_low();
	uint8_t tmp[2];
	tmp[0] = NRF24_CMD_W_REGISTER | reg;
	tmp[1] = val;
	spi_write_data(tmp, 2);
	if(device == 0)
		spi_cs_set_high();
	else if(device == 1)
		spi_cs2_set_high();

}

static void nRF24_WriteRegisters(uint8_t reg, uint8_t* val, uint8_t len,uint8_t device)
{

	reg = NRF24_CMD_W_REGISTER | reg;

	if(device == 0)
		spi_cs_set_low();
	else if(device == 1)
		spi_cs2_set_low();
	spi_write_data(&reg, 1);
	spi_write_data(val, len);
	if(device == 0)
		spi_cs_set_high();
	else if(device == 1)
		spi_cs2_set_high();

}

void nRF24_SetPALevel(uint8_t lev,uint8_t device)
{
	uint8_t rf_setup = nRF24_ReadRegister(NRF24_RF_SETUP,device);
	rf_setup &= 0xF8; // Clear PWR bits
	rf_setup |= (lev<<1);
	nRF24_WriteRegister(NRF24_RF_SETUP, rf_setup,device);
}

void nRF24_SetDataRate(uint8_t dr,uint8_t device)
{
	uint8_t rf_setup = nRF24_ReadRegister(NRF24_RF_SETUP,device);
	rf_setup &= 0xD7; // Clear DR bits (1MBPS)
	if(dr == NRF24_RF_DR_250KBPS)
		rf_setup |= (1<<NRF24_RF_DR_LOW);
	else if(dr == NRF24_RF_DR_2MBPS)
		rf_setup |= (1<<NRF24_RF_DR_HIGH);
	nRF24_WriteRegister(NRF24_RF_SETUP, rf_setup,device);
}

void nRF24_EnableCRC(uint8_t onoff,uint8_t device)
{
	uint8_t config = nRF24_ReadConfig(device);

	if(onoff)
		config |= (1<<NRF24_EN_CRC);
	else
		config &= ~(1<<NRF24_EN_CRC);
	nRF24_WriteConfig(config,device);
}
void nRF24_SetCRCLength(uint8_t crcl,uint8_t device)
{
	uint8_t config = nRF24_ReadConfig(device);
	if(crcl == NRF24_CRC_WIDTH_2B)
		config |= (1<<NRF24_CRCO);
	else
		config &= ~(1<<NRF24_CRCO);
	nRF24_WriteConfig(config,device);
}
void nRF24_SetRetries(uint8_t ard, uint8_t arc,uint8_t device)
{
	// ard * 250us, arc repeats
	nRF24_WriteRegister(NRF24_SETUP_RETR, (((ard & 0x0F)<<NRF24_ARD) | ((arc & 0x0F)<<NRF24_ARC)),device);
}

void nRF24_SetPayloadSize(uint8_t pipe, uint8_t size,uint8_t device)
{
	if(pipe > 5)
		pipe = 5; // Block too high pipe number
	nRF24_WriteRegister(NRF24_RX_PW_P0 + pipe , (size & 0x3F),device);
}
void nRF24_SetRFChannel(uint8_t channel,uint8_t device)
{
	nRF24_WriteRegister(NRF24_RF_CH, (channel & 0x7F),device);
}
void nRF24_EnablePipe(uint8_t pipe, uint8_t onoff,uint8_t device)
{
	if(pipe > 5)
		pipe = 5; // Block too high pipe number
	uint8_t enable_pipe = nRF24_ReadRegister(NRF24_EN_RXADDR,device);
	if(onoff == 1)
		enable_pipe |= (1<<pipe);
	else
		enable_pipe &= ~(1<<pipe);
	nRF24_WriteRegister(NRF24_EN_RXADDR, enable_pipe,device);
}
void nRF24_AutoACK(uint8_t pipe, uint8_t onoff,uint8_t device)
{
	if(pipe > 5)
		pipe = 5; // Block too high pipe number
	uint8_t enaa = nRF24_ReadRegister(NRF24_EN_AA,device);
	if(onoff == 1)
		enaa |= (1<<pipe);
	else
		enaa &= ~(1<<pipe);
	nRF24_WriteRegister(NRF24_EN_AA, enaa,device);
}

void nRF24_SetAddressWidth(uint8_t size,uint8_t device)
{
	if(size > 5)
		size = 5; // Maximum are 5 bytes
	if(size < 3)
		size = 3; // Minimum are 3 bytes
	nRF24_WriteRegister(NRF24_SETUP_AW, ((size-2) & 0x03),device);
}
void nRF24_EnableRXDataReadyIRQ(uint8_t onoff,uint8_t device)
{
	uint8_t config = nRF24_ReadConfig(device);

	if(!onoff)
		config |= (1<<NRF24_RX_DR);
	else
		config &= ~(1<<NRF24_RX_DR);

	nRF24_WriteConfig(config,device);
}

void nRF24_EnableTXDataSentIRQ(uint8_t onoff,uint8_t device)
{
	uint8_t config = nRF24_ReadConfig(device);

	if(!onoff)
		config |= (1<<NRF24_TX_DS);
	else
		config &= ~(1<<NRF24_TX_DS);

	nRF24_WriteConfig(config,device);
}

void nRF24_EnableMaxRetransmitIRQ(uint8_t onoff,uint8_t device)
{
	uint8_t config = nRF24_ReadConfig(device);

	if(!onoff)
		config |= (1<<NRF24_MAX_RT);
	else
		config &= ~(1<<NRF24_MAX_RT);

	nRF24_WriteConfig(config,device);
}
void nRF24_ClearInterrupts(uint8_t device)
{
	uint8_t status = nRF24_ReadStatus(device);
	status |= (7<<4); // Clear bits 4, 5, 6.
	nRF24_WriteStatus(status,device);
}
uint8_t nRF24_ReadStatus(uint8_t device)
{
	return (nRF24_ReadRegister(NRF24_STATUS,device));
}
void nRF24_WriteStatus(uint8_t st,uint8_t device)
{
	nRF24_WriteRegister(NRF24_STATUS, st,device);
}
uint8_t nRF24_ReadConfig(uint8_t device)
{
	return (nRF24_ReadRegister(NRF24_CONFIG,device));
}
void nRF24_WriteConfig(uint8_t conf,uint8_t device)
{
	nRF24_WriteRegister(NRF24_CONFIG, conf,device);
}

uint8_t nRF24_GetDynamicPayloadSize(uint8_t device)
{
    uint8_t result = 0;

    result = nRF24_ReadRegister(NRF24_CMD_R_RX_PL_WID,device);

    if (result > 32) // Something went wrong :)
    {
        nRF24_FlushRX(device);
        nRF24_Delay(2);
        return 0;
    }
    return result;
}

void nRF24_InitGPIO()
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	  /** Configuration
	  PB1   ------> CE
	  PB2   ------> CE2
	  */
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_1|LL_GPIO_PIN_2;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);


}

void nRF24_Init(uint8_t device)
{
	if(device == 1)
		NRF24_CE_LOW;
	else if(device == 0)
		NRF24_CE2_LOW;

	if(device == 0)
		spi_cs_set_high();
	else if(device == 1)
		spi_cs2_set_high();
	nRF24_Delay(5);// delay

	// config things
	nRF24_SetPALevel(NRF24_PA_PWR_0dBM,device); // Radio power
	nRF24_SetDataRate(NRF24_RF_DR_250KBPS,device); // Data Rate
	nRF24_EnableCRC(1,device); // Enable CRC
	nRF24_SetCRCLength(NRF24_CRC_WIDTH_1B,device); // CRC Length 1 byte
	nRF24_SetRetries(0x04, 0x07,device); // 1000us, 7 times Auto Retransmission (ART)

#if (NRF24_DYNAMIC_PAYLOAD == 1)
	nRF24_WriteRegister(NRF24_FEATURE, nRF24_ReadRegister(NRF24_FEATURE,device) | (1<<NRF24_EN_DPL),device); // Enable dynamic payload feature
	nRF24_WriteRegister(NRF24_DYNPD, 0x3F,device); // Enable dynamic payloads for all pipes
#else
	nRF24_WriteRegister(NRF24_DYNPD, 0,device); // Disable dynamic payloads for all pipes
	nRF24_SetPayloadSize(0, NRF24_PAYLOAD_SIZE,device); // Set 32 bytes payload for pipe 0
#endif

	nRF24_SetRFChannel(10,device); // Set RF channel for transmission (frequency)
	nRF24_EnablePipe(0, 1,device); // Enable pipe 0
	nRF24_AutoACK(0, 1,device); // Enable auto ACK for pipe 0
	nRF24_SetAddressWidth(NRF24_ADDR_SIZE,device); // Set address size for correct system

	nRF24_Delay(1);

	nRF24_EnableRXDataReadyIRQ(1,device);
	nRF24_EnableTXDataSentIRQ(0,device);
	nRF24_EnableMaxRetransmitIRQ(0,device);

	nRF24_Delay(1);

	nRF24_ClearInterrupts(device);
}

//TX
nRF24_TX_Status nRF24_SendPacket(uint8_t* Data, uint8_t Size,uint8_t device)
{
	if(Size > 32)
		return NRF24_NO_TRANSMITTED_PACKET;

	nRF24_WriteTXPayload(Data, Size,device);
	nRF24_WaitTX(device);

	return NRF24_TRANSMITTED_PACKET;
}

void nRF24_SetTXAddress(uint8_t* address,uint8_t device)
{
	// TX address is storaged similar to RX pipe 0 - LSByte first
	uint8_t i;
	uint8_t address_rev[NRF24_ADDR_SIZE];

	nRF24_ReadRegisters(NRF24_RX_ADDR_P0, address_rev, NRF24_ADDR_SIZE,device); // Backup P0 address
	for(i = 0; i<NRF24_ADDR_SIZE; i++)
		addr_p0_backup[NRF24_ADDR_SIZE - 1 - i] = address_rev[i]; //Reverse P0 address

	for(i = 0; i<NRF24_ADDR_SIZE; i++)
		address_rev[NRF24_ADDR_SIZE - 1 - i] = address[i];
	//make pipe 0 address backup;

	nRF24_WriteRegisters(NRF24_RX_ADDR_P0, address_rev, NRF24_ADDR_SIZE,device); // Pipe 0 must be same for auto ACk
	nRF24_WriteRegisters(NRF24_TX_ADDR, address_rev, NRF24_ADDR_SIZE,device);

}

void nRF24_TX_Mode(uint8_t device)
{
	if(device == 1)
		NRF24_CE_LOW;
	else if(device == 0)
		NRF24_CE2_LOW;
	uint8_t config = nRF24_ReadConfig(device);
	// PWR_UP bit set
	config |= (1<<NRF24_PWR_UP);
	// PRIM_RX bit low
	config &= ~(1<<NRF24_PRIM_RX);
	nRF24_WriteConfig(config,device);
	// Reset status
	nRF24_WriteStatus((1<<NRF24_RX_DR)|(1<<NRF24_TX_DS)|(1<<NRF24_MAX_RT),device);
	// Flush RX
	nRF24_FlushRX(device);
	// Flush TX
	nRF24_FlushTX(device);

	nRF24_Delay(1);
}

void nRF24_WriteTXPayload(uint8_t * data, uint8_t size,uint8_t device)
{
#if (NRF24_DYNAMIC_PAYLOAD == 1)
	nRF24_WriteRegisters(NRF24_CMD_W_TX_PAYLOAD, data, size,device);
#else
	nRF24_WriteRegisters(NRF24_CMD_W_TX_PAYLOAD, data, NRF24_PAYLOAD_SIZE,device);
#endif
}

void nRF24_WaitTX(uint8_t device)
{
	uint8_t status;
	if(device == 1)
		NRF24_CE_HIGH;
	else if(device == 0)
		NRF24_CE2_HIGH;
	nRF24_Delay(2);
	if(device == 1)
		NRF24_CE_LOW;
	else if(device == 0)
		NRF24_CE2_LOW;
	do
	{
		nRF24_Delay(1);
		status = nRF24_ReadStatus(device);
	}while(!((status & (1<<NRF24_MAX_RT)) || (status & (1<<NRF24_TX_DS))));

}

void nRF24_FlushTX(uint8_t device)
{
	uint8_t command = NRF24_CMD_FLUSH_TX;

	if(device == 0)
		spi_cs_set_low();
	else if(device == 1)
		spi_cs2_set_low();
	spi_write_data(&command, 1);
	if(device == 0)
		spi_cs_set_high();
	else if(device == 1)
		spi_cs2_set_high();
}

// RX

void nRF24_FlushRX(uint8_t device)
{
	if(device == 0)
		spi_cs_set_low();
	else if(device == 1)
		spi_cs2_set_low();
	uint8_t command = NRF24_CMD_FLUSH_RX;
	spi_write_data(&command, 1);
	if(device == 0)
		spi_cs_set_high();
	else if(device == 1)
		spi_cs2_set_high();
}


void nRF24_RX_Mode(uint8_t device)
{
	uint8_t config = nRF24_ReadConfig(device);
	// Restore pipe 0 adress after comeback from TX mode
	nRF24_SetRXAddress(0, addr_p0_backup,device);
	// PWR_UP bit set
	config |= (1<<NRF24_PWR_UP);
	// PRIM_RX bit set
	config |= (1<<NRF24_PRIM_RX);
	nRF24_WriteConfig(config,device);
	// Reset status
	nRF24_WriteStatus((1<<NRF24_RX_DR)|(1<<NRF24_TX_DS)|(1<<NRF24_MAX_RT),device);
	// Flush RX
	nRF24_FlushRX(device);
	// Flush TX
	nRF24_FlushTX(device);

	if(device == 1)
		NRF24_CE_HIGH;
	else if(device == 0)
		NRF24_CE2_HIGH;
	nRF24_Delay(1);
}

void nRF24_SetRXAddress(uint8_t pipe, uint8_t* address,uint8_t device)
{
	// pipe 0 and pipe 1 are fully 40-bits storaged
	// pipe 2-5 is storaged only with last byte. Rest are as same as pipe 1
	// pipe 0 and 1 are LSByte first so they are needed to reverse address
	if((pipe == 0) || (pipe == 1))
	{
		uint8_t i;
		uint8_t address_rev[NRF24_ADDR_SIZE];
		for(i = 0; i<NRF24_ADDR_SIZE; i++)
			address_rev[NRF24_ADDR_SIZE - 1 - i] = address[i];
		nRF24_WriteRegisters(NRF24_RX_ADDR_P0 + pipe, address_rev, NRF24_ADDR_SIZE,device);
	}
	else
		nRF24_WriteRegister(NRF24_RX_ADDR_P0 + pipe, address[NRF24_ADDR_SIZE-1],device);
}

nRF24_RX_Status nRF24_ReceivePacket(uint8_t* Data, uint8_t *Size,uint8_t device)
{
#if (NRF24_INTERRUPT_MODE == 0)
	if(nRF24_RXAvailible(device))
	{
#endif
		nRF24_ReadRXPaylaod(Data, Size,device);
#if (NRF24_INTERRUPT_MODE == 0)
		return NRF24_RECEIVED_PACKET;
	}
	return NRF24_NO_RECEIVED_PACKET;
#endif
}

void nRF24_ReadRXPaylaod(uint8_t *data, uint8_t *size,uint8_t device)
{
#if (NRF24_DYNAMIC_PAYLOAD == 1)
	*size = nRF24_GetDynamicPayloadSize(device);
	nRF24_ReadRegisters(NRF24_CMD_R_RX_PAYLOAD, data, *size,device);
#else
	nRF24_ReadRegisters(NRF24_CMD_R_RX_PAYLOAD, data, NRF24_PAYLOAD_SIZE,device);
#endif
#if (NRF24_INTERRUPT_MODE == 0)
	nRF24_WriteRegister(NRF24_STATUS, (1<NRF24_RX_DR),device);
	if(nRF24_ReadStatus(device) & (1<<NRF24_TX_DS))
		nRF24_WriteRegister(NRF24_STATUS, (1<<NRF24_TX_DS),device);
#endif
}

uint8_t nRF24_RXAvailible(uint8_t device)
{
	uint8_t status = nRF24_ReadStatus(device);

	// RX FIFO Interrupt
	if ((status & (1 << 6)))
	{
		nrf24_rx_flag = 1;
		status |= (1<<6); // Interrupt flag clear
		nRF24_WriteStatus(status,device);
		return 1;
	}
	return 0;
}

//
// FIFO Status
//

uint8_t nRF24_ReadFifoStatus(uint8_t device)
{
	return (nRF24_ReadRegister(NRF24_FIFO_STATUS,device));
}

void nRF24_WriteFifoStatus(uint8_t st,uint8_t device)
{
	nRF24_WriteRegister(NRF24_FIFO_STATUS, st,device);
}

uint8_t nRF24_IsBitSetInFifoStatus(uint8_t Bit,uint8_t device)
{
	uint8_t FifoStatus;

	FifoStatus = nRF24_ReadFifoStatus(device);

	if(FifoStatus & (1<<Bit))
	{
		return 1;
	}

	return 0;
}

uint8_t nRF24_IsTxReuse(uint8_t device)
{
	return nRF24_IsBitSetInFifoStatus(NRF24_TX_REUSE,device);
}

uint8_t nRF24_IsTxFull(uint8_t device)
{
	return nRF24_IsBitSetInFifoStatus(NRF24_TX_FULL,device);
}

uint8_t nRF24_IsTxEmpty(uint8_t device)
{
	return nRF24_IsBitSetInFifoStatus(NRF24_TX_EMPTY,device);
}

uint8_t nRF24_IsRxFull(uint8_t device)
{
	return nRF24_IsBitSetInFifoStatus(NRF24_RX_FULL,device);
}

uint8_t nRF24_IsRxEmpty(uint8_t device)
{
	return nRF24_IsBitSetInFifoStatus(NRF24_RX_EMPTY,device);
}

uint8_t nRF24_Test(uint8_t transmitter) {
	  uint8_t input = 128;
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

// interuption


