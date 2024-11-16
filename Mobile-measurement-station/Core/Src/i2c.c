/*
 * i2c.c
 *
 *  Created on: Nov 16, 2024
 *      Author: Damian
 */


#include "i2c.h"

/* USER CODE BEGIN 0 */
static buffer_t rx_buffer = {0};
static buffer_t tx_buffer = {0};

volatile bool i2c_transfer_complete = false;
/* USER CODE END 0 */

/* I2C1 init function */
void MX_I2C1_Init(void)
{

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

	GPIO_InitStruct.Pin = I2C1_SCL_PIN|I2C1_SDA_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
	LL_GPIO_Init(I2C1_PORT, &GPIO_InitStruct);

	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

	NVIC_SetPriority(I2C1_EV_IRQn, 0);
	NVIC_EnableIRQ(I2C1_EV_IRQn);

	LL_I2C_Disable(I2C1);
	LL_I2C_ConfigFilters(I2C1, LL_I2C_ANALOGFILTER_ENABLE, 0x00);
	LL_I2C_SetTiming(I2C1, 0x00303D5B);
	LL_I2C_EnableClockStretching(I2C1);
	LL_I2C_SetMode(I2C1, LL_I2C_MODE_I2C);
	LL_I2C_Enable(I2C1);
}

/* USER CODE BEGIN 1 */
int32_t I2C1_reg_write_it(uint16_t slave_addr, uint16_t reg_addr, uint8_t *data_ptr, uint16_t size)
{
	tx_buffer.data_ptr = data_ptr;
	tx_buffer.count = size;

	i2c_transfer_complete = false;

	LL_I2C_HandleTransfer(I2C1, slave_addr, LL_I2C_ADDRSLAVE_7BIT, (uint32_t)(size+1), LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

	while(LL_I2C_IsActiveFlag_TXIS(I2C1) == 0)
		;

	LL_I2C_TransmitData8(I2C1, reg_addr);

	LL_I2C_EnableIT_TX(I2C1);
	LL_I2C_EnableIT_STOP(I2C1);

	return 0;
}


int32_t I2C1_reg_read_it(uint16_t slave_addr, uint16_t reg_addr, uint8_t *data_ptr, uint16_t size)
{
	rx_buffer.data_ptr = data_ptr;
	rx_buffer.count = size;

	i2c_transfer_complete = false;

	LL_I2C_HandleTransfer(I2C1, slave_addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);

	while(LL_I2C_IsActiveFlag_TXIS(I2C1) == 0)
		;

	LL_I2C_TransmitData8(I2C1, reg_addr);

	while(LL_I2C_IsActiveFlag_TC(I2C1) == 0)
		;

	LL_I2C_HandleTransfer(I2C1, slave_addr, LL_I2C_ADDRSLAVE_7BIT, (uint32_t)size, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

	LL_I2C_EnableIT_RX(I2C1);
	LL_I2C_EnableIT_STOP(I2C1);

	return 0;
}

void I2C1_it_transmit_callback(void)
{
	if(tx_buffer.count > 0)
	{
		LL_I2C_TransmitData8(I2C1, *tx_buffer.data_ptr);

		tx_buffer.data_ptr++;
		tx_buffer.count--;
	}

	if(tx_buffer.count <= 0)
	{
		LL_I2C_DisableIT_TX(I2C1);
	}
}

void I2C1_it_receive_callback(void)
{
	if(rx_buffer.count > 0)
	{
		*rx_buffer.data_ptr = LL_I2C_ReceiveData8(I2C1);

		rx_buffer.data_ptr++;
		rx_buffer.count--;
	}

	if(rx_buffer.count <= 0)
	{
		LL_I2C_DisableIT_RX(I2C1);
	}
}

void Set_Tx_Buffer(uint8_t *data_ptr, uint16_t size)
{
	tx_buffer.data_ptr = data_ptr;
	tx_buffer.count = size;
}

void Set_Rx_Buffer(uint8_t *data_ptr, uint16_t size)
{
	rx_buffer.data_ptr = data_ptr;
	rx_buffer.count = size;
}
void I2C1_EV_IRQHandler(void)
{
	if(LL_I2C_IsActiveFlag_TXIS(I2C1) && LL_I2C_IsEnabledIT_TX(I2C1))
	{
		I2C1_it_transmit_callback();
	}
	else if(LL_I2C_IsActiveFlag_RXNE(I2C1) && LL_I2C_IsEnabledIT_RX(I2C1))
	{
		I2C1_it_receive_callback();
	}
	else if(LL_I2C_IsActiveFlag_STOP(I2C1) && LL_I2C_IsEnabledIT_STOP(I2C1))
	{
		LL_I2C_ClearFlag_STOP(I2C1);
		LL_I2C_DisableIT_STOP(I2C1);

		i2c_transfer_complete = true;
	}
}
/* USER CODE END 1 */
