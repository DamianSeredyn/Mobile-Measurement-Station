/*
 * bme280.c
 *
 *  Created on: Nov 16, 2024
 *      Author: szymo
 */
#include "bme280.h"

static uint16_t dig_T1,
         dig_P1;

static int16_t  dig_T2, dig_T3,
         dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

static int32_t tRaw, pRaw;

uint8_t BME280_test(void){
	uint8_t test_data;
	I2C1_reg_read_it(BME280_ADR, BME280_ID, &test_data, sizeof(test_data));
	while (!i2c_transfer_complete);
	if (test_data == 0x58 || test_data == 0x57 || test_data == 0x56 || test_data == 0x60){
		return test_data;
	}else{
		return 1;
	}
}

void BME280_trim_param_read(void){
	uint8_t trimdata[24];
	for (int i = 0; i < sizeof(trimdata); i++){
		I2C1_reg_read_it(BME280_ADR, BME280_TRIM_PARAM_REG_START+i, &trimdata[i], sizeof(trimdata[i]));
		while (!i2c_transfer_complete);
	}
	dig_T1 = (trimdata[1]<<8) | trimdata[0];
	dig_T2 = (trimdata[3]<<8) | trimdata[2];
	dig_T3 = (trimdata[5]<<8) | trimdata[4];
	dig_P1 = (trimdata[7]<<8) | trimdata[5];
	dig_P2 = (trimdata[9]<<8) | trimdata[6];
	dig_P3 = (trimdata[11]<<8) | trimdata[10];
	dig_P4 = (trimdata[13]<<8) | trimdata[12];
	dig_P5 = (trimdata[15]<<8) | trimdata[14];
	dig_P6 = (trimdata[17]<<8) | trimdata[16];
	dig_P7 = (trimdata[19]<<8) | trimdata[18];
	dig_P8 = (trimdata[21]<<8) | trimdata[20];
	dig_P9 = (trimdata[23]<<8) | trimdata[22];
}

void BME280_init(void){
	uint8_t commands[] = {
			BME280_RESET_COMMAND,
			BME280_CONFIG_REG_PARAM,
			BME280_CTRL_MEAS_REG_PARAM,
	};
	if(BME280_test() != 1){
		BME280_trim_param_read();
		I2C1_reg_write_it(BME280_ADR, BME280_RESET, &commands[0], sizeof(commands[0]));
		LL_mDelay(100); //delay for reset just in case, might not be required
		I2C1_reg_write_it(BME280_ADR, BME280_CONFIG_REG, &commands[1], sizeof(commands[1]));
		I2C1_reg_write_it(BME280_ADR, BME280_CTRL_MEAS_REG, &commands[2], sizeof(commands[2]));
	}
}

void BME280_wakeup(void){
	uint8_t reg_data;
	I2C1_reg_read_it(BME280_ADR, BME280_CTRL_MEAS_REG, &reg_data, sizeof(reg_data));
	while (!i2c_transfer_complete);
	reg_data = reg_data | 0x01;
	I2C1_reg_write_it(BME280_ADR, BME280_CTRL_MEAS_REG, &reg_data, sizeof(reg_data));
	LL_mDelay(10); // delay required for sensor to measure temp. and pressure
}

void BME280_read_raw(void){
	uint8_t RawData[6];
	BME280_wakeup();
	for(int i = 0; i < sizeof(RawData); i++){
		I2C1_reg_read_it(BME280_ADR, BME280_RAW_REG_START+i, &RawData[i], sizeof(RawData[i]));
		while (!i2c_transfer_complete);
	}
	pRaw = (RawData[0]<<12)|(RawData[1]<<4)|(RawData[2]>>4);
	tRaw = (RawData[3]<<12)|(RawData[4]<<4)|(RawData[5]>>4);
}

int32_t t_fine;
// Function from page 23 of BME280 datasheet
int32_t BME280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1)))>> 12) *((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

// Function from page 23 of BME280 datasheet (modified a bit)
uint32_t BME280_compensate_P_uint32(int32_t adc_P)
{
	int32_t var1, var2;
		uint32_t p;
		var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
		var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
		var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
		var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
		var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) *var1)>>1))>>18;
		var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
		if (var1 == 0)
		{
			return 0; // avoid exception caused by division by zero
		}
		p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
		if (p < 0x80000000)
		{
			p = (p << 1) / ((uint32_t)var1);
		}
		else
		{
			p = (p / (uint32_t)var1) * 2;
		}
		var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
		var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
		p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
		return p;
}

float BME280_read_temp(void){
	BME280_read_raw();
	return (float)BME280_compensate_T_int32(tRaw)/100;
}

uint32_t BME280_read_pressure(void){
	BME280_read_raw();
	return BME280_compensate_P_uint32(pRaw);
}


