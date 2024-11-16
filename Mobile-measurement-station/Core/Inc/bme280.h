#ifndef BME280_H
#define BME280_H

#include "i2c.h"

#define BME280_ADR				(0xEC)
#define BME280_ID				(0xD0)
#define BME280_RESET			(0xE0)
#define BME280_CTRL_MEAS_REG	(0xF4)
#define BME280_CONFIG_REG		(0xF5)

#define BME280_TRIM_PARAM_REG_START		(0x88)
#define BME280_RESET_COMMAND			(0xB6)
#define BME280_CONFIG_REG_PARAM			(0xA4) //101(Tstandby) 001(IIR filter) 00
#define BME280_CTRL_MEAS_REG_PARAM		(0x24) //001(osrs_t) 001(osrs_p) 00(sleep mode)
#define BME280_RAW_REG_START			(0xF7)

/*
 * Function used to test BME280 sensor
 * reads the chip id and returns it
 * if id doesn't match up it returns 1 instead
 */
uint8_t BME280_test(void);

/*
 * Function used to read the sensor's trimming parameters.
 * These parameters are stored in the sensor's non-volatile memory and are
 * used to compensate the raw temperature and pressure measurements.
 * They cannot be modified.
 */
void BME280_trim_param_read(void);

/*
 * Function used to initialize BME280
 * The settings are based on Table 7 for Weather monitoring (lowest power) mode.
 * It also reads trimming parameters
 */
void BME280_init(void);

/*
 * Function used to wake up the BME280 and put it into forced mode.
 * In this mode it reads values of temperature and pressure
 * then goes to sleep again
 */
void BME280_wakeup(void);

/*
 * Function used to read raw values of temperature and pressure registers
 * These values are later used to compute real values of temp. and pres.
 */
void BME280_read_raw(void);

/*
 * Function used to compute real value of temp.
 * @param adc_T Represents raw data(tRaw)
 */
int32_t BME280_compensate_T_int32(int32_t adc_T);

/*
 * Function used to compute real value of pres.
 * @param adc_P Represents raw data(pRaw)
 */
uint32_t BME280_compensate_P_uint32(int32_t adc_P);

/*
 * Function used to get the real temperature in degrees Celsius.
 * It reads raw data from the sensor, compensates it using the calibration parameters,
 * and returns the temperature as a float value.
 * @return Temperature in degrees Celsius.
 */
float BME280_read_temp(void);

/*
 * Function used to get the real atmospheric pressure in Pa.
 * It reads raw data from the sensor, compensates it using the calibration parameters,
 * and returns the pressure as an unsigned 32-bit integer.
 * @return Pressure in Pa.
 */
uint32_t BME280_read_pressure(void);

#endif /* BME280_H */
