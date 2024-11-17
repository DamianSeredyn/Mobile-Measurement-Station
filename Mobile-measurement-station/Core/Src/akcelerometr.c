/*
 * akcelerometr.c
 *
 *  Created on: Nov 16, 2024
 *      Author: ciast
 */
#include "akcelerometr.h"
#include "main.h"
#include "i2c.h"

void process_accel_data(int16_t raw_x, int16_t raw_y, int16_t raw_z, float *scaled_x, float *scaled_y, float *scaled_z);
float count_data(float data, float data2);
float distance, distance1, distance2 = 0;
/*void init_Akcelerometr(void){

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);


	LL_GPIO_SetPinPull(GPIO_AKCELEROMETR_PRZERWANIE, GPIO_AKCELEROMETR_PRZERWANIE_PIN, LL_GPIO_PULL_DOWN);
	LL_GPIO_SetPinSpeed(GPIO_AKCELEROMETR_PRZERWANIE, GPIO_AKCELEROMETR_PRZERWANIE_PIN, LL_GPIO_SPEED_FREQ_LOW);
	LL_GPIO_SetPinMode(GPIO_AKCELEROMETR_PRZERWANIE, GPIO_AKCELEROMETR_PRZERWANIE_PIN, LL_GPIO_MODE_INPUT);

	LL_EXTI_SetEXTISource(GPIO_AKCELEROMETR_PRZERWANIE, GPIO_AKCELEROMETR_PRZERWANIE_PIN);

	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_6);

	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_6);

	NVIC_SetPriority(EXTI9_5_IRQn, 0);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
}*/

/*void EXTI4_15_IRQHandler(void)
{
	if(LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_6) != RESET)
	{
		LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_6);
	}
}*/
#define FS_RANGE 4.0          // Zakres ±2g to 4g
#define RESOLUTION 32768.0    // Rozdzielczość 16-bitowa (2^15)
#define SENSITIVITY (FS_RANGE / RESOLUTION) // Wartość na 1 LSB w jednostkach g
#define ARRAY_SIZE 10
#define GRAVITY 9.81

// Offsety w LSB (zmierz w pozycji zerowej)
#define OFFSET_X 0
#define OFFSET_Y 64
#define OFFSET_Z (-9027)

int i = 0;
int e = 0;

bool check_accelerometr_alive(void){
	uint8_t output = 0;
	I2C1_reg_read_it(0x3A, 0x0F, &output, 1);
	if (output == 0x41){
		return 0;
	}else{
		return 1;
	}
}

// USE THESE FUCTIONS TO RETURN CALCULATED DATA
float return_x(void){
return distance;
}
float return_y(void){
return distance1;
}
float return_z(void){
return distance2;
}

void read_accelerometr(void){
	uint8_t input[6] = {OUT_X_H_A, OUT_X_L_A, OUT_Y_H_A, OUT_Y_L_A, OUT_Z_H_A, OUT_Z_L_A};
	uint8_t output[6];
	 float scaled_x, scaled_y, scaled_z, scaled_x2, scaled_y2, scaled_z2;
	for (int i = 0; i < 6; i++){
	I2C1_reg_read_it(ACCEL_ADRESS, input[i], &output[i], 1);
	while(i2c_transfer_complete != true);
	}
	int16_t accel_x = (int16_t)((output[0] << 8) | output[1]);
	int16_t accel_y = (int16_t)((output[2] << 8) | output[3]);
	int16_t accel_z = (int16_t)((output[4] << 8) | output[5]);
	e++;
	if (e == 1){
	process_accel_data(accel_x, accel_y, accel_z, &scaled_x, &scaled_y, &scaled_z);
	}
	if (e == 2) {
		process_accel_data(accel_x, accel_y, accel_z, &scaled_x2, &scaled_y2, &scaled_z2);
		float o = distance;
		float o1 = distance1;
		float o2 = distance2;
		distance = distance + count_data(scaled_x, scaled_x2); // X-distance
		distance1 = distance1 + count_data(scaled_y, scaled_y2); //Y-distance
		distance2 = distance2 + count_data(scaled_z, scaled_z2); //Z-distance
		e = 0;
	}

}

void init_accelerometr(void){
	uint8_t data = 0x37;
	I2C1_reg_write_it(ACCEL_ADRESS, CTRL_REG1_A, &data, 1);
	while(i2c_transfer_complete != true);
}


void process_accel_data(int16_t raw_x, int16_t raw_y, int16_t raw_z, float *scaled_x, float *scaled_y, float *scaled_z) {
    // Usuń offset
    int16_t corrected_x = raw_x - OFFSET_X;
    int16_t corrected_y = raw_y - OFFSET_Y;
    int16_t corrected_z = raw_z - OFFSET_Z;

    // Przeskaluj na g
    *scaled_x = corrected_x * SENSITIVITY;
    *scaled_y = corrected_y * SENSITIVITY;
    *scaled_z = corrected_z * SENSITIVITY/3;
}

float count_data(float data, float data2){
	float accel_x[2] = {data, data2};
	float accel_x_mps2[2];

	float velocity_x[2] = {0};  // Prędkość w m/s
	    float position_x[2] = {0};  // Pozycja w m
	    float dt = 0.01;  // Próbkowanie co 10 ms
	    for (int i = 0; i < 2; i++) {
	            accel_x_mps2[i] = accel_x[i] * GRAVITY;
	        }
	    for (int i = 1; i < 2; i++) {
	            velocity_x[i] = velocity_x[i - 1] + accel_x_mps2[i] * dt;
	        }
	    for (int i = 1; i < 2; i++) {
	           position_x[i] = position_x[i - 1] + velocity_x[i] * dt;
	       }
	    return position_x[1];
}

