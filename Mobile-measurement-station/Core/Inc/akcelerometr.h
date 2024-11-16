/*
 * akcelerometr.h
 *
 *  Created on: Nov 16, 2024
 *      Author: ciast
 */

#ifndef INC_AKCELEROMETR_H_
#define INC_AKCELEROMETR_H_

#include "main.h"
#include "i2c.h"

void EXTI4_15_IRQHandler(void);
void init_ControlerButtons(void);

#define WHO_AM_I_A 0XOF;
#define ACT_THS_A 0X1E;
#define ACT_DUR_A 0X1F;
#define CTRL_REG1_A 0X20;
#define CTRL_REG2_A 0X21;
#define CTRL_REG3_A 0X22;
#define CTRL_REG4_A 0X23;
#define CTRL_REG5_A 0X24;
#define CTRL_REG6_A 0X25;
#define CTRL_REG7_A 0X26;
#define STATUS_REG_A 0X27;
#define OUT_X_L_A OX28;
#define OUT_X_H_A OX29;
#define OUT_Y_L_A OX2A;
#define OUT_Y_H_A OX2B;
#define OUT_Z_L_A OX2C;
#define OUT_Z_H_A OX2D;
#define FIFO_CTRL OX2E;
#define FIFO_SRC 0X2F;

#define GPIO_AKCELEROMETR_PRZERWANIE_PIN LL_GPIO_PIN_6
#define GPIO_AKCELEROMETR_PRZERWANIE GPIOA



#endif /* INC_AKCELEROMETR_H_ */
