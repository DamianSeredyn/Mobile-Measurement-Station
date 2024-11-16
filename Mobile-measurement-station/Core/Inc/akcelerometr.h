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


#define GPIO_AKCELEROMETR_PRZERWANIE_PIN LL_GPIO_PIN_6
#define GPIO_AKCELEROMETR_PRZERWANIE GPIOA



#endif /* INC_AKCELEROMETR_H_ */
