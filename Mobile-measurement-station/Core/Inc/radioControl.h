/*
 * radioControl.h
 *
 *  Created on: Nov 10, 2024
 *      Author: szymo
 */

#ifndef INC_RADIOCONTROL_H_
#define INC_RADIOCONTROL_H_


#define FORWARD_BUTTON_Pin LL_GPIO_PIN_0
#define LEFT_BUTTON_Pin LL_GPIO_PIN_1
#define RIGHT_BUTTON_Pin LL_GPIO_PIN_4

void init_ControlerButtons(void);
uint8_t robotComandsSend_test(uint8_t transmitter);
void robotComandsSend(uint8_t transmitter);

#endif /* INC_RADIOCONTROL_H_ */