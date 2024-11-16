/*
 * oled.h
 *
 *  Created on: Nov 16, 2024
 *      Author: Damian
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

/*
 * oled.h
 *
 *  Created on: Nov 16, 2024
 *      Author: Damian
 */
#include "main.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_i2c.h"
#include "stm32l4xx_ll_utils.h"
#include <stdbool.h>
#include "font.h"
#include "i2c.h"

#define OLED_ADRESS 0x78
#define OLED_COMMAND 0x00
#define OLED_DATA 0x20
#define OLED_MULTIPLE_DATA 0x40
#define OLED_WIDTH 128
#define OLED_HEIGHT 32



typedef enum {
    Black = 0x00,   // Black color, no pixel
    White = 0x01,   // Pixel is set. Color depends on LCD
} SSD1306_COLOR;
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Inverted;
    uint8_t Initialized;
} SSD1306_t;

void Init_OLED(void);

/*
 * Function that changes the colour of the display
 * 0x00 - black
 * 0x01 and else - white
 */
void Oled_Fill(uint8_t colour);

/*
 * Function that updates the screen
 * Sends data from buffer to screen memory
 */
void Oled_UpdateScreen(void);

/*
 * Function used to test the screen
 * Basically it turns the display on and off
 */
void Oled_test(void);

void ssd1306_InvertColors(void);
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
char ssd1306_WriteString(const char* str, FontDef Font, SSD1306_COLOR color);
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color);
void ssd1306_SetCursor(uint8_t x, uint8_t y);

#endif



