/*
 * oled.c
 *
 *  Created on: Nov 16, 2024
 *      Author: Damian
 */
#include "oled.h"

static uint8_t SSD1306_Buffer[OLED_WIDTH * OLED_HEIGHT / 8];

static SSD1306_t SSD1306;

static uint8_t OLEDinited = 0;

void Init_OLED(void){
	uint8_t i;
	LL_mDelay(100);
	static uint8_t settings[28] ={
			0xAE, // Display off
			0x20, // Set memory adressing mode
			0x10, // Memory adressing mode: 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
			0xB0, // Set Page Start Address for Page Addressing Mode,0-7
			0xC8, // Set COM Output Scan Direction
			0x00, // Set low column address
			0x10, // Set high column address
			0x40, // Set start line address
			0x81, // Set contrast control register
			0x7F, // Contrast value from 0 to 255. 0x7F - default
			0xA1, // Set segment re-map 0 to 127. A1 - column address 0 is mapped to SEG0 (RESET) , A2 - column address 127 is mapped to SEG0
			0xA6, // Set normal display. A6 - normal(basic display colour black), A7 - inverse(basic display colour white)
			0xA8, // Set multiplex ratio(1 to 64). Basically height of the display
			OLED_HEIGHT-1, // Multiplex ratio value
			0xA4, // Set display mode. 0xA4 - Output follows RAM content, 0xA5 - Output ignores RAM content
			0xD3, // Set display offset
			0x00, // Display offest value. 0x00 - no offset
			0xD5, // Set display clock divide ratio/oscillator frequency
			0xF0, // Divide ratio/oscillator frequency value
			0xD9, // Set pre-charge period
			0x22, // Pre-charge period value
			0xDA, // Set com pins hardware configuration
			0x02, // Com pins hardware configuration. Refer to table 10-3 in datasheet(ssd1306)
			0xDB, // Set vcomh
			0x20, // Vcomh value, 0.77xVcc
			0x8D, // Set charge-pump
			0x14, // Charge pump enable
			0xAF}; // Display on
	for (i = 0; i < sizeof(settings); i++){
		if(i == 1){
			LL_mDelay(105); //required for proper power off of the disp
		}
		I2C1_reg_write_it(OLED_ADRESS, OLED_COMMAND, &settings[i], sizeof(settings[i]));
	}
	LL_mDelay(100); // required for proper power on of the disp

	Oled_Fill(0x00);
	Oled_UpdateScreen();
	OLEDinited = 1;

}

void Oled_Fill(uint8_t colour)
{
    uint32_t i;

    for(i = 0; i < sizeof(SSD1306_Buffer); i++)
    {
        SSD1306_Buffer[i] = (colour == 0x00) ? 0x00 : 0xFF;
    }
}

void Oled_UpdateScreen(void)
{
    uint8_t i;
    uint8_t command1;
    uint8_t command2 = 0x00;
    uint8_t command3 = 0x10;

    for (i = 0; i < 4; i++) {
    	command1 = 0xB0 + i;
        I2C1_reg_write_it(OLED_ADRESS, OLED_COMMAND, &command1, sizeof(command1));
        while (!i2c_transfer_complete);
        I2C1_reg_write_it(OLED_ADRESS, OLED_COMMAND, &command2, sizeof(command2));
        while (!i2c_transfer_complete);
        I2C1_reg_write_it(OLED_ADRESS, OLED_COMMAND, &command3, sizeof(command3));
        while (!i2c_transfer_complete);
    	I2C1_reg_write_it(OLED_ADRESS, OLED_MULTIPLE_DATA, &SSD1306_Buffer[OLED_WIDTH * i], OLED_WIDTH);
    	while (!i2c_transfer_complete);
    }
}


char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color)
{
    uint32_t i, b, j;

    // Check remaining space on current line
    if (OLED_WIDTH <= (SSD1306.CurrentX + Font.FontWidth) ||
        OLED_HEIGHT <= (SSD1306.CurrentY + Font.FontHeight))
    {
        // Not enough space on current line
        return 0;
    }

    // Translate font to screenbuffer
    for (i = 0; i < Font.FontHeight; i++)
    {
        b = Font.data[(ch - 32) * Font.FontHeight + i];
        for (j = 0; j < Font.FontWidth; j++)
        {
            if ((b << j) & 0x8000)
            {
                ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR) color);
            }
            else
            {
                ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
            }
        }
    }

    // The current space is now taken
    SSD1306.CurrentX += Font.FontWidth;

    // Return written char for validation
    return ch;
}

//
//  Write full string to screenbuffer
//
char ssd1306_WriteString(const char* str, FontDef Font, SSD1306_COLOR color)
{
    // Write until null-byte
    while (*str)
    {
        if (ssd1306_WriteChar(*str, Font, color) != *str)
        {
            // Char could not be written
            return *str;
        }

        // Next char
        str++;

    }

    // Everything ok
    return *str;
}

void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT)
    {
        // Don't write outside the buffer
        return;
    }

    // Check if pixel should be inverted
    if (SSD1306.Inverted)
    {
        color = (SSD1306_COLOR)!color;
    }

    // Draw in the correct color
    if (color == White)
    {
        SSD1306_Buffer[x + (y / 8) * OLED_WIDTH] |= 1 << (y % 8);
    }
    else
    {
        SSD1306_Buffer[x + (y / 8) * OLED_WIDTH] &= ~(1 << (y % 8));
    }
}
void ssd1306_InvertColors(void)
{
    SSD1306.Inverted = !SSD1306.Inverted;
}
void ssd1306_SetCursor(uint8_t x, uint8_t y)
{
    SSD1306.CurrentX = x;
    SSD1306.CurrentY = y;
}
void Oled_test(void)
{
	char buffer1[50] = {"Super fajny led!"};
		char buffer2[50];
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString(buffer1, Font_7x10, White);
		ssd1306_SetCursor(0, Font_7x10.FontHeight);
		ssd1306_WriteString(buffer2, Font_7x10, White);

		Oled_UpdateScreen();

}


void Oled_print_Gyroscope(uint8_t* response)
{
		if(OLEDinited == 0)
			return;
	  	char buffer1[50] = {"Uart przyszedl!"};
		char buffer2[50];
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString(buffer1, Font_7x10, White);
		ssd1306_SetCursor(0, Font_7x10.FontHeight);
		ssd1306_WriteString(buffer2, Font_7x10, White);

		Oled_UpdateScreen();

}

