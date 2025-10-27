/* 
 * File:   SSD1306.h
 * Author: sesha
 *
 * Created on 15 September, 2025, 12:04 AM
 */

#ifndef SSD1306_H
#define	SSD1306_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "font.h"
#include "I2C.h"

/*
 * Implement below code for selective page and column update
 * 
 * Requirement - Set all 8 bits of column 23 of page 3
 * 
// Set page and column first
ssd1306_send_command(0xB0 | 3);       // Set page address to 3
ssd1306_send_command(0x00 | (23 & 0x0F)); // Low nibble of column 23
ssd1306_send_command(0x10 | (23 >> 4));   // High nibble of column 23

// Send data byte
ssd1306_send_data(0xFF);  // Turn on all 8 pixels in this column of page 3
*/

void SSD1306_Init(void);
void SSD1306_WriteCommand(unsigned char cmd);
void SSD1306_String(const unsigned char *data);
void SSD1306_ClearScreen(void);
void SSD1306_GotoStart(void);

void SSD1306_WriteCommand(unsigned char cmd) {
	I2C_Start();
	I2C_Write(SSD1306_I2C_ADDRESS);
	I2C_Write(0x00);
	I2C_Write(cmd);
	I2C_Stop();
}

void SSD1306_Init(void) {
	I2C_Start();
	I2C_Write(SSD1306_I2C_ADDRESS);
	I2C_Write(0x00);
	
	SSD1306_WriteCommand(0xAE);     // 0xAE / Set Display OFF
	SSD1306_WriteCommand(0xD5);     // 0xD5 / 0x80 => D=1; DCLK = Fosc / D <=> DCLK = Fosc
	SSD1306_WriteCommand(0x80);
	SSD1306_WriteCommand(0xA8);     // 0xA8 / 0x3F (64MUX) for 128 x 64 version
	SSD1306_WriteCommand(0x3F);
	SSD1306_WriteCommand(0xD3);     // 0xD3 / 0x00 Remap set to normal
	SSD1306_WriteCommand(0x00);
	SSD1306_WriteCommand(0x40);
	SSD1306_WriteCommand(0x8D);     // 0x8D / Enable charge pump during display on
	SSD1306_WriteCommand(0x14);
	SSD1306_WriteCommand(0x20);     // 0x20 / Set Memory Addressing Mode
	SSD1306_WriteCommand(0x00);     // 0x00 / Horizontal Addressing Mode
                                    // 0x01 / Vertical Addressing Mode
                                    // 0x02 / Page Addressing Mode (RESET)
	SSD1306_WriteCommand(0xA1);     // 0xA0 / remap 0xA1
	SSD1306_WriteCommand(0xC8);
	SSD1306_WriteCommand(0xDA);     // 0xDA / 0x12 - Disable COM Left/Right remap, Alternative COM pin configuration
	SSD1306_WriteCommand(0x12);     //        0x12 - for 128 x 64 version
                                    //        0x02 - for 128 x 32 version
	SSD1306_WriteCommand(0x81);     // 0x81 / 0x7F - reset value (max 0xFF)
	SSD1306_WriteCommand(0x7F);
	SSD1306_WriteCommand(0xD9);     // 0xD9 / higher value less blinking
	SSD1306_WriteCommand(0xF1);     //        0xC2, 1st phase = 2 DCLK,  2nd phase = 13 DCLK
	SSD1306_WriteCommand(0xDB);     // 0xDB / Set V COMH Deselect, reset value 0x22 = 0,77xUcc
	SSD1306_WriteCommand(0x40);
	SSD1306_WriteCommand(0xA4);
	SSD1306_WriteCommand(0xA6);
	SSD1306_WriteCommand(0xAF);     // 0xAF / Set Display ON 
	
	I2C_Stop();
    
    __delay_ms(500);
    I2C_Start();
	I2C_Write(SSD1306_I2C_ADDRESS);
	I2C_Write(0x00);
	I2C_Write(0x21);
	I2C_Write(0);
	I2C_Write(127);
	I2C_Write(0x22);
	I2C_Write(0);
	I2C_Write(7);
	I2C_Stop();
}

void SSD1306_GotoStart(void) {
    I2C_Start();
	I2C_Write(SSD1306_I2C_ADDRESS);
	I2C_Write(0x00);
	I2C_Write(0x21);
	I2C_Write(0);
	I2C_Write(127);
	I2C_Write(0x22);
	I2C_Write(0);
	I2C_Write(7);
	I2C_Stop();
}

void SSD1306_String(const unsigned char *data) {
    
    unsigned char pixData[5];
    
    for (int a=0; data[a]!='\0'; a++){
        if (data[a] <= '~'){
            memcpy(pixData, TEXT5x8[data[a]-' '], 5);
        }else{
            memcpy(pixData, TEXT5x8[95], 5);
        }
        
        I2C_Start();
        I2C_Write(SSD1306_I2C_ADDRESS);
        I2C_Write(0x40);
        for (int i=0; i<5; i++){
            I2C_Write(pixData[i]);
        }
        I2C_Stop();
    }
}

void SSD1306_ClearScreen(void) {
    I2C_Start();
	I2C_Write(SSD1306_I2C_ADDRESS);
	I2C_Write(0x40);
	for (int i=0; i<8; i++) {
		for (int j=0; j<128; j++) {
			I2C_Write(0x00);
		}
	}
    I2C_Stop();
}

#ifdef	__cplusplus
}
#endif

#endif	/* SSD1306_H */

