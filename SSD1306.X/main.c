
// PIC18F4580 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = IRCIO67    // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (VBOR set to 2.1V)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = OFF      // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config BBSIZ = 1024     // Boot Block Size Select bit (1K words (2K bytes) boot block)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ 4000000
#define SSD1306_I2C_ADDRESS 0x78 // SSD1306 I2C ADDRESS (0x3C << 1)

#include <xc.h>

void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(unsigned char data);
void I2C_Hold(void);
void SSD1306_Init(void);
void SSD1306_WriteCommand(unsigned char cmd);
void SSD1306_String(int numChar, const unsigned char *data);
void SSD1306_ClearScreen(void);

const unsigned char TestStr[66] = {0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F,		//H
				0x00, 0x7F, 0x49, 0x49, 0x49, 0x41,		//E
				0x00, 0x7F, 0x40, 0x40, 0x40, 0x40,		//L
				0x00, 0x7F, 0x40, 0x40, 0x40, 0x40,		//L
				0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E,		//O
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		//Space
				0x00, 0x7F, 0x20, 0x10, 0x20, 0x7F,		//W
				0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E,		//O
				0x00, 0x7F, 0x09, 0x09, 0x09, 0x76,		//R
				0x00, 0x7F, 0x40, 0x40, 0x40, 0x40,		//L
				0x00, 0x7F, 0x41, 0x41, 0x41, 0x3E,};	//D

void main(void) {
    OSCCON = 0b01100010;
    TRISD = 0x00; //RD1 declared as output pin
    TRISC = 0xFF;
    
    __delay_ms(500);
    I2C_Init();
    __delay_ms(500);
    SSD1306_Init();
    __delay_ms(500);
    SSD1306_ClearScreen();
    __delay_ms(500);
    SSD1306_String(11, &TestStr);
    __delay_ms(500);

    while (1) {
        LATDbits.LATD1 = 0; //Make RD1 high to glow LED
        __delay_ms(1000);
        LATDbits.LATD1 = 1; //Make RD1 low to OFF LED
        __delay_ms(1000);
    }
    return;
}

void I2C_Init(void) {
	SSPCON1 = 0b00101000;
	SSPCON2 = 0b00000000;
	SSPADD = ((_XTAL_FREQ / 4) / 100000) - 1;
	SSPSTAT = 0b00000000;
}

void I2C_Hold(void) {
	while ((SSPCON2 & 0b00011111) || (SSPSTAT & 0b00000100));
}

void I2C_Start(void) {
	I2C_Hold();
	SEN = 1;
	while (SEN);
}

void I2C_Stop(void) {
	I2C_Hold();
	PEN = 1;
	while(PEN);
}

void I2C_Write(unsigned char data) {
	I2C_Hold();
	SSPBUF = data;
	while(!SSPIF);
	SSPIF = 0;
}

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
	
	SSD1306_WriteCommand(0xAE);
	SSD1306_WriteCommand(0xD5);
	SSD1306_WriteCommand(0x80);
	SSD1306_WriteCommand(0xA8);
	SSD1306_WriteCommand(0x3F);
	SSD1306_WriteCommand(0xD3);
	SSD1306_WriteCommand(0x00);
	SSD1306_WriteCommand(0x40);
	SSD1306_WriteCommand(0x8D);
	SSD1306_WriteCommand(0x14);
	SSD1306_WriteCommand(0x20);
	SSD1306_WriteCommand(0x00);
	SSD1306_WriteCommand(0xA1);
	SSD1306_WriteCommand(0xC8);
	SSD1306_WriteCommand(0xDA);
	SSD1306_WriteCommand(0x12);
	SSD1306_WriteCommand(0x81);
	SSD1306_WriteCommand(0x7F);
	SSD1306_WriteCommand(0xD9);
	SSD1306_WriteCommand(0xF1);
	SSD1306_WriteCommand(0xDB);
	SSD1306_WriteCommand(0x40);
	SSD1306_WriteCommand(0xA4);
	SSD1306_WriteCommand(0xA6);
	SSD1306_WriteCommand(0xAF);
	
	I2C_Stop();
}

void SSD1306_String(int numChar, const unsigned char *data) {
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
	
	for (int i=0; i<(numChar*6); i++){
		I2C_Start();
		I2C_Write(SSD1306_I2C_ADDRESS);
		I2C_Write(0x40);
		I2C_Write(data[i]);
		I2C_Stop();
	}
}

void SSD1306_ClearScreen(void) {
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
	
	for (int i=0; i<8; i++) {
		for (int j=0; j<128; j++) {
			I2C_Start();
			I2C_Write(SSD1306_I2C_ADDRESS);
			I2C_Write(0x40);
			I2C_Write(0x00);
			I2C_Stop();
		}
	}
}