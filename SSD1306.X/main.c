
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

#include "SSD1306.h"
#include "EUSART.h"

void __interrupt() _ISR(void) {
    if (PIR1bits.RCIF) {
        EUSART_Rx_InterruptHandler();
    }
}

void UpdateScreen(unsigned char command) {
    
    unsigned char printVar[4] = {command, ',', ' ', '\0'};
    SSD1306_String(&printVar);
    
}

unsigned int GetDistance (void) {
    unsigned int distance_cm = 0;
    unsigned int pulseWidth = 0;
    
    unsigned long timeout = 0;
    
    timeout = 0;
    LATDbits.LATD2 = 0;
    __delay_us(2);
    LATDbits.LATD2 = 1;
    __delay_us(10);
    LATDbits.LATD2 = 0;
    
    while(!PORTBbits.RB0 && timeout++ < 60000);
    
    if (timeout >= 60000)
        return 61000;
    
    TMR1H = 0;
    TMR1L = 0;
    PIR1bits.TMR1IF = 0;
    T1CONbits.TMR1ON = 1;
    
    timeout = 0;
    while(PORTBbits.RB0 && timeout++ < 60000);
    
    T1CONbits.TMR1ON = 0;
    
    if (timeout >= 60000 || PIR1bits.TMR1IF)
        return 62000;
    
    pulseWidth = ((TMR1H << 8) | TMR1L);
    //distance_cm = (pulseWidth * 343) / 20000;
    distance_cm = pulseWidth;
    
    return distance_cm;
}

void main(void) {
    OSCCON = 0b01100010;
    ADCON1 = 0x0F;
    CMCON = 0x07;
    TRISD = 0x00;           //RD1 declared as output pin for blinking LED
    TRISC = 0xFF;           //PORTC declared as input
    TRISCbits.TRISC6 = 0;   //RC6 made output for USART Tx
    TRISDbits.TRISD3 = 1;
    TRISDbits.TRISD2 = 0;
    TRISB = 0xFF;
    TRISBbits.TRISB0 = 1;
    
    //INTCON2bits.RBPU = 0;      // Enable PORTB pull-ups

    
    T1CON = 0x10;
    
    __delay_ms(500);
    I2C_Init();
    __delay_ms(500);
    SSD1306_Init();
    __delay_ms(500);
    SSD1306_ClearScreen();
    __delay_ms(500);
    SSD1306_String("USART test: ");
    __delay_ms(1000);
    USART_Init();
    __delay_ms(100);
    
    SendString("Connected to HC-06\n");
    unsigned char cmd = 0;
    unsigned char oldCmd = 0;
    unsigned char updateScn = 0;
    unsigned int dist = 0;
    unsigned char a[7];
    unsigned int counter = 0;

    while (1) {
        
        if (cmd != oldCmd) {
            if (cmd == '1') {
                LATDbits.LATD1 = 1;         //Make RD1 low to OFF LED
            } else if (cmd == '0') {
                LATDbits.LATD1 = 0;         //Make RD1 high to glow LED
            }
            oldCmd = cmd;
            updateScn = 1;
        }
        
        if (updateScn == 1) {
            UpdateScreen(oldCmd);
            updateScn = 0;
        }
        
        if (rxFlag) {
            cmd = rxData;
            SendByte(cmd);
            rxFlag = 0;
        }
        
        if (PORTDbits.RD3) {
            LATDbits.LATD1 = 0;         //Make RD1 low to OFF LED
        } else {
            LATDbits.LATD1 = 1;         //Make RD1 high to glow LED
        }
        
        //SSD1306_ClearScreen();
        /*dist = GetDistance();
        
        sprintf(a, "%u cm\n", dist);
        SendString(a);*/
        
        /*if (dist == 61000) {
            //SSD1306_ClearScreen();
            SSD1306_GotoStart();
            SSD1306_String("Echo didn't went high");
        } else if (dist == 62000) {
            //SSD1306_ClearScreen();
            SSD1306_GotoStart();
            SSD1306_String("Echo didn't went low");
        } else {
            SSD1306_ClearScreen();
            SSD1306_GotoStart();
            sprintf(a, "%d cm", dist);
            SSD1306_String(a);
        }*/
        
        LATDbits.LATD2 = 0;
        __delay_us(2);
        LATDbits.LATD2 = 1;
        __delay_us(10);
        LATDbits.LATD2 = 0;
        
        /*while (!PORTBbits.RB0 && (counter < 20000)){
            counter++;
        }
        LATDbits.LATD1 = 1;
        counter = 0;
        while (PORTBbits.RB0 && (counter < 20000)){
            counter++;
        }
        LATDbits.LATD1 = 0;*/
        
        /*while (PORTBbits.RB0);    // Wait for echo high
        LATDbits.LATD1 = 1;        // LED ON while echo high
        while (!PORTBbits.RB0);     // Wait for echo low
        LATDbits.LATD1 = 0;*/        // LED OFF


        
        while(!PORTDbits.RD3){
            SendString("Echo not yet high\n");
        }

        TMR1H = 0;
        TMR1L = 0;
        PIR1bits.TMR1IF = 0;
        T1CONbits.TMR1ON = 1;
        
        while(PORTDbits.RD3){
            SendString("Echo stuck high\n");
        }

        T1CONbits.TMR1ON = 0;

        dist = ((TMR1H << 8) | TMR1L);
        sprintf(a, "%u cm\n", dist);
        SendString(a);
        
        __delay_ms(1000);
        //distance_cm = (pulseWidth * 343) / 20000;
    }
    
    return;
}