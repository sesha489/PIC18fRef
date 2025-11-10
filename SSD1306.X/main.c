
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

volatile unsigned long tone_toggle_count = 0; // how many toggles left

void __interrupt() _ISR(void) {
    if (PIR1bits.RCIF) {
        EUSART_Rx_InterruptHandler();
    }
    
    if (PIR1bits.TMR2IF) {
        PIR1bits.TMR2IF = 0;

        if (tone_toggle_count > 0) {
            LATDbits.LATD1 ^= 1;  // toggle buzzer pin
            tone_toggle_count--;
        } else {
            T2CONbits.TMR2ON = 0; // stop timer
            LATDbits.LATD1 = 0;
        }
    }
}

void UpdateScreen(unsigned int dist) {
    
    unsigned char a[7];
    SSD1306_ClearScreen();
    SSD1306_GotoStart();
    sprintf(a, "%u cm\n", dist);
    SendString(a);
    SSD1306_String(a);
    
}

unsigned int GetDistance (void) {
    unsigned int distance_cm = 0;
    unsigned int pulseWidth = 0;
    
    TMR1H = 0;
    TMR1L = 0;
    PIR1bits.TMR1IF = 0;
    
    LATDbits.LATD2 = 0;
    __delay_us(2);
    LATDbits.LATD2 = 1;
    __delay_us(11);
    LATDbits.LATD2 = 0;
    
    while (!PORTDbits.RD3);
    T1CONbits.TMR1ON = 1;
    while (PORTDbits.RD3);
    T1CONbits.TMR1ON = 0;
    
    pulseWidth = ((TMR1H << 8) | TMR1L);
    distance_cm = pulseWidth * 0.0343;
    
    return distance_cm;
}

void tone(unsigned int frequency, unsigned int duration_ms) {
    if (frequency == 0) return;

    // Stop timer and put buzzer low before reconfig
    T2CONbits.TMR2ON = 0;
    LATDbits.LATD1 = 0;

    // compute half-period in microseconds
    unsigned long period_us = 1000000UL / (unsigned long)frequency;  // full period
    unsigned int toggle_time_us = (unsigned int)(period_us / 2UL);    // half period

    // choose prescaler and PR2
    unsigned char prescaler_bits = 0; // T2CKPS bits value (00 => 1:1, 01 => 1:4, 1x => 1:16)
    unsigned int pr = 0;

    // Try prescaler 1, then 4, then 16
    if (toggle_time_us <= 256) {
        prescaler_bits = 0; // 1:1
        pr = toggle_time_us - 1;
    } else if ((toggle_time_us / 4) <= 256) {
        prescaler_bits = 1; // 1:4
        pr = (toggle_time_us / 4) - 1;
    } else if ((toggle_time_us / 16) <= 256) {
        prescaler_bits = 2; // 1:16
        pr = (toggle_time_us / 16) - 1;
    } else {
        // frequency too low for Timer2 resolution
        return;
    }

    // Configure T2CON prescaler bits
    T2CONbits.T2CKPS = prescaler_bits; // 00, 01 or 10
    // Leave postscaler at 1:1 (TOUTPS = 0000)

    PR2 = (unsigned char)pr;
    TMR2 = 0;
    PIR1bits.TMR2IF = 0;

    // compute number of toggles (each interrupt toggles the pin once -> half-period)
    tone_toggle_count = ((unsigned long)duration_ms * 1000UL) / (unsigned long)toggle_time_us;

    // small settle (optional, helps in practice)
    __delay_us(10);

    // start Timer2
    T2CONbits.TMR2ON = 1;
}

void playNotes (void) {
    tone(262, 500);   //261Hz for 1s
    __delay_ms(500);
    tone(294, 500);
    __delay_ms(500);
    tone(330, 500);
    __delay_ms(500);
    tone(349, 500);
    __delay_ms(500);
    tone(392, 500);
    __delay_ms(500);
    tone(440, 500);
    __delay_ms(500);
    tone(466, 500);
    __delay_ms(500);
    tone(523, 500);
}

void playConfusedTone(void) {
    tone(440, 150);
    __delay_ms(150);
    tone(392, 150);
    __delay_ms(150);
    tone(370, 150);
    __delay_ms(150);
    tone(392, 150);
    __delay_ms(150);
    tone(415, 150);
    __delay_ms(150);
    tone(370, 200);
}

void playHappyTone(void) {
    tone(880, 80);  // C5
    __delay_ms(80);
    tone(988, 80);  // E5
    __delay_ms(80);
    tone(1175, 100);  // G5
    __delay_ms(140);
    tone(1568, 120); // C6
    __delay_ms(120);
    tone(1760, 180); // C6
}

void noise(void) {
    for (int i=50; i<1500; i=i+50) {
        tone(i, 5);
        __delay_ms(5);
        tone((i-20), 5);
        __delay_ms(5);
    }
    
    __delay_ms(500);
    
    for (int i=1500; i>50; i=i-25) {
        tone(i, 5);
        __delay_ms(5);
        tone((i-20), 5);
        __delay_ms(5);
    }
    
    __delay_ms(500);
    //Ah-huh
    for (int i=0; i<20; i++) {
        tone(1500, 5);
        __delay_ms(5);
        tone(1480, 5);
        __delay_ms(5);
    }
    __delay_ms(50);
    for (int i=0; i<10; i++) {
        tone(1550, 5);
        __delay_ms(5);
        tone(1530, 5);
        __delay_ms(5);
    }
    
    __delay_ms(500);
    //uh-huh
    for (int i=0; i<10; i++) {
        tone(750, 5);
        __delay_ms(5);
        tone(730, 5);
        __delay_ms(5);
    }
    __delay_ms(100);
    for (int i=0; i<20; i++) {
        tone(700, 5);
        __delay_ms(5);
        tone(680, 5);
        __delay_ms(5);
    }
}

void main(void) {
    OSCCON = 0b01100010;
    ADCON1 = 0x0F;
    CMCON = 0x07;
    TRISD = 0x00;           //PORTD declared as output
    TRISC = 0xFF;           //PORTC declared as input
    TRISCbits.TRISC6 = 0;   //RC6 made output for USART Tx
    TRISDbits.TRISD3 = 1;   //HC-SR04 Echo
    TRISDbits.TRISD2 = 0;   //HC-SR04 Trigger
    TRISDbits.TRISD1 = 0;   //Output for passive buzzer

    //Timer-1 for HC-SR04
    T1CON = 0x10;
    //Timer-2 for passive buzzer
    T2CON = 0x00;           //Prescaler 1:1, Timer2 off
    PIE1bits.TMR2IE = 1;    // Enable Timer2 interrupt
    PIR1bits.TMR2IF = 0;    // Clear interrupt flag
    
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    
    __delay_ms(500);
    I2C_Init();
    __delay_ms(500);
    SSD1306_Init();
    __delay_ms(500);
    SSD1306_ClearScreen();
    __delay_ms(500);
    USART_Init();
    __delay_ms(100);
    
    unsigned char cmd = 0;
    unsigned char oldCmd = 0;
    unsigned char updateScn = 0;
    unsigned int dist = 0;

    while (1) {
        
        if (cmd != oldCmd) {
            if (cmd == '1') {
                dist = GetDistance();
            }else if (cmd == '2') {
                playNotes();
            }else if (cmd == '3') {
                playConfusedTone();
            }else if (cmd == '4') {
                noise();
            }
            oldCmd = cmd;
            updateScn = 1;
        }
        
        if (updateScn == 1) {
            UpdateScreen(dist);
            updateScn = 0;
        }
        
        if (rxFlag) {
            cmd = rxData;
            SendByte(cmd);
            rxFlag = 0;
        }
        
    }
    
    return;
}