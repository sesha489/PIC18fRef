/* 
 * File:   EUSART.h
 * Author: sesha
 *
 * Created on 28 September, 2025, 7:58 PM
 */

#ifndef EUSART_H
#define	EUSART_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <xc.h>
#include <pic18f4580.h>

volatile char rxData;
volatile unsigned char rxFlag = 0;

void USART_Init()
{
    
    //Below configuration for 9600 baud rate at 4MHz
	SPBRG = 103;
    SPBRGH = 0;
    BRGH = 1;
    BRG16 = 1;

    SYNC = 0;
    SPEN = 1;
    
    //Enable interrupts
    PIE1bits.RCIE = 1;      //Enable USART Rx interrupt
    INTCONbits.PEIE = 1;    //Enable peripheral interrupt
    INTCONbits.GIE = 1;     //Enable global interrupt
    
    RX9 = 0;
    CREN = 1;
    
	TX9 = 0;
    TXEN = 1;

    /*Above configuration can be set to registers as below
     * TXSTA = 0x24;
     * RCSTA = 0x90;
     * BAUDCON = 0x08;
     */
    
}

void SendByte(unsigned char ch)
{
    while(!TXIF);
    TXREG = ch;
}

void SendString(unsigned char *str) {
    while (*str != '\0') {
        SendByte(*str++);
    }
}

unsigned char GetByte()
{
    while(!RCIF);
    return RCREG;
}

void EUSART_Rx_InterruptHandler (void) {
    rxData = RCREG;
    rxFlag = 1;
}


#ifdef	__cplusplus
}
#endif

#endif	/* EUSART_H */

