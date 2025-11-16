/* 
 * File:   I2C.h
 * Author: sesha
 *
 * Created on 15 September, 2025, 12:01 AM
 */

#ifndef I2C_H
#define	I2C_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>
#include <pic18f4580.h>

void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(unsigned char data);
void I2C_Hold(void);

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

void I2C_RepeatedStart(void) {
    I2C_Hold();
    RSEN = 1;
    while (RSEN);
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

unsigned char I2C_Read(unsigned char ack) {
    unsigned char data;

    I2C_Hold();
    RCEN = 1;          // Enable receive

    while (!SSPSTATbits.BF);   // Wait for buffer full
    data = SSPBUF;

    I2C_Hold();
    ACKDT = (ack) ? 0 : 1;     // ack=1 ? ACK, ack=0 ? NACK
    ACKEN = 1;                 // Send ACK/NACK
    while (ACKEN);

    return data;
}

#ifdef	__cplusplus
}
#endif

#endif	/* I2C_H */

