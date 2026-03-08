/* 
 * File:   PWM.h
 * Author: sesha
 *
 * Created on 8 March, 2026, 11:06 AM
 */

#ifndef PWM_H
#define	PWM_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>
    
int TMR2PRESCALE;

void PWM_Start()
{
	CCP1M0 = 0;
	CCP1M1 = 0;
    CCP1M2 = 1;
    CCP1M3 = 1;
    
	if(TMR2PRESCALE == 1){
		T2CKPS0 = 0;
		T2CKPS1 = 0;
	}else if(TMR2PRESCALE == 4){
		T2CKPS0 = 1;
		T2CKPS1 = 0;
	}else if(TMR2PRESCALE == 16){
		T2CKPS0 = 0;
		T2CKPS1 = 1;
	}
	TMR2ON = 1;
}

void PWM_Stop()
{
	CCP1M0 = 0;
	CCP1M1 = 0;
    CCP1M2 = 0;
    CCP1M3 = 0;
}

void PWM_Init(long xtal, long pwmFreq, int TMR2PRS)
{
    TMR2PRESCALE = TMR2PRS;
    
	PR2 = (xtal/(pwmFreq * 4 * TMR2PRESCALE))-1;
    
	PWM_Start();
}

void PWM_DutyCycle(int percent)
{
	int Duty;
    
    Duty = (percent * 4 * (PR2 + 1)) / 100;
    
	CCPR1L = Duty>>2;
    DC1B0 = Duty & 1;
    DC1B1 = (Duty >> 1) & 1;
}

#ifdef	__cplusplus
}
#endif

#endif	/* PWM_H */

