#include "PWM.h"
#include <pic16f1825.h>

long PWM_freq = 5000;

void pwm_init(void){
    TRISAbits.TRISA2 = 0; // make port pin on RA2 (CCP3) as output

    PR2 = (_XTAL_FREQ/(PWM_freq*4*1)) - 1; //Setting the PR2 formulae using Datasheet // Makes the PWM work in 5KHZ

    CCP3CON = 0b00001100; // 1100 = PWM mode PxA, PxC active-high, PxB, PxD active-high
    
    T2CON = 0b00000100; // (0) Postscaler (0000) = 1:1, Turn on Timer(1), Prescaler (00) = 1    
}

void pwm_duty(unsigned int duty)
{
    //CCPR3Lbits.CCPR3L = 90;
    duty = ((float)duty/1023)*(_XTAL_FREQ/(PWM_freq*1)); // On reducing //duty = (((float)duty/1023)*(1/PWM_freq)) / ((1/_XTAL_FREQ) * TMR2PRESCALE);
    DC3B1 = duty & 1;
    DC3B0 = duty & 2;
    //CCP3X = duty & 1; //Store the 1st bit
    //CCP3Y = duty & 2; //Store the 0th bit
    CCPR3L = duty>>2;// Store the remining 8 bit
}
