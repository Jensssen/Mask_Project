/* 
 * File:   PWM.h
 * Author: soe
 *
 * Created on August 14, 2020, 11:00 AM
 */

#ifndef PWM_H
#define	PWM_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>
#include <stdio.h>

#define _XTAL_FREQ 32000000

 /*---------------------------------------------------------
 Define PWM Interface Function Prototypes
 ---------------------------------------------------------*/    
void pwm_init(void);
void pwm_duty(unsigned int duty);
    
    
    
    

#ifdef	__cplusplus
}
#endif

#endif	/* PWM_H */

