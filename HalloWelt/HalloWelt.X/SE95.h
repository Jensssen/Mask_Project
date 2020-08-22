/* 
 * File:   SE95.h
 * Author: soe
 *
 * Created on August 2, 2020, 2:39 PM
 */

#ifndef SE95_H
#define	SE95_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <xc.h>
#include <stdio.h>

#define _XTAL_FREQ 32000000

#define SDA0 {LATAbits.LATA4=0; TRISAbits.TRISA4=0;}
#define SDA1 (TRISAbits.TRISA4=1)
#define SDAIN PORTAbits.RA4
#define SCL0 {LATAbits.LATA1=0;TRISAbits.TRISA1=0;}
#define SCL1 {TRISAbits.TRISA1=1;while (PORTAbits.RA1==0);}


 /*---------------------------------------------------------
 Define I2C Interface Function Prototypes
 ---------------------------------------------------------*/
void i2c_init(void);
unsigned char i2c_restart(unsigned char addr); /* allways RD */
unsigned char i2c_start(unsigned char addr,unsigned char rnw);
void i2c_stop(void);
unsigned char i2c_wr(unsigned char);
unsigned char i2c_rd(unsigned char ackbit);
void I2C_Master_Init(const unsigned long c);
void I2C_Master_Wait();
void I2C_Master_Start();
#ifdef	__cplusplus
}
#endif

#endif	/* SE95_H */

