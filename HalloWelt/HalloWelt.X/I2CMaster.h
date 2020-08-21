/* ----------------------------------------------------------
;**  $RCSfile: $
;**  $Source:  $
;**  $Author:  $
;**  $Revision: $
;**  $Date:  $
;**  $State:  $
;** --------------------------------------------------------- */
/* I2C Master driver routine header                    */
/* V1.0 first version from 29.3.2015                   */
/* V1.1 Added support for 2 I2C interfaces (1,2)       */
/*
 * File:   I2CMaster.h
 * Author: Klaus Erichsen
 *
 * Created on 29. September 2014, 19:34
 */

#ifndef I2C_H
#define	I2C_H

#include "globals.h"
#define I2C_WRITE   1
#define I2C_READ    1
#define I2C_RESTART 1

#define I2C_WR      0
#define I2C_RD      1
#define i2c_init     i2c_bbinit
#define i2c_start    i2c_bbstart
#define i2c_restart  i2c_bbrestart
#define i2c_stop     i2c_bbstop
#define i2c_wr       i2c_bbwr
#define i2c_rd       i2c_bbrd

/* Start ReStart and Wr returns ACK flag */
/* Rd function uses ACK flag as Parameter */
/* BitBanging I2C driver */
/*---------------------------------------------------------
 Define BB I2C Hardware
 ---------------------------------------------------------*/
#define SDA0 {LATCbits.LATC1=0;TRISCbits.TRISC1=0;}
#define SDA1 (TRISCbits.TRISC1=1)
#define SDAIN PORTCbits.RC1
#define SCL0 {LATAbits.LATA2=0;TRISAbits.TRISA2=0;}
#define SCL1 {TRISAbits.TRISA2=1;/*while (PORTAbits.RA2==0);*/}
/*---------------------------------------------------------
 Define BB I2C Interface Function Prototypes
 ---------------------------------------------------------*/
void i2c_bbinit(void);
BYTE i2c_bbstart(BYTE addr,BYTE rnw);
BYTE i2c_bbrestart(BYTE addr); /* allways RD */
void i2c_bbstop(void);
BYTE i2c_bbwr(BYTE);
unsigned char i2c_bbrd(BYTE ack);

/* Start ReStart and Wr returns ACK flag */
/* Rd function uses ACK flag as Parameter */
/* MSSP Hardware I2C Driver Stalls on PIC18F25K22 */
/*---------------------------------------------------------
 Define MSSP1 I2C Interface Function Prototypes
 ---------------------------------------------------------*/
void i2c1_init(void);
BYTE i2c1_start(BYTE addr,BYTE rnw);
BYTE i2c1_restart(BYTE addr); /* allways RD */
void i2c1_stop(void);
BYTE i2c1_wr(BYTE);
BYTE i2c1_rd(BYTE ack);

/* Start ReStart and Wr returns ACK flag */
/* Rd function uses ACK flag as Parameter */
/* MSSP Hardware I2C Driver Stalls on PIC18F25K22 */
/*---------------------------------------------------------
 Define MSSP2 I2C Interface Function Prototypes
 ---------------------------------------------------------*/
void i2c2_init(void);
BYTE i2c2_start(BYTE addr,BYTE rnw);
BYTE i2c2_restart(BYTE addr); /* allways RD */
void i2c2_stop(void);
BYTE i2c2_wr(BYTE);
BYTE i2c2_rd(BYTE ack);


#ifdef	__cplusplus
extern "C" {
#endif
#ifdef	__cplusplus
}
#endif

#endif	/* I2C_H */

