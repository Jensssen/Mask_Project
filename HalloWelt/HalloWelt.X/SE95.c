#include <pic16f1825.h>
#include "SE95.h"


#define I2CDLY1 __delay_us(1)
#define I2CDLY  __delay_us(5)

void i2c_init(void) //Begin IIC as master
{
  
  

  SSP1CON1 = 0b00101000; /* enable module, I2C master SSP1ADD=baud rate */
  SSP1CON2 = 0b00000000;  // Holds all registes to send and receive data.
  SSP1CON3 = 0b00000000;
  SSP1ADD  = (_XTAL_FREQ/4/100000)-1; /*  Baud rate = 100Khz */
  SSP1STAT = 0b00000000;  /* I2C only!  register */

  //Set SDA and SCL pins as input pins
  TRISCbits.TRISC1 = 1;  // SDA
  TRISCbits.TRISC0 = 1;  // SCL 
}



/** The read function */
unsigned char  i2c1_rd(unsigned char ackbit)
{
 unsigned char ucByteRead;
 SSP1CON2bits.RCEN = 1   ;      /* enable I2C receive mode(RCEN=0 after 8cks by hw) */
 while(SSP1CON2bits.RCEN);
 ucByteRead = SSP1BUF    ;
 SSP1CON2bits.ACKDT=ackbit;
 SSP1CON2bits.ACKEN=1;
 //while (SSP1CON2bits.ACKEN);    /* Wait for ACK cycle complete */
 __delay_us(10);                /* Wait for RD cycle to complete */
 return ucByteRead;
}

unsigned char i2c1_restart(unsigned char adr)     /* RESTART I2C communication (change to 'reads')*/
{
 SSP1CON2bits.RSEN = 1;         /* REPEATED START bit (cleared by hw in the end) */
 while (SSP1CON2bits.RSEN);     /* wait until Start cond Set */
 SSP1BUF = (adr<<1)|1;          /* ADDRESS with RD  bit set */
 while (SSP1STATbits.BF);       /* wait until 7 ADR bits + RnW are sent */
 //while (SSP1CON2bits.ACKEN);    /* Wait for ACK cycle complete */
 __delay_us(10);                 /* Wait for ACK cycle complete */
 return SSP1CON2bits.ACKSTAT;
}


unsigned char i2c1_start(unsigned char adr,unsigned char rnw)
{/* sets Master Mode Start conditon  */
 unsigned char ak;
 /* and sends address adr including R/W bit           */
 SSP1CON2bits.SEN = 1;          /* START bit (cleared by hw in the end) */
 while (SSP1CON2bits.SEN);      /* wait until Start cond Set */
 SSP1BUF = (adr<<1)|(rnw&1);    /* Set Address + RnW bit */
 while (SSP1STATbits.BF);       /* wait until 7 ADR bits + RnW are sent */
// while (SSP1CON2bits.ACKEN);    /* Wait for ACK cycle complete */
 __delay_us(10);                 /* Wait for ACK cycle complete */
 ak=SSP1CON2bits.ACKSTAT;
 __delay_us(10);                 /* Wait for Start cycle complete */
 return ak;
}

/** Send STOP for the I2C1 COMMUNICATION */
void i2c1_stop(void)       /* STOP I2C communication */
{
 SSP1CON2bits.PEN = 1;    /* STOP bit (cleared by hw in the end) */
 while (SSP1CON2bits.PEN); /* wait until Stop cond set */
}

/** The write function returns ACK bit after Transfer */
unsigned char i2c1_wr(unsigned char i2c_data) // writes a byte in the I2C SLAVE
{
 unsigned char ak;
 SSP1BUF = i2c_data      ;    /* load char in data buffer ; start streaming */
 while (SSP1STATbits.BF);     /* wait until 8 bits are sent */
 //SSP1CON2bits.ACKEN=1;      /* Enable ACK cycle */
 //while (SSP1CON2bits.ACKEN); /* Wait for ACK cycle complete */
 __delay_us(10);              /* Wait for ACK cycle complete */
 ak= SSP1CON2bits.ACKSTAT;
 __delay_us(10);              /* Wait for wr cycle complete */
 return ak;
}



