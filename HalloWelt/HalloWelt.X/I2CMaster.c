/* ----------------------------------------------------------
;**  $RCSfile: $
;**  $Source:  $
;**  $Author:  $
;**  $Revision: $
;**  $Date:  $
;**  $State:  $
;** --------------------------------------------------------- */
/* I2C Master driver routines                                 */
/* V1.0 first version from 29.3.2014                          */
/* V1.1 Support for MSSP1+2 data_wr returns ACK bit 21.11.2017*/

#include "globals.h"
#include "I2CMaster.h"
#include <xc.h>

#if I2C_BB==1

#define I2CDLY __delay_us(5)
#define I2CDLY1 __delay_us(1)

/* put both Bits into Tristate */
void i2c_bbinit(void)
{
 SCL1;
 I2CDLY1;
 SDA1; /* inherent STOP sequence */
}

/* Send Bit7 of buf */
void i2c_txbit(BYTE buf)
{
 SCL0;
 I2CDLY1;
 if (buf&0x80) SDA1;else SDA0;
 I2CDLY;
 SCL1;
 I2CDLY;
 SCL0;
}
BYTE i2c_rxbit(void)
{
 BYTE b;
 SCL0;
 I2CDLY1;
 SDA1; /* SDA tristate */
 I2CDLY;
 SCL1;
 I2CDLY;
 b= SDAIN;
 SCL0;
 return b;
}


BYTE i2c_bbstart(BYTE adr, BYTE rnw)
{
 BYTE shiftbuf;
 BYTE i;
 /* Start */
 SCL1;
 I2CDLY;
 SDA1;
 I2CDLY;
 SDA0;
 I2CDLY;
 SCL0;
 I2CDLY;
 shiftbuf =(BYTE) ((adr<<1)|(rnw & 1));    /* Set Address + RnW bit */
 for (i=0;i<8;i++)
 {
  i2c_txbit(shiftbuf);
  shiftbuf<<=1;
 }
 i=i2c_rxbit(); /* return ACK status */
 SDA0;
 return i;
}

#if I2C_RESTART==1
BYTE i2c_bbrestart(BYTE adr)
{
 BYTE shiftbuf;
 BYTE i;
 /* REStart= Avoid STOP State Start*/
 SCL0;
 I2CDLY;
 SDA1;
 I2CDLY;
 SCL1;
 I2CDLY;
 SDA0;
 I2CDLY;
 SCL0;
 I2CDLY;
 shiftbuf =(BYTE) (adr<<1|1);    /* Set Address + RD */
 for (i=0;i<8;i++)
 {
  i2c_txbit(shiftbuf);
  shiftbuf<<=1;
 }
 i=i2c_rxbit(); /* return ACK status */
 SDA0;
 return i;
}
#endif

/** Send STOP for the I2C2 COMMUNICATION */
void i2c_bbstop(void)        /* STOP I2C communication */
{
 SDA0;
 I2CDLY1;
 SCL1;
 I2CDLY;
 SDA1;
 I2CDLY;
}
/** The write function returns ACK bit after Transfer*/
BYTE i2c_bbwr(BYTE i2c_data) // writes a byte in the I2C SLAVE
{
 BYTE i;
 for (i=0;i<8;i++)
 {
  i2c_txbit(i2c_data);
  i2c_data<<=1;
 }
 return i2c_rxbit(); /* return ACK status */
}

/** The read function */
#if I2C_READ==1
BYTE i2c_bbrd(BYTE ackbit)
{
 BYTE i;
 BYTE shiftbuf;
 shiftbuf=0;
 for (i=0;i<8;i++)
 {
  shiftbuf<<=1;
  shiftbuf |= i2c_rxbit();
 }
 ackbit<<=7;
 i2c_txbit(ackbit);
 return shiftbuf;
}
#endif

#endif

/* ***************************************************************/
/*  I2C1 interface                                               */
/* ***************************************************************/
#if I2C1_HW==1
void i2c1_init(void)
{
 PMD1bits.MSSP1MD=0; /* Hardware clocks ON */
 /* sets Baudrate and Master Mode bits */
 /* clear SSPN bits and set SSPEM bits in SSPCON1 register*/
 //SSP1ADD   = 0x9D;   // Baud rate = 100Khz
 //SSP1ADD   = 0x79;   // Baud rate = 125Khz
 //SSP1ADD   = 0x31  ; // Baud rate = 300Khz
 SSP1ADD   = (_XTAL_FREQ/4/100000)-1; /*  Baud rate = 100Khz */

 SSP1STAT =0b00000000;  /* I2C only!  register */
 /*          ||||||||
             |||||||+> Buffer SSPBUF is full (rec:DAV) (0=tx:TBE)
             ||||||+-> Update Adresses req.
             |||||+--> R/W Bit in I2C
             ||||+---> 0 (Start bit in Serial modes)
             |||+----> 0 (Stop bit in Serial modes)
             ||+-----> 0= Last Byte was Data, 1= Last transfer was Address
             |+------> SMBUS inputs enabled
             +-------> slew rate control disabled
 */
 SSP1CON1 =0b00111000; /* enable module, I2C master SSP1ADD=baud rate */
   /*        ||||||||
             |||||||+> \1111=10Bit I2C Master; 0111 = 10 bit Slave
             ||||||+-> |1110=7 Bit I2C Master; 0110 = 7 Bit Slave
             |||||+--> |1011=firmware Master Mode
             ||||+---> /1000=Master clk=F/4*(SSPADD+1)
             |||+----> 1= SCL enable Clock; 0= Stretch SCL, Master I2C: Unused
             ||+-----> 1= I2C Pins configured and enabled
             |+------> 1= SSP1BUF was overwritten, 0= no Overflow
             +-------> 1= I2C Collision Bus was not ready for TX.
 */
 SSP1CON2 =0b00000000;
 /*          ||||||||
             |||||||+> SEN  START in Master (Stretch in Slave) bit
             ||||||+-> RSEN Repeated START I2C (Master only)
             |||||+--> PEN  STOP enable (Master only)
             ||||+---> RCEN Receive enable (Master only)
             |||+----> ACKEN Ack Enabled (Master only) ACKDT sent
             ||+-----> ACKDT Acknowledge Data Bit 1=NotAck 0=Ack
             |+------> ACKSTAT Acknowledge Status
             +-------> GCEN General Call Adress (00) received Interrupt ena
 */
 SSP1CON3 =0b00000000;
 /*          ||||||||
             |||||||+> DHEN I2CSlave 1: Clock Streching enabled
             ||||||+-> AHEN I2CSlave Address Hold enabled
             |||||+--> SBCDE Slave Bus Coll Detect Interrupts Enabled
             ||||+---> SDAHT hold Time 0=100ns, 1=300ns after SCL
             |||+----> BOEN Slave only 1:SSPxBUFF is updated even on OV + ACK
             ||+-----> SCIE Start and Restart Cond generates Int
             |+------> PCIE Stop Cond generates INT
             +-------> ACKTIM I2C is in ACK Phase (9th bit)
 */
}


BYTE i2c1_start(BYTE adr,BYTE rnw)
{/* sets Master Mode Start conditon  */
 BYTE ak;
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

/** restart the I2C COMMUNICATION */
#if I2C_RESTART==1
BYTE i2c1_restart(BYTE adr)     /* RESTART I2C communication (change to 'reads')*/
{
 SSP1CON2bits.RSEN = 1;         /* REPEATED START bit (cleared by hw in the end) */
 while (SSP1CON2bits.RSEN);     /* wait until Start cond Set */
 SSP1BUF = (adr<<1)|1;          /* ADDRESS with RD  bit set */
 while (SSP1STATbits.BF);       /* wait until 7 ADR bits + RnW are sent */
 //while (SSP1CON2bits.ACKEN);    /* Wait for ACK cycle complete */
 __delay_us(10);                 /* Wait for ACK cycle complete */
 return SSP1CON2bits.ACKSTAT;
}
#endif

/** Send STOP for the I2C1 COMMUNICATION */
void i2c1_stop(void)       /* STOP I2C communication */
{
 SSP1CON2bits.PEN1 = 1;    /* STOP bit (cleared by hw in the end) */
 while (SSP1CON2bits.PEN); /* wait until Stop cond set */
}

/** The write function returns ACK bit after Transfer */
BYTE i2c1_wr(unsigned char i2c_data) // writes a byte in the I2C SLAVE
{
 BYTE ak;
 SSP1BUF = i2c_data      ;    /* load char in data buffer ; start streaming */
 while (SSP2STATbits.BF);     /* wait until 8 bits are sent */
 //SSP1CON2bits.ACKEN=1;      /* Enable ACK cycle */
 //while (SSP1CON2bits.ACKEN); /* Wait for ACK cycle complete */
 __delay_us(10);              /* Wait for ACK cycle complete */
 ak= SSP1CON2bits.ACKSTAT;
 __delay_us(10);              /* Wait for wr cycle complete */
 return ak;
}

/** The read function */
#if I2C_READ==1
unsigned char  i2c1_rd(BYTE ackbit)
{
 BYTE ucByteRead;
 SSP1CON2bits.RCEN = 1   ;      /* enable I2C receive mode(RCEN=0 after 8cks by hw) */
 while(SSP1CON2bits.RCEN);
 ucByteRead = SSP1BUF    ;
 SSP1CON2bits.ACKDT=ackbit;
 SSP1CON2bits.ACKEN=1;
 while (SSP1CON2bits.ACKEN);    /* Wait for ACK cycle complete */
 __delay_us(10);                /* Wait for RD cycle to complete */
 return ucByteRead;
}
#endif
#endif

/* ***************************************************************/
/*  I2C2 interface                                               */
/* ***************************************************************/


#if I2C2_HW==1
void i2c2_init(void)
{
 PMD1bits.MSSP2MD=0; /* Hardware clocks ON */
 /* sets Baudrate and Master Mode bits */
 /* clear SSPN bits and set SSPEM bits in SSPCON1 register*/
 SSP2ADD     = 39;     /* Baud rate = 100Khz  */
 //SSP2ADD   = 79;     /* Baud rate =  50Khz  */
 //SSP2ADD   =  9;     /* Baud rate = 400Khz  */
 //SSP2ADD   = (_XTAL_FREQ/4/100000)-1; /*  Baud rate = 100Khz */

 SSP2STAT =0b11000000;  /* I2C only!  register */
 /*          ||||||||
             |||||||+> Buffer SSPBUF is full (rec:DAV) (0=tx:TBE)
             ||||||+-> Update Adresses req.
             |||||+--> R/W Bit in I2C
             ||||+---> 0 (Start bit in Serial modes)
             |||+----> 0 (Stop bit in Serial modes)
             ||+-----> 0= Last Byte was Data, 1= Last transfer was Address
             |+------> SMBUS inputs enabled
             +-------> slew rate control disabled
 */
 SSP2CON1 =0b00101000; /* enable module, I2C master SSP1ADD=baud rate */
   /*        ||||||||
             |||||||+> \1111=10Bit I2C Slave S+P Int; 0111 = 10 bit Slave
             ||||||+-> |1110=7 Bit I2C Slave S+P Int; 0110 = 7 Bit Slave
             |||||+--> |1011=firmware Master Mode
             ||||+---> /1000=Master clk=F/4*(SSPADD+1)
             |||+----> 1= SCL enable Clock; 0= Stretch SCL, Master I2C: Unused
             ||+-----> 1= I2C Pins configured and enabled
             |+------> 1= SSPxBUF was overwritten, 0= no Overflow
             +-------> 1= I2C Collision Bus was not ready for TX.
 */
 SSP2CON2 =0b000000000;
 /*          ||||||||
             |||||||+> SEN  START in Master (Stretch in Slave) bit
             ||||||+-> RSEN Repeated START I2C (Master only)
             |||||+--> PEN  STOP enable (Master only)
             ||||+---> RCEN Receive enable (Master only)
             |||+----> ACKEN Ack Enabled (Master only) ACKDT sent
             ||+-----> ACKDT Acknowledge Data Bit 1=NotAck 0=Ack
             |+------> ACKSTAT Acknowledge Status
             +-------> GCEN General Call Adress (00) received Interrupt ena
 */
 SSP2CON3 =0b00001000;
 /*          ||||||||
             |||||||+> DHEN I2CSlave 1: Clock Streching enabled
             ||||||+-> AHEN I2CSlave Address Hold enabled
             |||||+--> SBCDE Slave Bus Coll Detect Interrupts Enabled
             ||||+---> SDAHT hold Time 0=100ns, 1=300ns after SCL
             |||+----> BOEN Slave only 1:SSPxBUFF is updated even on OV + ACK
             ||+-----> SCIE Start and Restart Cond generates Int
             |+------> PCIE Stop Cond generates INT
             +-------> ACKTIM I2C is in ACK Phase (9th bit)
 */
}

BYTE i2c2_start(BYTE adr,BYTE rnw)
{/* sets Master Mode Start conditon  */
 /* and sends address adr including R/W bit           */
 PIR3bits.SSP2IF=0;
 PIR3bits.BCL2IF=0;
 SSP2CON2bits.SEN = 1;          /* START bit (cleared by hw in the end) */
 while (SSP2CON2bits.SEN);      /* wait until Start cond Set */

 PIR3bits.SSP2IF=0;
 PIR3bits.BCL2IF=0;
 SSP2BUF = (adr<<1)|(rnw&1);    /* Set Address + RnW bit */
 while (SSP2STATbits.BF);        /* wait until 7 ADR bits + RnW are sent */
 __delay_us(20);
 PIR3bits.SSP2IF=0;
 PIR3bits.BCL2IF=0;
 return SSP2CON2bits.ACKSTAT;  /*  Start cycle complete */
}

/** restart the I2C COMMUNICATION */
#if I2C_RESTART==1
BYTE i2c2_restart(BYTE adr)     /* RESTART I2C communication (change to 'reads')*/
{
 PIR3bits.SSP2IF=0;
 SSP2CON2bits.RSEN = 1;         /* REPEATED START bit (cleared by hw in the end) */
 while (SSP2CON2bits.RSEN);     /* wait until Start cond Set */
 PIR3bits.SSP2IF=0;
 PIR3bits.BCL2IF=0;
 SSP2BUF = (adr<<1)|1;          /* ADDRESS with RD  bit set */
 while (SSP2STATbits.BF);       /* Wait buffer sent complete */
 __delay_us(20);                /* give time for ACK */
 PIR3bits.SSP2IF=0;
 PIR3bits.BCL2IF=0;
 return SSP2CON2bits.ACKSTAT;
}
#endif

/** Send STOP for the I2C2 COMMUNICATION */
void i2c2_stop(void)        /* STOP I2C communication */
{
 BYTE z;
 z=1;
 SSP2CON2bits.PEN = 1;     /* STOP bit (cleared by hw in the end)*/
 while (SSP2CON2bits.PEN)  /* wait until Stop cond set */
 PIR3bits.SSP2IF=0;
 PIR3bits.BCL2IF=0;
}

/** The write function returns ACK bit after Transfer*/
BYTE i2c2_wr(unsigned char i2c_data) // writes a byte in the I2C SLAVE
{
 BYTE z;
 z=1;
 SSP2BUF = i2c_data;            /* load char in data buffer ; start streaming */
 PIR3bits.SSP2IF=0;
 PIR3bits.BCL2IF=0;
 SSP2CON1bits.WCOL=0;
 while (SSP2STATbits.BF);       /* Wait buffer sent complete */
  __delay_us(20);               /* give time for ACK */
 PIR3bits.SSP2IF=0;
 PIR3bits.BCL2IF=0;
 return SSP2CON2bits.ACKSTAT;
}

/** The read function */
#if I2C_READ==1
BYTE i2c2_rd(BYTE ackbit)
{
 BYTE ucByteRead;
 if (SSP2CON2&0b00011111) LOGBOOK('i');   /* warten  bis Idle */
 GIE=0; //!
 SSP2CON2bits.RCEN = 1   ;      /* enable I2C receive mode(RCEN=0 after 8cks by hw) */
 PIR3bits.SSP2IF=0;
 PIR3bits.BCL2IF=0;
 while(SSP2STATbits.BF==0);
 ucByteRead = SSP2BUF;          /* byte reception complete */
 SSP2CON2bits.ACKDT=ackbit;
 SSP2CON2bits.ACKEN=1;
 while (SSP2CON2bits.ACKEN)     /* Wait for ACK cycle complete */
 PIR3bits.SSP2IF=0;
 PIR3bits.BCL2IF=0;
 GIE=1; //!
 return ucByteRead;
}
#endif
#endif
