
#include "bluetooth.h"
#include <xc.h>

//******Initialize Bluetooth using USART********//
void Initialize_Bluetooth(void)
{
   //Set the pins of RX and TX//
    TRISCbits.TRISC4 = 1;
    TRISCbits.TRISC5 = 1;
    
  // Switch bit function to TR RX
    // TXCKSEL: Pin Selection bit -> 0: TX/CK function is on RC4
    APFCON0bits.TXCKSEL = 0;
    // RXDTSEL: Pin Selection bit -> 0: TX/CK function is on RC5
    APFCON0bits.RXDTSEL = 0;
  
    //Set the baud rate using the look up table in datasheet(https://ww1.microchip.com/downloads/en/DeviceDoc/40001440E.pdf Table 26-5)//
    BRGH=1;      //Always use high speed baud rate with Bluetooth else it wont work
    SPBRG=207;
    
    //Turn on Asyc. Serial Port//
    SYNC=0;
    SPEN=1;
    
    //Set 8-bit reception and transmission
    RX9=0;
    TX9=0;

   //Enable transmission and reception//
    TXEN=1; 
    CREN=1; 
      
    //Enable interrupts for Tx. and Rx.//
    RCIE=1;
    TXIE=1;

    //Enable global and ph. interrupts//
    //GIE = 1;
    PEIE= 1;
    
    //SSP1STAT =0b00000000;  /* I2C only!  register */
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
}
//___________BT initialized_____________//

//Function to get a char from Rx.buffer of BT//
char BT_get_char(void)   
{
    if(OERR) // check for over run error 
    {
        CREN = 0;
        CREN = 1; //Reset CREN
    }
    
    if(RCIF==1) //if the user has sent a char return the char (ASCII value)
    {
        return RCREG;
    }else{
        return 0;
    }
}

//Function to broadcast data from RX. buffer//
void broadcast_BT(void)
{
  TXREG = 13;  
  __delay_ms(500);
}


//Function to load the Bluetooth Rx. buffer with one char.//
void BT_send_char(char byte)  
{
    //while(!TXIF);  
    while(!TRMT);
    TXREG = byte;
}


//Function to Load Bluetooth Rx. buffer with string//
void BT_send_string(char* string)
{
    while(*string)
    BT_send_char(*string++);
}