/*
 * File:   main.c
 * Author: soe
 *
 * Created on July 30, 2020, 2:51 PM
 */

#include "config.h"
#include <stdio.h>
#include <stdlib.h>
#include <xc.h> // include processor files - each processor file is guarded.  
#include "bluetooth.h"
#include "PWM.h"
#include "SE95.h"
//#include "I2CMaster.h"




#define SE95_CONF_PTR  0b00000001
#define SE95_TEMP_PTR  0b00000000
#define TINNEN         0b01001010
#define SE95_CONF_DATA 0b00000000


char gettemp_se95(unsigned char addr)
{
  /* read Thermometer t*/
 unsigned short i;
 unsigned char b;
 unsigned char ak;
 char temp;
 ak=i2c_start(addr, 0);
 
 
 if (ak==0)
 {
  ak+=i2c_wr(SE95_TEMP_PTR);

  ak+=i2c_restart(addr);

  i=i2c_rd(0);
  b=i2c_rd(1);

  i=(i<<8) | b;
 }else i=0x8000; /*Kein Sensor da: unwahrscheinlich kalt */

 i2c_stop();
 
 unsigned char vorzeichen = i >> 15;
 i = i & 0x7fff;
 i = i >> 3;
 temp = i * 0.03125;
 return temp;
}

void putch(char c){
    BT_send_char(c);
}


void main(void) { 
    
    GIE = 0;
    
    //Scope variable declarations//
    int get_value;
    char temp;
    unsigned char ak;
    // Set internal clock speed to 8 MHz (1110)
    OSCCON = 0b11110000;
    
    LATA = 0;
    LATC = 0;
    ANSELA = 0;
    ANSELC = 0;
    //I/O Declarations//
    TRISCbits.TRISC2 = 0; //RC2 as Output PIN (LED)
       
    pwm_init();
    //pwm_duty(512);

    Initialize_Bluetooth(); 
    i2c_init();

while(1) //The infinite lop
    {   
    get_value = BT_get_char(); //Read the char. received via BT
    pwm_duty(20);
    //If we receive a '0'//
    if (get_value=='0')
      {
         RC2 = 0;  // LED OFF
         //BT_load_string("LED turned OFF \n");
         printf("LED turned OFF \n");
      }

    //If we receive a '1'//   
    if (get_value=='1')
        {
        temp=gettemp_se95(TINNEN); /* read internal Temperature */
        printf("Temp = %d\n", temp);
        RC2 = 1;  // LED ON
        }      
    }
}
