/*
 * File:   main.c
 * Author: soe
 *
 * Created on July 30, 2020, 2:51 PM
 */

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = OFF      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF         // Low-Voltage Programming Enable (Low-voltage programming enabled)

//#include "config.h"
#include <stdio.h>
#include <stdlib.h>
#include <xc.h> // include processor files - each processor file is guarded.  
#include "bluetooth.h"
//#include "PWM.h"
//#include "SE95.h"
#include "I2CMaster.h"




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
 ak=i2c_bbstart(addr, 0);
 
 
 if (ak==0)
 {
  ak+=i2c_bbwr(SE95_TEMP_PTR);

  ak+=i2c_bbrestart(addr);

  i=i2c_bbrd(0);
  b=i2c_bbrd(1);

  i=(i<<8) | b;
 }else i=0x8000; /*Kein Sensor da: unwahrscheinlich kalt */

 i2c_bbstop();
 
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
    float temp;
    unsigned char ak;
    // Set internal clock speed to 8 MHz (1110)
    OSCCON = 0b11110000;
    
    ANSELA = 0;
    ANSELC = 0;
    //I/O Declarations//
    TRISCbits.TRISC2 = 0; //RC2 as Output PIN (LED)
       
    //pwm_init();
    //pwm_duty(512);
    //Init Bluetooth
    Initialize_Bluetooth(); 


while(1) //The infinite lop
     {   
    
     get_value = BT_get_char(); //Read the char. received via BT

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
              printf("Temp = %f", temp);
              RC2 = 1;  // LED ON
           }      
    }
}
