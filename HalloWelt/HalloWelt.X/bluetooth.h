/* 
 * File:   bluetooth.h
 * Author: soe
 *
 * Created on August 2, 2020, 2:33 PM
 */

#ifndef BLUETOOTH_H
#define	BLUETOOTH_H




#ifdef	__cplusplus
extern "C" {
#endif

#define _XTAL_FREQ 32000000

void Initialize_Bluetooth(void);
char BT_get_char(void);
void broadcast_BT(void);
void BT_send_char(char byte);
void BT_send_string(char* string);

#ifdef	__cplusplus
}
#endif

#endif	/* BLUETOOTH_H */

