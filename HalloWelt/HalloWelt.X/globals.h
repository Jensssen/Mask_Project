/* ----------------------------------------------------------
**  $RCSfile: GLOBALS.H $
**  $Source: $
**  $Author: Klaus $
**  $Revision: $
**  $Date:  $
**  $State: Exp $
** ---------------------------------------------------------- */
/*  Ver 1.0   14-01-2014 global definitions
 *  File:   globals.h
 *  Author: Klaus
 *
 *  Created on 14. Februar 2014, 01:24
*/

#ifndef GLOBALS_H
#define	GLOBALS_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifndef __GLOBALS_H
#define __GLOBALS_H
#define UNUSED 0
#undef DEBUG
#undef SIMULATOR
//#pragma warning disable 751
typedef unsigned char  BYTE;
typedef unsigned short WORD;
typedef unsigned long  ULONG;
#define HIGH(UWRD)  (UWRD>>8)
#define LOW(UWRD)   (UWRD&0xFF)

#define _XTAL_FREQ 32000000

/*--------------------------------------------
 * I2C Hardware
---------------------------------------------*/
#define I2C1_HW 0
#define I2C2_HW 0
#define I2C_BB  1

#endif

#ifdef	__cplusplus
}
#endif

#endif	/* GLOBALS_H */

