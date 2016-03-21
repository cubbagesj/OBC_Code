/* UIO48.H

  Copyright 1996 by WinSystems Inc.

  Permission is hereby granted to the purchaser of the WinSystems
  UIO cards and CPU products incorporating the UIO device, to distribute
  any binary file or files compiled using this source code directly or
  in any work derived by the user from this file. In no case may the
  source code, original or derived from this file, be distributed to any
  third party except by explicit permission of WinSystems. This file is
  distributed on an "As-is" basis and no warranty as to performance,
  fitness of purposes, or any other warranty is expressed or implied.
  In no case shall WinSystems be liable for any direct or indirect loss
  or damage, real or consequential resulting from the usage of this
  source code. It is the user's sole responsibility to determine
  fitness for any considered purpose.
*/
/**************************************************************************
* Name  : uio48.h
*
* Project : PCM-UIO48 Software Samples/Examples
*
* Date  : October 30, 1996
*
* Revision: 1.00
*
* Author  : Steve Mottin
*
****************************************************************************
*
* Changes :
*
* Date    Revision  Description
* ________  ________  ______________________________________________
* 10/30/96  1.00    Created
*
*****************************************************************************
*/

#define UIO48BASE 0x120

#define RISING 1
#define FALLING 0

// uio48 dio port numbers
#define FSLOW        0
#define FSHIGH       3
#define FSHIGH2      4

// uio48 dio port bit numbers (to failsafe board)
#define PONDMASK     1   // input
#define FSCONDL      2   // input
#define FSSIGNAL     3   // input
#define DEPTH        4   // input
#define DCPOWER      5   // input
#define EBATT        6   // input
#define PFSSWITCH2   7   // input
#define PFSSWITCH1   8   // input
#define PFSVENTBLOW  9   // output
#define VALVE1      10   // output
#define VALVE2      11   // output
#define VALVE3      12   // output
#define VALVE4      13   // output
#define VALVE5      14   // output
#define VALVE6      15   // output
#define PUMP        16   // output
#define HEARTBEAT   17   // output
//#define HEARTBEAT   19   // output    bad dio bit 17.  DB  8/9/05
#define ERS         18   // output
#define VALVEOFF     1
#define VALVEON      0
#define PUMPOFF      1
#define PUMPON       0
#define BITOFF       1
#define BITON        0
#define ERSBLOW      0
#define ERSNOBLOW    1
#define PFSBLOW      0
#define PFSVENT      1



void init_io(unsigned io_address);
int read_bit(int bit_number);
void write_bit(int bit_number, int val);
void set_bit(int bit_number);
void clr_bit(int bit_number);
void enab_int(int bit_number, int polarity);
void disab_int(int bit_number);
void clr_int(int bit_number);
int get_int(void);
unsigned read_port(int port_number);
