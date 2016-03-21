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

#define RISING 1
#define FALLING 0
#define IO_BASE 0x120
#define IO_PORT_0 IO_BASE
#define IO_PORT_1 (IO_BASE + 1)
#define IO_PORT_2 (IO_BASE + 2)
#define IO_PORT_3 (IO_BASE + 3)
#define IO_PORT_4 (IO_BASE + 4)
#define IO_PORT_5 (IO_BASE + 5)
#define IO_PORT_6 (IO_BASE + 6)
#define IO_PORT_7 (IO_BASE + 7)
#define OUT_BIT 48
#define IN_BIT 48
#define HZ100 25
#define TAKING_DATA 30
#define CAGED 31
#define DIO_15 29

  // UIO48 defines for OBC DIO
#define POR 1        // DIO bit 1 used for LN200 Power On Reset (port 0 bit 0)
#define HEARTBEAT 2  // DIO bit 2 (port 0 bit 1)
#define PINGER 8     // DIO bit 8 used for pinger sync (port 0 bit 7)
#define BALLAST 1    // DIO port 1 used for ballast functions (outputs)
#define IN_HW_FAILSAFE 17 // DIO bit 17 (port 2 bit 0)
#define OUTPUTS 0    // DIO port 0 configured as outputs


  // UIO48 #defines for OBC to DAQ DIO interface
#define COMMAND0 30  // Bit 30 used for lsb of DAQ command word (port 3 bit 5 (0-7))
#define COMMAND1 31  // Bit 31 used for msb of DAQ command word (port 3 bit 6 (0-7))
#define SYNC 32      // Bit 32 used for DAQ frame sync (Port 3 bit 7 (0-7))
#define FRAME_HIGH 5 // Port 5 used for high byte of DAQ frame counter
#define FRAME_LOW  4 // Port 4 used for low byte of DAQ frame counter
#define RUN_NUMBER 3 // Port 3 used for DAQ run number indicator (5 lsb only)

#define outportb _outp
#define inportb _inp

#ifndef SET
#define SET 0
#endif

#ifndef RESET
#define RESET 1
#endif

class __far Uio48
{

public:

Uio48();
~Uio48();

void init_io(unsigned io_address);
int read_bit(int bit_number);
void write_bit(int bit_number, int val);
void set_bit(int bit_number);
void clr_bit(int bit_number);
void enab_int(int bit_number, int polarity);
void disab_int(int bit_number);
void clr_int(int bit_number);
int get_int(void);
unsigned port_read(unsigned offset);
void port_write(unsigned offset, int value); 

};

